"""录制资源上界和四元数误差；不依赖ROS。"""
import math
import numpy as np


class RecordingReadiness:
    """只决定能否开始录包；录制时长、缓存和关包仍使用原墙钟上界。"""
    def __init__(self):
        self.samples = {'cloud': None, 'imu': None}
        self.last_ros = self.last_wall = self.progress_wall = None
        self.clock_fault = None

    def observe(self, stream, stamp, received_wall):
        self.samples[stream] = (float(stamp), float(received_wall))

    def observe_clock(self, ros_now, received_wall):
        # 只能由真实/clock回调调用；录包轮询不能把过去的推进重新记成“现在”。
        if not math.isfinite(ros_now) or not math.isfinite(received_wall):
            self.clock_fault = 'invalid_clock'
            return
        if self.last_wall is not None and received_wall < self.last_wall:
            self.clock_fault = 'wall_clock_rewound'
        if self.last_ros is not None:
            if ros_now < self.last_ros:
                self.clock_fault = 'ros_clock_rewound'
            elif ros_now > self.last_ros:
                self.progress_wall = received_wall
        self.last_ros, self.last_wall = ros_now, received_wall

    @staticmethod
    def sample_status(mode, ros_now, wall_now, sample):
        ros_age = wall_age = None
        if sample is None:
            reason = 'missing'
        else:
            stamp, received = sample
            if not math.isfinite(received) or not math.isfinite(wall_now):
                reason = 'invalid_wall_clock'
            else:
                wall_age = wall_now-received
                if wall_age < 0:
                    reason = 'wall_rewound'
                elif mode == 'real':
                    # 真实模式保留原先的1墙秒接收租约，不依赖仿真clock。
                    reason = 'ready' if wall_age < 1. else 'wall_stale'
                elif not math.isfinite(stamp) or stamp <= 0:
                    reason = 'invalid_stamp'
                elif not math.isfinite(ros_now):
                    reason = 'invalid_ros_clock'
                else:
                    ros_age = ros_now-stamp
                    if ros_age < 0:
                        reason = 'future_stamp'
                    elif ros_age > 1.5:
                        reason = 'ros_stale'
                    elif wall_age > 10.:
                        reason = 'wall_hard_limit'
                    else:
                        reason = 'ready'
        return {'fresh': reason == 'ready', 'reason': reason,
                'ros_age_s': ros_age, 'wall_age_s': wall_age}

    def status(self, mode, ros_now, wall_now):
        # 此处只读推进接收时刻；单个非零clock值不能证明仿真正在运行。
        clock_reason = 'ready'
        if mode == 'sim':
            if not math.isfinite(ros_now) or not math.isfinite(wall_now):
                self.clock_fault = 'invalid_clock'
            elif self.last_wall is not None and wall_now < self.last_wall:
                self.clock_fault = 'wall_clock_rewound'
            if self.clock_fault:
                clock_reason = self.clock_fault
            elif ros_now <= 0:
                clock_reason = 'clock_not_started'
            elif self.progress_wall is None:
                clock_reason = 'clock_not_advancing'
            elif wall_now-self.progress_wall > .5:
                clock_reason = 'clock_stalled'
        streams = {name: self.sample_status(mode, ros_now, wall_now, sample)
                   for name, sample in self.samples.items()}
        reason = ('bag_mode' if mode == 'bag' else 'invalid_mode' if mode not in ('sim', 'real')
                  else clock_reason)
        if reason == 'ready':
            reason = next((name+'_'+row['reason'] for name, row in streams.items()
                           if not row['fresh']), 'ready')
        return {'ready': reason == 'ready', 'reason': reason, 'streams': streams,
                'clock_reason': clock_reason,
                'clock_progress_age_wall_s': None if self.progress_wall is None else wall_now-self.progress_wall}


def recording_clock_arguments(mode):
    # bag接收时间也必须与传感器header同域，否则生成的回放clock会落在系统纪元。
    return ['--use-sim-time'] if mode in ('sim', 'bag') else []


def recording_limits(duration, cache):
    # 1~30秒，内存缓存1~64MiB；rosbag本身还存在DDS和SQLite开销。
    return max(1., min(30., float(duration))), max(1048576, min(67108864, int(cache)))


def recording_metadata(metadata, returncode):
    """核对真实metadata中的采样内容；存在文件/进程exit0都不能代替有点云和IMU。"""
    result = {'valid_recording': False, 'bag_duration_ns': None, 'bag_duration_s': None,
              'topic_message_counts': {}, 'message_count': None, 'validation_errors': [],
              'validation_scope': 'metadata content and recorder exit; replay not validated'}
    errors = result['validation_errors']
    if returncode != 0:
        errors.append('recorder_exit_not_zero')
    try:
        if not isinstance(metadata, dict):
            raise ValueError('metadata must be a mapping')
        information = metadata['rosbag2_bagfile_information']
        duration = information['duration']['nanoseconds']
        total = information['message_count']
        if type(duration) is not int or duration < 0 or type(total) is not int or total < 0:
            raise ValueError('duration/message_count must be nonnegative integers')
        rows = information['topics_with_message_count']
        if not isinstance(rows, list):
            raise ValueError('topics_with_message_count must be a list')
        counts = {}
        for row in rows:
            name, count = row['topic_metadata']['name'], row['message_count']
            if not isinstance(name, str) or not name.startswith('/') or type(count) is not int or count < 0:
                raise ValueError('topic name/count is invalid')
            counts[name] = counts.get(name, 0)+count
        result.update(bag_duration_ns=duration, bag_duration_s=duration*1e-9,
                      topic_message_counts=counts, message_count=total)
        if sum(counts.values()) != total:
            errors.append('message_count_mismatch')
        if duration <= 0:
            errors.append('nonpositive_duration')
        if counts.get('/course/points_raw', 0) <= 0:
            errors.append('no_pointcloud_messages')
        if counts.get('/course/imu', 0) <= 0:
            errors.append('no_imu_messages')
    except (KeyError, TypeError, ValueError) as error:
        errors.append('invalid_metadata: '+str(error))
    result['valid_recording'] = not errors
    return result


def quaternion_error(actual, expected):
    a, b = np.asarray(actual, dtype=float), np.asarray(expected, dtype=float)
    if a.shape != (4,) or b.shape != (4,) or np.linalg.norm(a) < 1e-9 or np.linalg.norm(b) < 1e-9:
        raise ValueError('四元数必须为非零xyzw四元数组')
    dot = abs(float(np.dot(a / np.linalg.norm(a), b / np.linalg.norm(b))))
    return float(2 * np.arccos(np.clip(dot, 0., 1.)))


def compare_extrinsics(translation, quaternion, expected_translation, expected_quaternion,
                       translation_tolerance_m, rotation_tolerance_deg):
    """生成可写入TF证据的外参误差，数据边界在这里转换为Python原生值。"""
    error = float(np.linalg.norm(np.asarray(translation, dtype=float)-
                                 np.asarray(expected_translation, dtype=float)))
    angle_deg = float(np.rad2deg(quaternion_error(quaternion, expected_quaternion)))
    return {'translation_error_m': error, 'rotation_error_deg': angle_deg,
            'matches_expected': bool(error <= float(translation_tolerance_m) and
                                     angle_deg <= float(rotation_tolerance_deg))}

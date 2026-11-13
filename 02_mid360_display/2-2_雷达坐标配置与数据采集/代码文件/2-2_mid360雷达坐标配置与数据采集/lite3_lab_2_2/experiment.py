"""3-2：外参验算与有上界的rosbag2录制，使用SIGINT写完索引后关闭。"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Imu
from rosgraph_msgs.msg import Clock as ClockMessage
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from lite3_course_common.ros_utils import result_directory, write_json
from .algorithms import recording_limits, compare_extrinsics, recording_clock_arguments, RecordingReadiness, recording_metadata


class FramesRecordingExperiment(Node):
    def __init__(self):
        super().__init__('frames_recording_experiment')
        defaults = [('mode', 'real'), ('autostart', True), ('result_dir', ''),
                    ('record', True), ('duration_s', 30.), ('cache_bytes', 16777216),
                    ('max_bagfile_size_bytes', 268435456),
                    ('lidar_translation_m', [.22, 0.0, .175]),
                    ('lidar_quaternion_xyzw', [0., 0., 0., 1.]),
                    ('translation_tolerance_m', .05), ('rotation_tolerance_deg', 5.),
                    ('record_wait_s', 2.), ('max_total_bag_bytes', 536870912)]
        for name, default in defaults:
            self.declare_parameter(name, default)
        self.output = result_directory(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(String, '/course/frames/status', 10)
        self.duration, self.cache = recording_limits(self.get_parameter('duration_s').value,
                                                     self.get_parameter('cache_bytes').value)
        self.started = time.monotonic()
        self.record_started, self.process, self.log = None, None, None
        self.finished = False
        self.readiness, self.ready_since = RecordingReadiness(), None
        self.create_subscription(PointCloud2, '/course/points_raw', self.cloud_ready, qos_profile_sensor_data)
        self.create_subscription(Imu, '/course/imu', self.imu_ready, qos_profile_sensor_data)
        self.create_subscription(ClockMessage, '/clock', self.clock_progress,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.report_data = {}
        from rclpy.clock import Clock, ClockType
        self.create_timer(.5, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def stamp_seconds(stamp):
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000:
            return None
        return stamp.sec + stamp.nanosec*1e-9

    def cloud_ready(self, msg):
        stamp = self.stamp_seconds(msg.header.stamp)
        if msg.width*msg.height and stamp is not None:
            self.readiness.observe('cloud', stamp, time.monotonic())

    def imu_ready(self, msg):
        stamp = self.stamp_seconds(msg.header.stamp)
        if stamp is not None:
            self.readiness.observe('imu', stamp, time.monotonic())

    def clock_progress(self, msg):
        stamp = self.stamp_seconds(msg.clock)
        if stamp is not None:
            self.readiness.observe_clock(stamp, time.monotonic())

    def start_recording(self):
        # 列表形式调用，不执行shell；输出目录唯一，避免覆盖旧证据。
        path = self.output / 'raw_bag'
        if path.exists():
            self.finished = True
            self.get_logger().error('raw_bag已存在，拒绝覆盖，请使用新的result_dir')
            return
        max_size = max(1048576, min(536870912, int(self.get_parameter('max_bagfile_size_bytes').value)))
        command = ['ros2', 'bag', 'record', '-o', str(path), '--storage', 'sqlite3',
                   '--max-cache-size', str(self.cache), '--max-bag-size', str(max_size),
                   '--qos-profile-overrides-path', 'bag_qos.yaml']
        # ament安装后config不在Python模块目录；用包share定位QoS配置。
        from ament_index_python.packages import get_package_share_directory
        command[-1] = str(Path(get_package_share_directory('lite3_lab_3_2')) / 'config' / 'bag_qos.yaml')
        command += recording_clock_arguments(self.get_parameter('mode').value)
        command += ['/course/points_raw', '/course/imu', '/tf', '/tf_static', '/clock']
        if self.get_parameter('mode').value == 'real':
            command += ['/livox/lidar']  # 保留逐点offset的CustomMsg，供4章重估运动补偿。
        self.log = (self.output / 'rosbag_record.log').open('w', encoding='utf-8')
        try:
            self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                            start_new_session=True)
            self.record_started = time.monotonic()
            write_json(self.output / 'record_command.json', {'argv': command, 'duration_s': self.duration,
                       'cache_bytes': self.cache, 'max_bagfile_size_bytes': max_size})
            self.get_logger().info('已启动有界录包；到时自动SIGINT保存metadata。')
        except OSError as exc:
            self.finished = True
            self.log.close()
            self.log = None
            self.get_logger().error('无法启动ros2 bag：' + str(exc))

    def stop_recording(self):
        if self.process is None:
            return
        sigint_requested = self.process.poll() is None
        if sigint_requested:
            # POSIX会话组只包括本节点启动的录包进程，不能pkill所有ros2进程。
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                self.get_logger().error('录包10秒未正常关闭，将终止本进程组；此bag需核查metadata')
                os.killpg(self.process.pid, signal.SIGTERM)
                try:
                    self.process.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=2.)
        metadata_path = self.output / 'raw_bag' / 'metadata.yaml'
        try:
            validation = recording_metadata(yaml.safe_load(metadata_path.read_text(encoding='utf-8')),
                                            self.process.returncode)
        except (OSError, yaml.YAMLError) as error:
            validation = recording_metadata(None, self.process.returncode)
            validation['validation_errors'].insert(0, 'metadata_read_error: '+str(error))
        write_json(self.output / 'record_result.json', {
            'returncode': self.process.returncode,
            'metadata_exists': metadata_path.is_file(),
            'normal_sigint_requested': sigint_requested,
            'elapsed_s': None if self.record_started is None else time.monotonic() - self.record_started,
            'requested_duration_s': self.duration, 'cache_bytes': self.cache, **validation})
        if not validation['valid_recording']:
            self.get_logger().error('录包内容校验失败：'+', '.join(validation['validation_errors']))
        self.process = None
        self.finished = True
        if self.log:
            self.log.close()
            self.log = None

    def check_frames(self):
        result = {}
        for target, source in [('base_link', 'lidar_link'), ('base_link', 'imu_link'),
                               ('base_footprint', 'base_link')]:
            try:
                tf = self.buffer.lookup_transform(target, source, Time())
                t, q = tf.transform.translation, tf.transform.rotation
                row = {'translation_m': [t.x, t.y, t.z], 'quaternion_xyzw': [q.x, q.y, q.z, q.w]}
                if source == 'lidar_link':
                    row.update(compare_extrinsics(row['translation_m'], row['quaternion_xyzw'],
                               self.get_parameter('lidar_translation_m').value,
                               self.get_parameter('lidar_quaternion_xyzw').value,
                               self.get_parameter('translation_tolerance_m').value,
                               self.get_parameter('rotation_tolerance_deg').value))
                result[target + '<-' + source] = row
            except Exception as exc:
                result[target + '<-' + source] = {'error': str(exc)}
        return result

    def tick(self):
        self.report_data = self.check_frames()
        write_json(self.output / 'frames.json', self.report_data)
        self.pub.publish(String(data=json.dumps(self.report_data, ensure_ascii=False)))
        now = time.monotonic()
        mode = self.get_parameter('mode').value
        readiness = self.readiness.status(mode, self.get_clock().now().nanoseconds*1e-9, now)
        readiness['tf_ready'] = all('error' not in row for row in self.report_data.values())
        enabled = self.get_parameter('record').value and self.get_parameter('autostart').value
        if mode == 'bag':
            enabled = False
        if enabled and not self.finished and self.process is None:
            ready = readiness['ready'] and readiness['tf_ready']
            if ready:
                self.ready_since = now if self.ready_since is None else self.ready_since
                if now-self.ready_since >= max(0., self.get_parameter('record_wait_s').value):
                    self.start_recording()
            else:
                self.ready_since = None
        readiness['wait_elapsed_wall_s'] = None if self.ready_since is None else now-self.ready_since
        readiness['recording_started'] = self.process is not None
        write_json(self.output / 'recording_readiness.json', readiness)
        if self.process is not None:
            byte_limit = max(1048576, min(1073741824, int(self.get_parameter('max_total_bag_bytes').value)))
            used_bytes = sum(p.stat().st_size for p in (self.output/'raw_bag').glob('*') if p.is_file())
            if self.process.poll() is not None or time.monotonic() - self.record_started >= self.duration or used_bytes >= byte_limit:
                self.stop_recording()


def main(args=None):
    rclpy.init(args=args)
    node = FramesRecordingExperiment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recording()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""Observe real LIO, record operator-controlled trials, request and verify PCD saves.

This node has no motion publisher, robot network socket, or arm operation.
"""
import csv
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import queue
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry, Path as PathMessage
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from lite3_course_common.ros_utils import result_directory, write_json
from .core import StreamMonitor, loop_metrics, summarize_trials, validate_pcd


class Experiment(Node):
    def __init__(self):
        super().__init__('lab_4_1')
        defaults = dict(mode='real', autostart=False,
                        result_dir=str(Path.home()/'catkin_ws_lite3dog/results/lite3_lab_4_1'/time.strftime('%Y%m%d_%H%M%S')),
                        stale_timeout_s=1.0, imu_samples=100, imu_norm_min_mps2=8.5,
                        imu_norm_max_mps2=11.0, imu_norm_std_max_mps2=0.8,
                        minimum_travel_m=1.0, returned_to_start=False,
                        map_file='~/catkin_ws_lite3dog/maps/robot_dog_auto_map.yaml',
                        map_path='~/catkin_ws_lite3dog/maps/robot_dog_auto_map.yaml')
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if self.get_parameter('minimum_travel_m').value <= 0:
            raise ValueError('minimum_travel_m must be positive')
        self.folder = Path(result_directory(self))
        self.monitor = StreamMonitor(**{k:self.get_parameter(k).value for k in (
            'stale_timeout_s','imu_samples','imu_norm_min_mps2','imu_norm_max_mps2','imu_norm_std_max_mps2')})
        self.active = None
        self.poses = []
        # Display the aligned base_link estimate, never raw camera_init poses.
        # This display buffer is independent of the complete per-trial CSV.
        self.trajectory_poses = deque(maxlen=5000)
        self.trajectory_last_stamp = None
        self.trajectory_last_yaw = None
        self.trajectory_pub = self.create_publisher(
            PathMessage, '/course/trajectory',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        archive = self.folder/'trials.json'
        self.trials = json.loads(archive.read_text(encoding='utf-8')) if archive.is_file() else []
        if not isinstance(self.trials, list) or any(not isinstance(t, dict) for t in self.trials):
            raise ValueError('Existing trials.json is invalid; use a new result_dir')
        self.save_pending = False
        self.save_results = queue.Queue()
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.save_client = self.create_client(Trigger, '/course/save_pcd')
        self.create_service(Trigger, '~/start_trial', self.start_trial)
        self.create_service(Trigger, '~/finish_trial', self.finish_trial)
        self.create_service(Trigger, '~/save_pcd', self.save_pcd)
        self.create_subscription(PointCloud2, '/course/points_raw', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, '/course/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/course/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/course/lio_points', self.on_map, qos_profile_sensor_data)
        health_path = self.folder/'health.csv'
        health_columns = ['stamp_s','ready','initialized','stale_streams','imu_norm_mean_mps2',
                          'imu_norm_std_mps2','cloud_latency_mean_s','imu_latency_mean_s',
                          'odom_latency_mean_s','map_latency_mean_s','health_phase',
                          'imu_stationary_ready','health_reasons']
        # Preserve an existing CSV schema when deliberately reopening a folder.
        if health_path.is_file() and health_path.stat().st_size:
            with health_path.open(newline='', encoding='utf-8') as handle:
                health_columns = next(csv.reader(handle))
        self.health_handle = health_path.open('a', newline='', encoding='utf-8')
        self.health_csv = csv.DictWriter(self.health_handle, fieldnames=health_columns,
                                         extrasaction='ignore')
        if self.health_handle.tell() == 0:
            self.health_csv.writeheader()
        write_json(self.folder/'parameters.json', {k:self.get_parameter(k).value for k in defaults})
        self.create_timer(1.0, self.tick)
        self.tick()
        if self.get_parameter('autostart').value:
            self.start_trial(None, Trigger.Response())
        self.get_logger().info('先静置约10秒；health.json显示ready后，由操作者调用start_trial。')

    def now_s(self):
        return self.get_clock().now().nanoseconds*1e-9

    def observe(self, stream, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        accepted = self.monitor.observe(stream, stamp, self.now_s())
        if self.active is not None and not accepted:
            self.active['data_faults'].add(stream+':invalid_or_stale')
        return accepted, stamp

    def on_cloud(self, msg):
        accepted, _ = self.observe('cloud', msg)
        if accepted and (msg.width*msg.height <= 0 or not msg.header.frame_id):
            self.monitor.records['cloud']['valid_latest'] = False
            if self.active is not None:
                self.active['data_faults'].add('cloud:invalid_values')

    def on_map(self, msg):
        accepted, _ = self.observe('map', msg)
        if accepted and (msg.width*msg.height <= 0 or not msg.header.frame_id):
            self.monitor.records['map']['valid_latest'] = False
            if self.active is not None:
                self.active['data_faults'].add('map:invalid_values')

    def on_imu(self, msg):
        accepted, _ = self.observe('imu', msg)
        if accepted:
            a, w = msg.linear_acceleration, msg.angular_velocity
            valid = (all(math.isfinite(v) for v in (w.x,w.y,w.z))
                     and self.monitor.imu((a.x,a.y,a.z)))
            if not valid:
                self.monitor.invalidate_imu()
                self.monitor.records['imu']['valid_latest'] = False
                if self.active is not None:
                    self.active['data_faults'].add('imu:invalid_values')

    def on_odom(self, msg):
        accepted, stamp = self.observe('odom', msg)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        vals = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        norm = sum(v*v for v in (q.x,q.y,q.z,q.w))
        if (not accepted or msg.header.frame_id != 'odom'
                or not all(math.isfinite(v) for v in vals) or not 0.5 <= norm <= 1.5):
            self.monitor.records['odom']['valid_latest'] = False
            if self.active is not None:
                self.active['data_faults'].add('odom:invalid_pose_or_stamp')
            return
        scale = math.sqrt(norm)
        x,y,z,w = (v/scale for v in (q.x,q.y,q.z,q.w))
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        # Keep at most 5 display samples/s, omitting stationary jitter.
        if (self.trajectory_last_stamp is None
                or stamp-self.trajectory_last_stamp >= 0.2):
            previous = self.trajectory_poses[-1].pose.position if self.trajectory_poses else None
            distance = math.inf if previous is None else math.sqrt(
                (p.x-previous.x)**2+(p.y-previous.y)**2+(p.z-previous.z)**2)
            angle = math.inf if self.trajectory_last_yaw is None else abs(math.atan2(
                math.sin(yaw-self.trajectory_last_yaw), math.cos(yaw-self.trajectory_last_yaw)))
            if distance >= 0.02 or angle >= 0.05:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p.x,p.y,p.z
                pose.pose.orientation.x, pose.pose.orientation.y = x,y
                pose.pose.orientation.z, pose.pose.orientation.w = z,w
                self.trajectory_poses.append(pose)
                self.trajectory_last_stamp = stamp
                self.trajectory_last_yaw = yaw
        if self.active is not None and stamp >= self.active['start_ros_s']:
            self.poses.append((stamp,p.x,p.y,yaw))

    def publish_trajectory(self):
        record = self.monitor.records['odom']
        if (not self.trajectory_poses or not record['valid_latest']
                or record['stamp'] is None
                or not -0.05 <= self.now_s()-record['stamp'] <= self.monitor.timeout
                or not 0 <= time.monotonic()-record['wall'] <= self.monitor.timeout):
            return
        path = PathMessage()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = list(self.trajectory_poses)
        self.trajectory_pub.publish(path)

    def start_trial(self, request, response):
        if self.active is not None:
            response.success, response.message = False, '已有进行中的试验。'
        elif len(self.trials) >= 3:
            response.success, response.message = False, '本目录已记录3轮；额外批次请使用新的result_dir。'
        elif self.get_parameter('returned_to_start').value:
            response.success, response.message = False, '开始前先将returned_to_start设为false。'
        elif not self.monitor.begin_motion(self.now_s()):
            response.success, response.message = False, '四路源数据或SI IMU检查尚未就绪，详见health.json。'
        else:
            self.active = dict(start_ros_s=self.now_s(), start_wall=time.monotonic(), data_faults=set(),
                               health_at_start=self.monitor.motion_initial_health)
            self.poses = []
            response.success, response.message = True, '开始记录；使用原控制流程运动并实际返回地面标记及朝向。'
        return response

    def finish_trial(self, request, response, interrupted=False):
        if self.active is None:
            response.success, response.message = False, '没有进行中的试验。'
            return response
        active = self.active
        # Closing a trial still requires parking and a stationary SI check.
        health = self.monitor.health(self.now_s(), require_stationary=True)
        trial = dict(duration_s=time.monotonic()-active['start_wall'], start_ros_s=active['start_ros_s'],
                     finished_ros_s=self.now_s(), odom_samples=len(self.poses),
                     returned_to_start=bool(self.get_parameter('returned_to_start').value),
                     interrupted=interrupted, health_at_finish=health,
                     data_faults=sorted(active['data_faults']), errors=[],
                     health_at_start=active['health_at_start'])
        try:
            trial.update(loop_metrics(self.poses))
        except ValueError as error:
            trial['errors'].append(str(error))
        if not trial['returned_to_start']:
            trial['errors'].append('操作者未确认实际回到起点和原朝向')
        if not health['ready']:
            trial['errors'].append('结束时源数据不健康')
        if active['data_faults']:
            trial['errors'].append('记录过程中出现数据异常或断流')
        if trial.get('path_length_m', 0) < self.get_parameter('minimum_travel_m').value:
            trial['errors'].append('实际记录路程不足minimum_travel_m')
        if trial.get('odom_max_gap_s', 0) > self.monitor.timeout:
            trial['errors'].append('里程计记录中存在超时空隙')
        if interrupted:
            trial['errors'].append('节点退出，试验未正常结束')
        confirmed = trial['returned_to_start'] and not interrupted
        trial['position_drift_m'] = trial.get('endpoint_separation_m') if confirmed else None
        trial['yaw_drift_rad'] = trial.get('endpoint_yaw_difference_rad') if confirmed else None
        trial['success'] = not trial['errors']
        index = len(self.trials)+1
        if (self.folder/('trial_%02d.json'%index)).exists():
            response.success, response.message = False, '试验编号对应文件已存在，请保留结果并使用新目录。'
            return response
        with (self.folder/('trajectory_%02d.csv'%index)).open('x', newline='', encoding='utf-8') as f:
            writer=csv.writer(f)
            writer.writerow(['stamp_s','x_m','y_m','yaw_rad'])
            writer.writerows(self.poses)
        write_json(self.folder/('trial_%02d.json'%index), trial)
        self.trials.append(trial)
        write_json(self.folder/'trials.json', self.trials)
        write_json(self.folder/'three_trial_summary.json', summarize_trials(self.trials))
        self.active = None
        self.monitor.end_motion()
        response.success, response.message = True, json.dumps(trial, ensure_ascii=False)
        return response

    def save_pcd(self, request, response):
        if self.save_pending or not self.save_client.service_is_ready():
            response.success, response.message = False, '保存正在进行或/course/save_pcd未就绪。'
            return response
        self.save_pending = True
        request_ns = time.time_ns()
        write_json(self.folder/'pcd_save_result.json', dict(success=False, pending=True, requested_unix_ns=request_ns))
        future = self.save_client.call_async(Trigger.Request())
        future.add_done_callback(lambda f:self.saved(f, request_ns))
        response.success, response.message = True, '异步保存请求已发出；以pcd_save_result.json为准。'
        return response

    def saved(self, future, request_ns):
        try:
            answer = future.result()
            if answer is None or not answer.success:
                raise ValueError(answer.message if answer is not None else '保存服务没有响应')
            details = json.loads(answer.message)
            if not isinstance(details, dict) or not details.get('pcd'):
                raise ValueError('保存后端没有返回PCD路径')
            # Complete reload is off the ROS callback thread to avoid sensor backlog.
            task = self.worker.submit(validate_pcd, details['pcd'])
            def complete(result):
                try:
                    value = dict(details, **result.result())
                    value.update(success=True, pending=False, requested_unix_ns=request_ns,
                                 backend_success=True, backend_message=answer.message)
                except Exception as error:
                    value = dict(success=False, pending=False, reload_valid=False, error=str(error),
                                 requested_unix_ns=request_ns)
                self.save_results.put(value)
            task.add_done_callback(complete)
        except Exception as error:
            self.save_results.put(dict(success=False, pending=False, reload_valid=False, error=str(error),
                                       requested_unix_ns=request_ns))

    def drain_saves(self):
        while not self.save_results.empty():
            value = self.save_results.get_nowait()
            write_json(self.folder/'pcd_save_result.json', value)
            self.save_pending = False
            self.get_logger().info(json.dumps(value, ensure_ascii=False))

    def tick(self):
        self.publish_trajectory()
        health = self.monitor.health(self.now_s())
        write_json(self.folder/'health.json', health)
        row = dict(stamp_s=self.now_s(), ready=health['ready'], initialized=health['initialized'],
                   stale_streams=','.join(health['stale_streams']),
                   imu_norm_mean_mps2=health['imu_norm_mean_mps2'],
                   imu_norm_std_mps2=health['imu_norm_std_mps2'],
                   health_phase=health['health_phase'],
                   imu_stationary_ready=health['imu_stationary_ready'],
                   health_reasons=','.join(health['health_reasons']))
        row.update({s+'_latency_mean_s':health['streams'][s]['latency_mean_s']
                    for s in StreamMonitor.STREAMS})
        self.health_csv.writerow(row)
        self.health_handle.flush()
        if self.active is not None and not health['ready']:
            self.active['data_faults'].add('health:unready')
            self.active['data_faults'].update(health['health_reasons'])
        self.drain_saves()

    def destroy_node(self):
        if self.active is not None:
            self.finish_trial(None, Trigger.Response(), interrupted=True)
        self.worker.shutdown(wait=True)
        self.drain_saves()
        if self.save_pending:
            write_json(self.folder/'pcd_save_result.json', dict(success=False, pending=False,
                       reload_valid=False, error='节点在保存服务返回前退出；请检查后端及原日志'))
        self.health_handle.close()
        return super().destroy_node()


def main(args=None):
    # Keep the ROS context alive until trial/save files and node resources close.
    # Terminal SIGINT and launch-forwarded SIGINT may both arrive; neither should
    # interrupt cleanup a second time.
    stopping = False
    previous = {}
    def stop(signum, frame):
        nonlocal stopping
        stopping = True
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.signal(signum, stop)
    node = None
    try:
        node = Experiment()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)

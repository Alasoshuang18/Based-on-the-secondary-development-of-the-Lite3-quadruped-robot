"""Humble原生ActionClient与可审计实验生命周期；后续三个实验复用此层。

控制只交给NavigateToPose及公共cmd_vel适配器。真值回调只写评估记录，
不会发布initialpose、里程计或TF，也不会更改导航目标。
"""
import csv
import json
import math
import os
import time
from collections import deque
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import ParticleCloud
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger, Empty
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from .geometry import StableGate, map_health, weighted_region, yaw_from_quaternion, wrap_angle
from .safety import CancellationGuard, backend_available, real_motion_allowed, fresh_source


class NavigationExperiment(Node):
    def __init__(self, package_name, extra_parameters=None):
        super().__init__(package_name)
        defaults = {
            'mode': 'real', 'autostart': False, 'result_dir': '',
            'enable_real_motion': False, 'site_configuration_verified': False,
            # 示例；现场测得腿、脚、负载运动扫掠边界后修改。padding属于规划裕量。
            'robot_length_m': 1.2, 'robot_width_m': .8, 'footprint_padding_m': .15,
            'map_file': '~/catkin_ws_lite3dog/maps/robot_dog_auto_map.yaml',
            'initial_pose_strategy': 'rviz', 'initial_x': 0.0, 'initial_y': 0.0, 'initial_yaw': 0.0,
            'initial_covariance_xy': 0.04, 'initial_covariance_yaw': 0.0685,
            'map_frame': 'map', 'base_frame': 'base_link',
            'goal_x': 2.0, 'goal_y': 0.0, 'goal_yaw': 0.0,
            'particle_diameter_limit_m': 0.3, 'localization_stable_s': 3.0,
            'data_stale_wall_s': 10.0, 'health_timeout_wall_s': 1800.0,
            'backend_status_timeout_wall_s': .5,
            'data_stale_sim_s': 1.0,
            'goal_timeout_sim_s': 900.0, 'goal_timeout_wall_s': 18000.0,
            'lifecycle_nodes': ['amcl', 'planner_server', 'controller_server', 'bt_navigator'],
            'true_error_limit_m': 0.3,
            'nomotion_update_period_sim_s': 0.5,
        }
        defaults.update(extra_parameters or {})
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.package_name = package_name
        self.backend_status, self.backend_received = {}, None
        self.create_subscription(String,'/course/control/status',self.on_backend,10)
        self.begin_wall = time.monotonic()
        directory = self.p('result_dir')
        if not directory:
            directory = '~/catkin_ws_lite3dog/results/%s/%s' % (package_name, time.strftime('%Y%m%d_%H%M%S'))
        self.result_dir = Path(os.path.expanduser(directory)).resolve()
        self.result_dir.mkdir(parents=True, exist_ok=True)
        if (self.result_dir / 'summary.json').exists() or (self.result_dir / 'raw' / 'events.jsonl').exists():
            raise FileExistsError('结果目录已有运行证据，请指定新的result_dir：%s' % self.result_dir)
        for directory_name in ('raw', 'csv', 'figures', 'screenshots', 'trials'):
            (self.result_dir / directory_name).mkdir(exist_ok=True)
        self.events_file = (self.result_dir / 'raw' / 'events.jsonl').open('w', encoding='utf-8')
        self.truth_raw_file = (self.result_dir / 'raw' / 'ground_truth.csv').open('w', newline='', encoding='utf-8')
        self.truth_raw_csv = csv.writer(self.truth_raw_file)
        self.truth_raw_csv.writerow(['stamp_sec', 'stamp_nanosec', 'frame_id', 'child_frame_id',
                                     'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self.pose_file = (self.result_dir / 'csv' / 'poses.csv').open('w', newline='', encoding='utf-8')
        self.pose_csv = csv.writer(self.pose_file)
        self.pose_csv.writerow(['sim_s', 'wall_monotonic_s', 'truth_source_frame', 'truth_x', 'truth_y',
                                'truth_yaw', 'estimate_x', 'estimate_y', 'estimate_yaw',
                                'actual_pose_error_m', 'actual_yaw_error_rad'])
        self.feedback_file = (self.result_dir / 'csv' / 'feedback.csv').open('w', newline='', encoding='utf-8')
        self.feedback_csv = csv.writer(self.feedback_file)
        self.feedback_csv.writerow(['sim_s', 'wall_monotonic_s', 'distance_remaining_m', 'navigation_s',
                                    'eta_s', 'recoveries'])
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, durable)
        self.create_subscription(Odometry, '/course/odom', self.on_odom, sensor)
        if self.p('mode') == 'sim':
            self.create_subscription(Odometry, '/course/ground_truth', self.on_truth, sensor)
        self.create_subscription(ParticleCloud, '/particle_cloud', self.on_particles, sensor)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 10)
        self.create_subscription(NavPath, '/plan', self.on_plan, 10)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.stop_pub = self.create_publisher(Twist, '/course/cmd_vel', 10)
        self.emergency_client = self.create_client(Trigger, '/course/emergency_stop')
        self.emergency_pending = None
        self.emergency_request_wall = None
        self.last_emergency_attempt_wall = -1e9
        self.stop_state_serialized = None
        self.stop_guard = CancellationGuard(10.0)
        self.create_service(Trigger, '~/' + 'cancel', self.cancel_service)
        self.create_service(Trigger, '~/' + 'start', self.start_service)
        self.start_requested = self.p('autostart')
        self.nomotion_client = self.create_client(Empty, 'request_nomotion_update')
        self.nomotion_pending = False
        self.last_nomotion_sim = -1e9
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.lifecycle = {name: self.create_client(GetState, name + '/get_state')
                          for name in self.p('lifecycle_nodes')}
        self.lifecycle_active = {name: False for name in self.lifecycle}
        self.lifecycle_pending = set()
        self.lifecycle_request_wall = {}
        self.last_lifecycle_poll = 0.0
        self.latest_map = None
        self.map_report = {'healthy': False, 'reasons': ['尚未收到/map']}
        self.map_yaml_report = self.inspect_map_yaml()
        self.odom_wall = None
        self.odom_stamp = None
        self.estimate_file = (self.result_dir / 'csv' / 'estimated_trajectory.csv').open('w', newline='', encoding='utf-8')
        self.estimate_csv = csv.writer(self.estimate_file)
        self.estimate_csv.writerow(['ros_unix_s', 'wall_monotonic_s', 'estimated_map_x', 'estimated_map_y', 'estimated_map_yaw'])
        self.latest_truth = None
        self.truth_queue = deque(maxlen=600)
        self.last_truth_stamp = None
        self.truth_errors = []
        self.converged_truth_errors = []
        self.truth_transform_failures = 0
        self.particle_report = None
        self.particle_wall = None
        self.particle_stamp = None
        self.initial_seen = False
        self.configured_initial_sent = False
        self.localization_gate = StableGate(self.p('localization_stable_s'))
        self.localization_stable = False
        self.localization_ever_stable = False
        self.latest_path = None
        self.latest_path_wall = None
        self.latest_path_stamp = None
        self.nav_handle = None
        self.nav_sending = False
        self.nav_started = None
        self.nav_result_pending = False
        self.last_result_request_wall = -1e9
        self.cancel_requested = False
        self.cancel_sent = False
        self.cancel_wall = None
        self.last_cancel_attempt_wall = -1e9
        self.terminal_reason = None
        self.started = False
        self.finished = False
        self.navigation_success = False
        self.last_feedback_wall = 0.0
        self.last_health_log = 0.0
        self.event('experiment_created', parameters={key: self.p(key) for key in defaults},
                   map_yaml=self.map_yaml_report, runtime_verification='not_yet_executed')
        self.get_logger().info('结果目录：%s；等待地图、Nav2生命周期、里程计和人工初始位姿' % self.result_dir)
        # 调度使用单调墙钟，暂停/停止/clock时仍能执行墙钟超时和取消；
        # 收敛、停留、速度和仿真响应指标仍显式读取ROS仿真时钟。
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self.tick, clock=self.steady_clock)

    def p(self, name):
        return self.get_parameter(name).value

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, kind, **data):
        row = {'event': kind, 'sim_s': self.sim_now(), 'wall_monotonic_s': time.monotonic()}
        row['ros_s'] = row['sim_s']
        row['clock_domain'] = 'unix_system' if self.p('mode') == 'real' else self.p('mode')
        row.update(data)
        self.events_file.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
        self.events_file.flush()

    def inspect_map_yaml(self):
        path = Path(os.path.expanduser(self.p('map_file')))
        try:
            with path.open(encoding='utf-8') as stream:
                value = yaml.safe_load(stream)
            image = Path(value['image'])
            if not image.is_absolute():
                image = path.parent / image
            reasons = []
            if abs(float(value['resolution']) - 0.05) > 1e-6:
                reasons.append('YAML分辨率不是0.05m')
            if not image.is_file():
                reasons.append('YAML引用图像不存在')
            if len(value['origin']) != 3:
                reasons.append('origin必须为[x,y,yaw]')
            return {'healthy': not reasons, 'path': str(path), 'image': str(image),
                    'resolution_m': value['resolution'], 'reasons': reasons}
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
            return {'healthy': False, 'path': str(path), 'reasons': [str(exc)]}

    def on_map(self, msg):
        self.latest_map = msg
        self.map_report = map_health(msg.data, msg.info.width, msg.info.height, msg.info.resolution)
        if msg.header.frame_id != self.p('map_frame'):
            self.map_report['healthy'] = False
            self.map_report['reasons'].append('地图frame与map_frame不一致')

    def on_odom(self, msg):
        wall = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.p('mode') == 'real' and not fresh_source(stamp, wall, self.sim_now(), wall, .5, .5, self.odom_stamp):
            self.event('odom_original_stamp_rejected', stamp_ros_s=stamp)
            return
        self.odom_wall, self.odom_stamp = wall, stamp
        if self.p('mode') == 'real':
            try:
                estimate = self.estimate_xyyaw(Time.from_msg(msg.header.stamp))
                self.estimate_csv.writerow([stamp, wall] + list(estimate))
                self.estimate_file.flush()
            except TransformException:
                pass  # 缺同时间TF不补造pose，原始odom仍由rosbag保存。

    def on_truth(self, msg):
        if self.finished or self.p('mode') != 'sim':
            return
        self.latest_truth = (msg, time.monotonic())
        self.truth_queue.append(self.latest_truth)
        pose = msg.pose.pose
        self.truth_raw_csv.writerow([msg.header.stamp.sec, msg.header.stamp.nanosec,
            msg.header.frame_id, msg.child_frame_id, pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        self.truth_raw_file.flush()

    def on_particles(self, msg):
        wall = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.p('mode') == 'real' and not fresh_source(stamp, wall, self.sim_now(), wall, 1.5, 1.5, self.particle_stamp):
            self.event('particle_original_stamp_rejected', stamp_ros_s=stamp)
            self.particle_report = {'valid': False, 'error': 'original_stamp_stale_future_or_reordered'}
            return
        self.particle_wall, self.particle_stamp = wall, stamp
        try:
            if msg.header.frame_id != self.p('map_frame'):
                raise ValueError('particle_cloud必须在map中')
            self.particle_report = weighted_region(
                [[p.pose.position.x, p.pose.position.y] for p in msg.particles],
                [p.weight for p in msg.particles])
        except ValueError as exc:
            self.particle_report = {'valid': False, 'error': str(exc)}

    def on_initialpose(self, msg):
        if msg.header.frame_id == self.p('map_frame'):
            self.initial_seen = True
            self.localization_gate.update(False, self.sim_now())
            self.event('initial_pose_received', source='initialpose_topic',
                       configured_initial_already_published=self.configured_initial_sent,
                       x=msg.pose.pose.position.x, y=msg.pose.pose.position.y)
            if self.package_name == 'lite3_lab_5_1':
                self.get_logger().info('已收到初始位姿 x=%.3f m, y=%.3f m, yaw=%.3f rad，重新计时检查收敛' % (
                    msg.pose.pose.position.x, msg.pose.pose.position.y,
                    yaw_from_quaternion(msg.pose.pose.orientation)))

    def on_plan(self, msg):
        if self.finished:
            return
        if msg.header.frame_id != self.p('map_frame'):
            self.event('plan_rejected_frame', frame=msg.header.frame_id)
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wall = time.monotonic()
        if self.p('mode') == 'real' and not fresh_source(stamp, wall, self.sim_now(), wall, 1.5, 1.5, self.latest_path_stamp):
            self.event('plan_original_stamp_rejected', stamp_ros_s=stamp)
            return
        self.latest_path = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        if not all(math.isfinite(value) for point in self.latest_path for value in point):
            self.event('plan_rejected_nonfinite')
            self.latest_path = None
            return
        self.latest_path_wall = time.monotonic()
        self.latest_path_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.event('nav2_plan_topic', path_xy=self.latest_path, stamp_sec=msg.header.stamp.sec,
                   stamp_nanosec=msg.header.stamp.nanosec)
        self.plan_observed(self.latest_path)

    def plan_observed(self, path):
        """供动态实验读取Nav2实际发布的路线；不把话题当作导航成功。"""

    def poll_lifecycle(self):
        now = time.monotonic()
        if now - self.last_lifecycle_poll < 1.0:
            return
        self.last_lifecycle_poll = now
        for name, client in self.lifecycle.items():
            if not client.service_is_ready():
                self.lifecycle_active[name] = False
                continue
            if name in self.lifecycle_pending:
                if now - self.lifecycle_request_wall[name] > 3.0:
                    self.lifecycle_active[name] = False
                continue
            self.lifecycle_pending.add(name)
            self.lifecycle_request_wall[name] = now
            future = client.call_async(GetState.Request())
            future.add_done_callback(lambda f, key=name: self.lifecycle_reply(key, f))

    def request_sensor_update(self):
        # Humble AMCL静止时不更新粒子云。通过真实激光的无运动更新服务获得新证据，
        # 不能用缓存一帧维持“连续收敛”。此服务既不定位初始化，也不读取真值。
        now = self.sim_now()
        if (self.initial_seen and not self.nomotion_pending and self.nomotion_client.service_is_ready()
                and now - self.last_nomotion_sim >= self.p('nomotion_update_period_sim_s')):
            self.nomotion_pending = True
            self.last_nomotion_sim = now
            self.nomotion_client.call_async(Empty.Request()).add_done_callback(self.nomotion_response)

    def nomotion_response(self, future):
        self.nomotion_pending = False
        try:
            future.result()
        except Exception as exc:
            self.event('nomotion_update_error', error=str(exc))

    def lifecycle_reply(self, name, future):
        self.lifecycle_pending.discard(name)
        try:
            response = future.result()
            self.lifecycle_active[name] = response.current_state.id == 3
        except Exception as exc:
            self.lifecycle_active[name] = False
            self.event('lifecycle_query_error', node=name, error=str(exc))

    def publish_configured_initial_pose(self):
        strategy = self.p('initial_pose_strategy')
        if strategy not in ('rviz', 'configured'):
            self.finish('invalid_initial_pose_strategy')
        if strategy != 'configured' or self.configured_initial_sent or not self.map_report['healthy']:
            return
        if not self.lifecycle_active.get('amcl', False):
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.p('map_frame')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.p('initial_x'), self.p('initial_y')
        msg.pose.pose.orientation.z = math.sin(self.p('initial_yaw') / 2)
        msg.pose.pose.orientation.w = math.cos(self.p('initial_yaw') / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = self.p('initial_covariance_xy')
        msg.pose.covariance[35] = self.p('initial_covariance_yaw')
        self.configured_initial_sent = True
        self.initial_pub.publish(msg)
        self.event('configured_initial_pose_published', origin='user_configuration_only')

    def estimate_xyyaw(self, stamp=None):
        transform = self.tf_buffer.lookup_transform(self.p('map_frame'), self.p('base_frame'), stamp or Time())
        t = transform.transform
        return t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation)

    def transform_truth(self, msg):
        """真值使用自身时间戳转换，不允许以最新估计冒充同一时刻误差。"""
        pose = msg.pose.pose
        quaternion = pose.orientation
        values = [pose.position.x, pose.position.y, pose.position.z,
                  quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('评估真值包含非有限坐标或四元数')
        if abs(sum(value * value for value in values[3:]) - 1.0) > 0.05:
            raise ValueError('评估真值四元数没有正确归一化')
        x, y, yaw = pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)
        if msg.header.frame_id != self.p('map_frame'):
            transform = self.tf_buffer.lookup_transform(self.p('map_frame'), msg.header.frame_id,
                                                        Time.from_msg(msg.header.stamp)).transform
            rotation = yaw_from_quaternion(transform.rotation)
            x, y = (transform.translation.x + math.cos(rotation) * x - math.sin(rotation) * y,
                    transform.translation.y + math.sin(rotation) * x + math.cos(rotation) * y)
            yaw = wrap_angle(yaw + rotation)
        return x, y, yaw

    def evaluate_truth(self):
        # 留下短队列等估计TF跟上消息时间；不能始终只拿最新真值，导致每帧都向未来外推。
        for _ in range(min(20, len(self.truth_queue))):
            msg, received_wall = self.truth_queue[0]
            if time.monotonic() - received_wall > self.p('data_stale_wall_s'):
                self.truth_queue.popleft()
                continue
            if not self.evaluate_truth_sample(msg):
                stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                if self.sim_now() - stamp_s > 2.0:
                    self.truth_queue.popleft()
                    continue
                break
            self.truth_queue.popleft()

    def evaluate_truth_sample(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_truth_stamp:
            return True
        try:
            truth = self.transform_truth(msg)
            estimate = self.estimate_xyyaw(Time.from_msg(msg.header.stamp))
        except (TransformException, ValueError):
            self.truth_transform_failures += 1
            return False
        self.last_truth_stamp = stamp
        error = math.hypot(truth[0] - estimate[0], truth[1] - estimate[1])
        yaw_error = abs(wrap_angle(truth[2] - estimate[2]))
        sim_stamp = stamp[0] + stamp[1] * 1e-9
        self.truth_errors.append(error)
        if (self.localization_stable and self.localization_gate.since is not None
                and sim_stamp >= self.localization_gate.since + self.localization_gate.duration):
            self.converged_truth_errors.append(error)
        self.pose_csv.writerow([sim_stamp, time.monotonic(), msg.header.frame_id] + list(truth) + list(estimate) + [error, yaw_error])
        self.pose_file.flush()
        self.truth_observed(truth, sim_stamp, msg)
        return True

    def truth_observed(self, truth, stamp, msg):
        """供静态净空实验使用。真值接口只从评估链调用。"""

    def health(self):
        now, sim = time.monotonic(), self.sim_now()
        stale = self.p('data_stale_wall_s')
        particle_ok = bool(self.particle_report and 'diameter_m' in self.particle_report
                           and self.particle_report['diameter_m'] <= self.p('particle_diameter_limit_m')
                           and self.particle_wall is not None and now - self.particle_wall <= stale
                           and self.particle_stamp is not None
                           and -0.1 <= sim - self.particle_stamp <= self.p('data_stale_sim_s'))
        self.localization_stable = self.localization_gate.update(particle_ok and self.initial_seen, sim)
        self.localization_ever_stable |= self.localization_stable
        reasons = []
        if not real_motion_allowed(self.p('mode'), self.p('enable_real_motion'), self.p('site_configuration_verified')):
            reasons.append('真机运动未启用或地图目标/全身包络的现场配置尚未核对；bag禁止运动')
        if not backend_available(self.backend_status,self.p('mode'),now,self.backend_received,
                                 self.p('backend_status_timeout_wall_s')):
            reasons.append('连续运动后端尚未认证就绪或状态过期')
        if not self.map_yaml_report['healthy']:
            reasons.append('地图YAML/图像检查未通过')
        if not self.map_report['healthy']:
            reasons.append('在线/map健康检查未通过')
        odom_fresh = (fresh_source(self.odom_stamp, self.odom_wall, sim, now, .5, .5)
                      if self.p('mode') == 'real' else self.odom_wall is not None and now-self.odom_wall <= stale
                      and self.odom_stamp is not None and -.1 <= sim-self.odom_stamp <= self.p('data_stale_sim_s'))
        if not odom_fresh:
            reasons.append('/course/odom缺失或过期')
        if not self.initial_seen:
            reasons.append('尚未收到人工/配置初始位姿')
        if not self.localization_stable:
            reasons.append('加权95%粒子区域未连续收敛')
        if not all(self.lifecycle_active.values()):
            reasons.append('Nav2生命周期节点尚未全部active')
        if not self.nav_client.server_is_ready():
            reasons.append('NavigateToPose服务不可用')
        try:
            self.estimate_xyyaw()
        except TransformException:
            reasons.append('缺少map到base_link的TF')
        return reasons

    def on_backend(self,message):
        try:
            value=json.loads(message.data)
            self.backend_status=value if isinstance(value,dict) else {}
            self.backend_received=time.monotonic()
        except (ValueError,TypeError):
            self.backend_status,self.backend_received={},None

    def goal_pose(self, x=None, y=None, yaw=None):
        msg = PoseStamped()
        msg.header.frame_id = self.p('map_frame')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.p('goal_x') if x is None else float(x)
        msg.pose.position.y = self.p('goal_y') if y is None else float(y)
        angle = self.p('goal_yaw') if yaw is None else float(yaw)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(angle / 2), math.cos(angle / 2)
        return msg

    def goal_cell_free(self, pose):
        if self.latest_map is None:
            return False
        grid = self.latest_map
        origin = grid.info.origin
        yaw = yaw_from_quaternion(origin.orientation)
        dx, dy = pose.pose.position.x - origin.position.x, pose.pose.position.y - origin.position.y
        gx = math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy) / grid.info.resolution)
        gy = math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy) / grid.info.resolution)
        if not (0 <= gx < grid.info.width and 0 <= gy < grid.info.height):
            return False
        value = grid.data[gy * grid.info.width + gx]
        return 0 <= value <= 25

    def send_goal(self, pose):
        if self.nav_sending or self.nav_handle is not None or self.finished:
            return
        if not self.goal_cell_free(pose):
            self.finish('goal_outside_known_free_map')
            return
        request = NavigateToPose.Goal()
        request.pose = pose
        self.nav_sending = True
        self.nav_started = (self.sim_now(), time.monotonic())
        self.cancel_requested, self.cancel_sent = False, False
        self.stop_guard = CancellationGuard(10.0)
        self.event('navigate_goal_requested', x=pose.pose.position.x, y=pose.pose.position.y,
                   yaw=yaw_from_quaternion(pose.pose.orientation))
        future = self.nav_client.send_goal_async(request, feedback_callback=self.feedback)
        future.add_done_callback(self.goal_accepted)

    def goal_accepted(self, future):
        self.nav_sending = False
        try:
            handle = future.result()
        except Exception as exc:
            self.finish('action_send_exception', error=str(exc))
            return
        if not handle.accepted:
            self.navigation_result(GoalStatus.STATUS_ABORTED, 'goal_rejected')
            return
        self.nav_handle = handle
        self.event('navigate_goal_accepted')
        self.request_navigation_result()
        if self.cancel_requested or self.finished:
            self.cancel_navigation(self.terminal_reason or 'cancel_requested')

    def feedback(self, msg):
        if time.monotonic() - self.last_feedback_wall < 0.5:
            return
        self.last_feedback_wall = time.monotonic()
        feedback = msg.feedback
        seconds = lambda duration: duration.sec + duration.nanosec * 1e-9
        self.feedback_csv.writerow([self.sim_now(), time.monotonic(), feedback.distance_remaining,
                                    seconds(feedback.navigation_time), seconds(feedback.estimated_time_remaining),
                                    feedback.number_of_recoveries])
        self.feedback_file.flush()

    def goal_result(self, future):
        self.nav_result_pending = False
        try:
            result = future.result()
        except Exception as exc:
            # 结果RPC失败不代表远端动作结束，保留handle继续取消并锁存停车。
            self.event('action_result_exception', error=str(exc))
            self.stop_guard.fail('action_result_state_unknown')
            self.cancel_navigation('action_result_exception')
            self.maintain_stop()
            return
        if not self.stop_guard.terminal(result.status):
            self.stop_guard.fail('nonterminal_action_result')
            self.cancel_navigation('nonterminal_action_result')
            self.maintain_stop()
            return
        self.nav_handle = None
        self.nav_started = None
        labels = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
                  GoalStatus.STATUS_CANCELED: 'canceled'}
        label = labels.get(result.status, 'unknown_status_%d' % result.status)
        self.event('navigate_action_result', status=result.status, label=label)
        if self.terminal_reason:
            self.finish(self.terminal_reason)
        else:
            self.navigation_result(result.status, label)

    def request_navigation_result(self):
        """结果RPC失效时可重试读取，直到获取真实终态；不推测动作已经停止。"""
        if self.nav_handle is None or self.nav_result_pending:
            return
        self.last_result_request_wall = time.monotonic()
        self.nav_result_pending = True
        try:
            self.nav_handle.get_result_async().add_done_callback(self.goal_result)
        except Exception as exc:
            self.nav_result_pending = False
            self.stop_guard.fail('get_result_request_exception')
            self.cancel_navigation('get_result_request_exception')
            self.event('get_result_request_exception', error=str(exc))

    def navigation_result(self, status, label):
        self.finish(label, successful=status == GoalStatus.STATUS_SUCCEEDED)

    def cancel_navigation(self, reason):
        now = time.monotonic()
        if not self.cancel_requested:
            self.cancel_wall = now
        self.cancel_requested = True
        self.stop_guard.request(now)
        if self.terminal_reason is None:
            self.terminal_reason = reason
        if rclpy.ok():
            self.stop_pub.publish(Twist())
        if (self.nav_handle is not None and not self.stop_guard.terminal_confirmed
                and (not self.cancel_sent or now - self.last_cancel_attempt_wall >= 2.0)):
            self.cancel_sent = True
            self.last_cancel_attempt_wall = now
            self.event('navigate_cancel_requested', reason=reason)
            try:
                self.nav_handle.cancel_goal_async().add_done_callback(self.cancel_response)
            except Exception as exc:
                self.stop_guard.fail('cancel_request_exception')
                self.event('navigate_cancel_error', error=str(exc))

    def cancel_response(self, future):
        try:
            response = future.result()
            target = bytes(self.nav_handle.goal_id.uuid) if self.nav_handle is not None else None
            listed = any(bytes(item.goal_id.uuid) == target for item in response.goals_canceling)
            self.event('navigate_cancel_response', return_code=response.return_code,
                       goals_canceling=len(response.goals_canceling), target_goal_listed=listed)
            # 终态之后到达的取消响应不覆盖已经确认的终态。
            if not self.stop_guard.terminal_confirmed:
                self.stop_guard.cancel_response(response.return_code, listed)
        except Exception as exc:
            self.stop_guard.fail('cancel_response_exception')
            self.event('navigate_cancel_error', error=str(exc))
        self.maintain_stop()

    def maintain_stop(self):
        """即使实验已结束也持续零候选；异常取消升级为公共出口的锁存停止。"""
        now = time.monotonic()
        context_ready = rclpy.ok()
        if self.finished:
            self.stop_guard.emergency_required = True
        old_issues = list(self.stop_guard.issues)
        if self.nav_handle is not None or self.nav_sending:
            self.stop_guard.watchdog(now)
        if self.stop_guard.issues != old_issues:
            self.event('cancellation_watchdog_escalated', issues=self.stop_guard.issues,
                       still_waiting_for_terminal=not self.stop_guard.terminal_confirmed)
            self.get_logger().error('取消未确认终态，持续停车并请求/course/emergency_stop')
        if self.stop_guard.needs_stop(self.finished) and context_ready:
            self.stop_pub.publish(Twist())
        if context_ready and self.cancel_requested and self.nav_handle is not None and not self.stop_guard.terminal_confirmed:
            self.cancel_navigation(self.terminal_reason or 'cancel_requested')
            if not self.nav_result_pending and now - self.last_result_request_wall >= 1.0:
                self.request_navigation_result()
        if self.emergency_pending is not None and now - self.emergency_request_wall > 3.0:
            self.event('emergency_stop_response_timeout')
            self.emergency_pending = None
        if (context_ready and self.stop_guard.needs_emergency(self.finished) and not self.stop_guard.emergency_acknowledged
                and self.emergency_pending is None and now - self.last_emergency_attempt_wall >= 1.0):
            self.last_emergency_attempt_wall = now
            if self.emergency_client.service_is_ready():
                try:
                    self.emergency_request_wall = now
                    self.emergency_pending = self.emergency_client.call_async(Trigger.Request())
                    self.emergency_pending.add_done_callback(self.emergency_stop_response)
                    self.event('emergency_stop_requested', service='/course/emergency_stop')
                except Exception as exc:
                    self.emergency_pending = None
                    self.event('emergency_stop_request_error', error=str(exc))
            else:
                self.event('emergency_stop_service_unavailable', service='/course/emergency_stop')
        state = self.stop_guard.snapshot()
        state.update(experiment_finished=self.finished, goal_inflight=self.nav_handle is not None or self.nav_sending,
                     stop_candidate_continues=context_ready and self.stop_guard.needs_stop(self.finished),
                     ros_context_ready=context_ready,
                     emergency_service='/course/emergency_stop', terminal_reason=self.terminal_reason)
        serialized = json.dumps(state, ensure_ascii=False, indent=2, allow_nan=False)
        if serialized != self.stop_state_serialized:
            self.stop_state_serialized = serialized
            temporary = self.result_dir / 'safety_status.json.tmp'
            temporary.write_text(serialized, encoding='utf-8')
            temporary.replace(self.result_dir / 'safety_status.json')

    def emergency_stop_response(self, future):
        if self.emergency_pending is future:
            self.emergency_pending = None
        try:
            response = future.result()
            self.stop_guard.emergency_acknowledged |= bool(response.success)
            self.event('emergency_stop_response', acknowledged=bool(response.success), message=response.message)
        except Exception as exc:
            self.event('emergency_stop_response_error', error=str(exc))
        self.maintain_stop()

    def cancel_service(self, request, response):
        self.cancel_navigation('user_cancel')
        if self.nav_handle is None and not self.nav_sending:
            self.finish('user_cancel')
        response.success, response.message = True, '已请求取消；以action最终状态和summary为准'
        return response

    def start_service(self, request, response):
        if self.finished:
            response.success, response.message = False, '本次已结束，请重新launch生成新记录'
        else:
            self.start_requested = True
            response.success, response.message = True, '已请求开始，仍须等待健康门控'
        return response

    def begin_experiment(self):
        self.send_goal(self.goal_pose())

    def experiment_tick(self):
        """各实验特有状态机。"""

    def log_localization_progress(self, reasons):
        if self.package_name != 'lite3_lab_5_1':
            return
        if not self.initial_seen:
            progress = '等待初始位姿，请在 C 的 2D Pose Estimate 标记位置和朝向'
        elif not self.particle_report or 'diameter_m' not in self.particle_report:
            progress = '已收到初始位姿，等待有效粒子云 /particle_cloud'
        else:
            elapsed = (max(0., self.sim_now() - self.localization_gate.since)
                       if self.localization_gate.since is not None else 0.)
            stage = '定位已连续收敛' if self.localization_stable else '定位尚未连续收敛'
            progress = '%s | 粒子直径=%.3f/%.3f m | 连续合格=%.1f/%.1f s | 粒子源年龄=%.3f s' % (
                stage, self.particle_report['diameter_m'], self.p('particle_diameter_limit_m'),
                min(elapsed, self.p('localization_stable_s')), self.p('localization_stable_s'),
                self.sim_now() - self.particle_stamp)
        observation_only_reasons = {
            '真机运动未启用或地图目标/全身包络的现场配置尚未核对；bag禁止运动',
            '连续运动后端尚未认证就绪或状态过期',
            '尚未收到人工/配置初始位姿', '加权95%粒子区域未连续收敛',
        }
        pending = [reason for reason in reasons if reason not in observation_only_reasons]
        progress += ' | 其余感知与导航检查=' + ('、'.join(pending) if pending else '就绪')
        self.get_logger().info(progress)

    def tick(self):
        self.maintain_stop()
        if self.finished:
            return
        if self.cancel_requested:
            # 10s看门狗只升级停车，绝不能假装结束后丢弃仍在运行的Nav2动作。
            self.evaluate_truth()
            self.experiment_tick()
            return
        self.poll_lifecycle()
        self.publish_configured_initial_pose()
        self.request_sensor_update()
        self.evaluate_truth()
        reasons = self.health()
        now = time.monotonic()
        if now - self.last_health_log > 2.0:
            self.last_health_log = now
            self.event('health', waiting_for=reasons, map=self.map_report, particle_region=self.particle_report,
                       lifecycle_active=self.lifecycle_active, localization_stable=self.localization_stable)
            self.log_localization_progress(reasons)
        if not self.started:
            if self.start_requested and not reasons:
                self.started = True
                self.event('experiment_started')
                self.begin_experiment()
            elif now - self.begin_wall > self.p('health_timeout_wall_s'):
                self.finish('health_timeout', waiting_for=reasons)
            return
        # 地图/定位/估计流中断时取消导航，不能带病继续并宣称通过。
        if reasons:
            self.cancel_navigation('runtime_health_lost')
            if self.nav_handle is None and not self.nav_sending:
                self.finish('runtime_health_lost', waiting_for=reasons)
            return
        if self.nav_started:
            sim_delta, wall_delta = self.sim_now() - self.nav_started[0], now - self.nav_started[1]
            if sim_delta < 0:
                self.cancel_navigation('ros_system_clock_rewind' if self.p('mode') == 'real' else 'simulation_clock_rewind')
            elif sim_delta > self.p('goal_timeout_sim_s') or wall_delta > self.p('goal_timeout_wall_s'):
                self.cancel_navigation('goal_timeout')
        self.experiment_tick()

    def summary_extra(self):
        return {}

    def finish(self, reason, successful=False, **details):
        if self.finished:
            return
        self.finished, self.navigation_success = True, bool(successful)
        self.maintain_stop()
        summary = {'package': self.package_name, 'mode': self.p('mode'), 'termination_reason': reason,
                   'navigation_success': bool(successful), 'runtime_process_executed': True,
                   'localization_ever_stable': self.localization_ever_stable,
                   'last_particle_region': self.particle_report, 'map_yaml': self.map_yaml_report,
                   'map': self.map_report, 'actual_pose_error_samples': len(self.truth_errors),
                   'actual_pose_error_mean_m': sum(self.truth_errors) / len(self.truth_errors) if self.truth_errors else None,
                   'actual_pose_error_max_m': max(self.truth_errors) if self.truth_errors else None,
                   'actual_pose_error_last_m': self.truth_errors[-1] if self.truth_errors else None,
                   'actual_pose_error_evaluated': bool(self.truth_errors),
                   'converged_actual_pose_error_samples': len(self.converged_truth_errors),
                   'converged_actual_pose_error_max_m': max(self.converged_truth_errors) if self.converged_truth_errors else None,
                   'converged_actual_pose_error_mean_m': sum(self.converged_truth_errors) / len(self.converged_truth_errors) if self.converged_truth_errors else None,
                   'converged_actual_pose_error_within_limit': bool(self.converged_truth_errors and
                       max(self.converged_truth_errors) <= self.p('true_error_limit_m')),
                   'truth_transform_failures': self.truth_transform_failures,
                   'particle_convergence_is_not_truth_accuracy': True,
                   'clock_domain': 'unix_system' if self.p('mode') == 'real' else self.p('mode'),
                   'estimated_trajectory_file': 'csv/estimated_trajectory.csv',
                   'real_absolute_pose_accuracy': 'not_evaluated_without_independent_survey',
                   'safety_guard_at_result': self.stop_guard.snapshot(),
                   'latest_stop_confirmation_file': 'safety_status.json', 'details': details}
        summary.update(self.summary_extra())
        temporary = self.result_dir / 'summary.json.tmp'
        temporary.write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
        temporary.replace(self.result_dir / 'summary.json')
        self.event('experiment_finished', reason=reason, navigation_success=bool(successful))
        self.get_logger().info('实验结束：%s；导航成功=%s；summary=%s' % (reason, successful, self.result_dir / 'summary.json'))

    def close_experiment(self):
        if rclpy.ok():
            if not self.finished:
                self.cancel_navigation('interrupted')
            self.stop_guard.emergency_required = True
            self.maintain_stop()
            # 允许公共出口回复锁存确认；不能仅发异步请求就立即销毁客户端。
            deadline = time.monotonic() + 2.0
            while rclpy.ok() and not self.stop_guard.emergency_acknowledged and time.monotonic() < deadline:
                try:
                    rclpy.spin_once(self, timeout_sec=0.05)
                except (KeyboardInterrupt, ExternalShutdownException):
                    break
            if not self.stop_guard.emergency_acknowledged:
                self.event('shutdown_without_emergency_stop_acknowledgement',
                           terminal_confirmed=self.stop_guard.terminal_confirmed)
        if not self.finished:
            self.finish('interrupted' if rclpy.ok() else 'interrupted_context_shutdown')
        if rclpy.ok():
            self.stop_pub.publish(Twist())
        for file in (self.events_file, self.truth_raw_file, self.pose_file, self.feedback_file, self.estimate_file):
            file.close()


def run_node(factory, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close_experiment()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

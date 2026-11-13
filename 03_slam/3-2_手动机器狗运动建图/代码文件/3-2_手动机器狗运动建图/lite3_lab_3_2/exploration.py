"""Frontier exploration with the 5-1 official velocity supervisor.

The original 5-1 runtime is inherited for evidence readers only. Its AMCL,
native backend health, goal dispatch, cancellation and shutdown are overridden.
No velocity is published by this runtime.
"""
import json
import hashlib
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from explore_lite_msgs.msg import ExploreStatus
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import TransformException
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from lite3_lab_5_1.runtime import NavigationExperiment
from lite3_lab_5_1.safety import fresh_source
from .checkpoint import register_audit
from .motion_evidence import MotionEvidence
from .standstill import PoseStandstill


class ExplorationExperiment(NavigationExperiment):
    def __init__(self, package_name='lite3_lab_5_2', extra_parameters=None):
        values = dict(robot_length_m=.779, robot_width_m=.480, footprint_padding_m=.10,
                      lifecycle_nodes=['planner_server', 'controller_server', 'bt_navigator',
                                       'velocity_smoother'], exploration_seconds=60.,
                      exploration_radius_m=2., max_linear_speed_mps=.20, max_yaw_rate_rps=.20,
                      max_aborted_goals=0 if package_name == 'lite3_lab_5_4' else 3)
        values.update(extra_parameters or {})
        # Audit motion bounds must cover the actual official bridge limits.
        values.update(max_linear_speed_mps=.20, max_yaw_rate_rps=.20)
        super().__init__(package_name, values)
        # Remove inherited legacy output endpoints. Evidence subscribers and
        # the NavigateToPose client remain; all stopping uses the official bridge.
        self.destroy_publisher(self.stop_pub)
        self.destroy_client(self.emergency_client)
        self.destroy_client(self.nomotion_client)
        self.upper, self.upper_at = {}, -math.inf
        self.process = self.process_log = None
        self.explorer_name = package_name + '_explorer'
        self.explorer_ready = False
        self.healthy_since = None
        self.exploration_started_wall = self.exploration_started_ros = None
        self.center = None
        self.task_states, self.preexisting_tasks = {}, set()
        self.preexisting_active = False
        self.reached_goals = set()
        self.aborted_goals = set()
        self.frontiers_exhausted = False
        self.stop_reason = None
        self.stop_at = self.stationary_since = None
        self.stop_future = self.cancel_future = None
        self.stop_ack = self.cancel_ack = False
        self.last_stop_request = -math.inf
        self.measured_speed = self.measured_yaw = math.inf
        self.motion=MotionEvidence(self.result_dir)
        self.standstill=PoseStandstill()
        self.create_subscription(String, '/lab42/upper/status', self.receive_upper, 1)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.tasks, 10)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(ExploreStatus, '/explore/status', self.explore_status, durable)
        self.pause = self.create_publisher(Bool, '/explore/resume', 10)
        self.official_stop = self.create_client(Trigger, '/lab42/upper/stop')
        self.cancel_all = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        register_audit(self.result_dir, int(package_name.rsplit('_',1)[1]))

    def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
        if topic == '/course/odom':
            # State consumers need the newest state, not an old queued history.
            # Raw lidar/IMU and the LIO estimator are untouched.
            qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        return super().create_subscription(msg_type, topic, callback, qos_profile, **kwargs)

    def receive_upper(self, message):
        try:
            self.upper = json.loads(message.data)
            self.upper_at = time.monotonic()
        except (ValueError, TypeError):
            self.upper, self.upper_at = {}, -math.inf

    def on_odom(self, message):
        # Stage 2-4 lease; leave the original 5-1 receiver unchanged.
        wall=time.monotonic()
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        ros=self.sim_now()
        if not fresh_source(stamp,wall,ros,wall,1.,1.,self.odom_stamp):
            self.event('odom_original_stamp_rejected',stamp_ros_s=stamp,source_age_s=ros-stamp)
            return
        self.odom_wall,self.odom_stamp=wall,stamp
        self.odom_age_at_receive=ros-stamp
        v, w = message.twist.twist.linear, message.twist.twist.angular
        self.measured_speed, self.measured_yaw = math.hypot(v.x, v.y), abs(w.z)
        self.motion.observe(message,ros,wall,self.upper,self.upper_at)
        self.standstill.observe_message(message)
        try:
            estimate=self.estimate_xyyaw(Time.from_msg(message.header.stamp))
            self.estimate_csv.writerow([stamp,wall]+list(estimate))
            self.estimate_file.flush()
        except TransformException:
            pass

    def tasks(self, message):
        if not self.started:
            self.preexisting_active = any(item.status in (1,2,3) for item in message.status_list)
        for item in message.status_list:
            key = bytes(item.goal_info.goal_id.uuid).hex()
            if not self.started:
                self.preexisting_tasks.add(key)
                continue
            stamp = item.goal_info.stamp.sec + item.goal_info.stamp.nanosec*1e-9
            if key in self.preexisting_tasks or stamp < self.exploration_started_ros - .05:
                continue
            previous = self.task_states.get(key)
            if previous in (4,5,6) and item.status not in (4,5,6):
                # The result response can arrive before an older status event.
                # A terminal goal cannot become executing again.
                self.event('older_nonterminal_status_ignored',goal_id=key,
                           terminal_status=previous,older_status=item.status)
                continue
            self.task_states[key] = item.status
            if previous == item.status:
                continue
            self.event('exploration_goal_status', goal_id=key, status=item.status)
            self.get_logger().info('导航目标 '+key[:8]+' 状态='+str(item.status)+
                                   '（2=执行中，4=成功，5=取消，6=失败）')
            if item.status == 4:
                self.reached_goals.add(key)
            if item.status == 6 and self.stop_reason is None:
                self.aborted_goals.add(key)
                limit = self.p('max_aborted_goals')
                self.event('frontier_goal_aborted', goal_id=key,
                           aborted_count=len(self.aborted_goals), max_aborted_goals=limit)
                if limit <= 0 or len(self.aborted_goals) >= limit:
                    self.cancel_navigation('nav2_aborted_goal_limit_reached')
                else:
                    # explore_lite already blacklists an aborted frontier and
                    # selects another on its timer. The bridge has no active
                    # goal and stays at zero; health loss still stops the session.
                    self.get_logger().warning('本目标失败已保留；等待探索器更换目标，失败计数 '+
                                              str(len(self.aborted_goals))+'/'+str(limit))

    def explore_status(self, message):
        publishers = self.get_publishers_info_by_topic('/explore/status')
        if len(publishers) != 1 or publishers[0].node_name != self.explorer_name:
            return
        if message.status == 'exploration_paused' and not self.started:
            self.explorer_ready = True
            self.event('explorer_ready_paused')
            self.get_logger().info('探索器已准备好并保持暂停；等待官方桥就绪与显式 start。')
        if self.started:
            self.event('frontier_status', status=message.status)
            if message.status == 'exploration_complete':
                self.frontiers_exhausted = True

    def health(self):
        now, ros = time.monotonic(), self.sim_now()
        reasons = []
        if (self.p('mode') != 'real' or self.get_parameter('use_sim_time').value
                or not self.p('enable_real_motion') or not self.p('site_configuration_verified')):
            reasons.append('real motion/site verification not enabled')
        if not self.started and self.preexisting_active:
            reasons.append('another navigation task is already active')
        if not self.map_report['healthy']:
            reasons.append('live SLAM map unhealthy')
        if not fresh_source(self.odom_stamp, self.odom_wall, ros, now, 1., 1.):
            reasons.append('odom missing or stale')
        if now-self.upper_at > .5 or self.upper.get('mode') != 'ENABLED' or not self.upper.get('ready'):
            reasons.append('official upper bridge not enabled/ready/fresh')
        if self.upper.get('execute') is not True:
            reasons.append('official bridge is in preview mode')
        if (self.upper.get('max_forward_mps') != .20 or self.upper.get('max_yaw_radps') != .20):
            reasons.append('official speed limits differ from 5-1')
        if self.p('max_linear_speed_mps') != .20 or self.p('max_yaw_rate_rps') != .20:
            reasons.append('audit motion bounds differ from the official limits')
        if not all(self.lifecycle_active.values()) or not self.nav_client.server_is_ready():
            reasons.append('Nav2 lifecycle/action not ready')
        if not self.explorer_ready or self.process is None or self.process.poll() is not None:
            reasons.append('explorer warming up in paused mode')
        if set(self.get_node_names()).intersection({'amcl', 'map_server', 'explore', 'explore_node'}):
            # This runtime owns the one explorer process, after start only.
            conflicts = set(self.get_node_names()).intersection({'amcl', 'map_server'})
            if conflicts or not self.started:
                reasons.append('conflicting localization or exploration node')
        if self.get_node_names().count(self.explorer_name) > 1:
            reasons.append('duplicate explorer node')
        if 'slam_toolbox' not in self.get_node_names():
            reasons.append('SLAM node missing')
        if (not math.isfinite(self.measured_speed) or not math.isfinite(self.measured_yaw)
                or self.measured_speed > self.p('max_linear_speed_mps')
                or self.measured_yaw > self.p('max_yaw_rate_rps')):
            reasons.append('measured motion exceeds audit bounds: speed='+format(self.measured_speed,'.4f')+
                           ' m/s, yaw='+format(self.measured_yaw,'.4f')+' rad/s; limits=0.2000/0.2000')
        try:
            for parent, child, future in [('map','odom',.55),('odom','base_footprint',.1)]:
                tf = self.tf_buffer.lookup_transform(parent, child, Time())
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec*1e-9
                if not -future <= ros-stamp <= 1.:
                    reasons.append('SLAM/odometry TF stale')
        except Exception:
            reasons.append('SLAM/odometry TF missing')
        return reasons

    def warmup_explorer(self):
        if (not self.p('enable_real_motion') or not self.p('site_configuration_verified')
                or self.p('mode') != 'real' or not self.map_report['healthy']
                or not self.nav_client.server_is_ready() or self.preexisting_active):
            return
        if any(name in self.get_node_names() for name in ('explore','explore_node',self.explorer_name)):
            return
        from ament_index_python.packages import get_package_prefix
        executable = Path(get_package_prefix('explore_lite'))/'lib/explore_lite/explore'
        feature = json.loads((executable.parent/'paused_start.json').read_text())
        if (feature.get('start_paused') is not True or
                feature.get('sha256') != hashlib.sha256(executable.read_bytes()).hexdigest()):
            raise RuntimeError('Verified paused-start explorer binary is missing; rebuild before running')
        self.process_log = (self.result_dir/'raw/explore.log').open('w')
        self.process = subprocess.Popen([
            str(executable), '--ros-args', '-r', '__node:='+self.explorer_name,
            '-p', 'start_paused:=true',
            '-p', 'robot_base_frame:=base_footprint', '-p', 'costmap_topic:=/map',
            '-p', 'costmap_updates_topic:=/map_updates', '-p', 'return_to_init:=false',
            '-p', 'visualize:=true', '-p', 'planner_frequency:=0.15',
            '-p', 'progress_timeout:=30.0', '-p', 'potential_scale:=3.0',
            '-p', 'gain_scale:=1.0', '-p', 'min_frontier_size:=0.75',
            '-p', 'transform_tolerance:=0.3'], stdout=self.process_log,
            stderr=subprocess.STDOUT, start_new_session=True)
        self.event('explorer_warmup_started', executable=str(executable), start_paused=True)

    def begin_experiment(self):
        if not self.explorer_ready or self.process is None or self.process.poll() is not None:
            raise RuntimeError('Paused explorer is not ready')
        self.exploration_started_wall, self.exploration_started_ros = time.monotonic(), self.sim_now()
        self.center = self.estimate_xyyaw()[:2]
        radius = self.p('exploration_radius_m')
        if not math.isfinite(radius) or radius <= math.hypot(self.p('robot_length_m'), self.p('robot_width_m'))/2+self.p('footprint_padding_m'):
            raise ValueError('exploration radius must contain the complete robot')
        if not 1 <= self.p('exploration_seconds') <= 3600:
            raise ValueError('exploration_seconds must be 1..3600')
        self.pause.publish(Bool(data=True))
        self.event('frontier_exploration_started', center_map=list(self.center),
                   radius_m=radius, seconds=self.p('exploration_seconds'))
        self.get_logger().info('自主探索已恢复：由探索器自动选目标，无需在RViz点击目标。')

    def experiment_tick(self):
        pass

    def cancel_navigation(self, reason):
        if self.stop_reason is None:
            self.stop_reason, self.stop_at = reason, time.monotonic()
            self.cancel_requested = True
            self.event('exploration_stop_requested', reason=reason)
            self.get_logger().warning('Stopping exploration: '+reason)
            self.pause.publish(Bool(data=False))
            if self.process and self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGINT)
        self.maintain_stop()

    def cancel_service(self, request, response):
        # A previous accepted stop does not acknowledge a new stop request.
        self.stop_ack = self.cancel_ack = False
        self.last_stop_request = -math.inf
        self.cancel_navigation('user_cancel')
        response.success, response.message = True, 'Stopping; verify terminal action status, DISABLED and physical standstill'
        return response

    def start_service(self, request, response):
        if self.finished or self.stop_reason is not None:
            response.success, response.message = False, '本轮已结束；请使用新的探索记录目录。'
            return response
        if self.started:
            response.success, response.message = False, '本轮探索已开始，不重复提交。'
            return response
        reasons = self.health()
        if reasons or self.healthy_since is None or time.monotonic()-self.healthy_since < .3:
            response.success = False
            response.message = '未开始，也未排队：'+(', '.join(reasons) if reasons else '等待连续新鲜数据0.30秒')
            return response
        self.start_requested = True
        response.success, response.message = True, '接受本轮开始请求；以探索启动、Action状态和速度记录确认实际执行。'
        return response

    def maintain_stop(self):
        if self.stop_reason is None:
            return
        now = time.monotonic()
        for attr, ack in [('stop_future','stop_ack'), ('cancel_future','cancel_ack')]:
            future = getattr(self, attr)
            if future is not None and future.done():
                try:
                    result = future.result()
                    setattr(self, ack, bool(result and (result.success if ack == 'stop_ack' else result.return_code == 0)))
                    if ack == 'cancel_ack' and result is not None:
                        for goal in result.goals_canceling:
                            key = bytes(goal.goal_id.uuid).hex()
                            self.task_states.setdefault(key,3)
                except Exception:
                    setattr(self, ack, False)
                setattr(self, attr, None)
        if now-self.upper_at <= .5 and self.upper.get('mode') in ('ENABLING','ENABLED'):
            # Also reassert zero/handback if an ended F session is re-enabled.
            self.stop_ack = False
        if now-self.last_stop_request >= 1.:
            self.last_stop_request = now
            if not self.stop_ack and self.stop_future is None and self.official_stop.service_is_ready():
                self.stop_future = self.official_stop.call_async(Trigger.Request())
            if not self.cancel_ack and self.cancel_future is None and self.cancel_all.service_is_ready():
                self.cancel_future = self.cancel_all.call_async(CancelGoal.Request())
        if self.process and self.process.poll() is None and now-self.stop_at > 3:
            os.killpg(self.process.pid, signal.SIGTERM)
        disabled = (now-self.upper_at <= .5 and self.upper.get('mode') == 'DISABLED'
                    and all(abs(self.upper.get(k, math.inf)) < 1e-9 for k in ('vx','vy','wz')))
        motion_window=self.standstill.report(self.sim_now())
        stationary = (fresh_source(self.odom_stamp, self.odom_wall, self.sim_now(), now, 1., 1.)
                      and motion_window['stationary'])
        self.stationary_since = (self.stationary_since or now) if stationary else None
        active = any(code in (1,2,3) for code in self.task_states.values())
        exited = self.process is None or self.process.poll() is not None
        confirmed = bool(self.stop_ack and self.cancel_ack and disabled and not active and exited
                         and self.stationary_since and now-self.stationary_since >= .3)
        safety = dict(official_stop_accepted=self.stop_ack, cancel_all_accepted=self.cancel_ack,
                      active_actions=active, bridge_disabled=disabled, explorer_exited=exited,
                      odometry_standstill=stationary, stop_confirmed=confirmed,
                      physical_stop_confirmed=None,
                      stationary_pose_window=motion_window,
                      estimated_speed_mps=self.measured_speed if math.isfinite(self.measured_speed) else None,
                      estimated_yaw_rate_radps=self.measured_yaw if math.isfinite(self.measured_yaw) else None,
                      stationary_speed_limit_mps=.005,stationary_yaw_limit_radps=.005)
        (self.result_dir/'safety_status.json').write_text(json.dumps(safety, indent=2))
        if not self.finished and (confirmed or now-self.stop_at > 10):
            successful = bool(confirmed and self.reached_goals and self.stop_reason in ('budget_finished','frontiers_exhausted','dynamic_succeeded','manual_goal_succeeded'))
            self.finish(self.stop_reason, successful=successful, safety=safety)

    def tick(self):
        if self.finished:
            self.maintain_stop()
            return
        self.poll_lifecycle()
        if self.stop_reason is not None:
            self.maintain_stop()
            return
        if self.process is None and not self.started:
            try:
                self.warmup_explorer()
            except Exception as exc:
                self.cancel_navigation('explorer_warmup_failed: '+str(exc))
                return
        if self.process is not None and self.process.poll() is not None:
            self.cancel_navigation('explorer_process_exited')
            return
        reasons = self.health()
        now = time.monotonic()
        self.healthy_since = (self.healthy_since or now) if not reasons else None
        if now-self.last_health_log >= 2:
            self.last_health_log = now
            self.event('exploration_health', waiting_for=reasons)
            if not self.started:
                self.get_logger().info('Waiting: '+(', '.join(reasons) if reasons else 'explicit start request'))
        if not self.started:
            if self.start_requested and not reasons and now-self.healthy_since >= .3:
                self.started = True
                try:
                    self.begin_experiment()
                except Exception as exc:
                    self.cancel_navigation('explorer_start_failed: '+str(exc))
            return
        if reasons:
            self.event('runtime_health_lost', waiting_for=reasons,
                       odom_source_age_s=None if self.odom_stamp is None else self.sim_now()-self.odom_stamp,
                       odom_receive_age_s=None if self.odom_wall is None else now-self.odom_wall,
                       measured_motion=self.motion.last,
                       upper_receive_age_s=now-self.upper_at, upper_status=self.upper)
            self.cancel_navigation('runtime_health_lost: '+','.join(reasons))
            return
        xy = self.estimate_xyyaw()[:2]
        extent = math.hypot(self.p('robot_length_m'), self.p('robot_width_m'))/2+self.p('footprint_padding_m')
        # A monitored abort boundary, not a geofence/path containment guarantee.
        if math.dist(xy, self.center)+extent >= self.p('exploration_radius_m'):
            self.cancel_navigation('monitored_radius_reached')
        elif self.process is not None and self.process.poll() is not None:
            self.cancel_navigation('explorer_process_exited')
        elif self.frontiers_exhausted:
            self.cancel_navigation('frontiers_exhausted')
        elif now-self.exploration_started_wall >= self.p('exploration_seconds'):
            self.cancel_navigation('budget_finished')
        else:
            self.experiment_tick()

    def finish(self, reason, successful=False, **details):
        if self.stop_reason is None:
            self.cancel_navigation(reason)
            return
        if self.finished:
            return
        self.navigation_success = successful
        result = dict(package=self.package_name, termination_reason=reason,
                      navigation_success=successful, reached_goals=len(self.reached_goals),
                      aborted_goals=len(self.aborted_goals),
                      all_observed_goals_succeeded=bool(self.task_states and
                                                       all(code == 4 for code in self.task_states.values())),
                      frontier_exhaustion_reported=self.frontiers_exhausted,
                      full_map_coverage_verified=False, motion_backend='official_MotionSender',
                      localization='slam_toolbox_mapping', actual_pose_error_evaluated=False,
                      details=details)
        result['motion_evidence']=self.motion.summary()
        result.update(self.summary_extra())
        temp = self.result_dir/'summary.json.tmp'
        temp.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))
        temp.replace(self.result_dir/'summary.json')
        self.finished=True
        self.event('experiment_finished', reason=reason, navigation_success=successful)
        self.get_logger().info('Exploration ended: '+reason+'; summary='+str(self.result_dir/'summary.json'))

    def close_experiment(self):
        if rclpy.ok() and (self.started or self.process is not None):
            self.cancel_navigation('interrupted')
            deadline = time.monotonic()+11
            while not self.finished and rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.05)
                self.maintain_stop()
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGTERM)
        if self.process_log:
            self.process_log.close()
        for file in (self.events_file, self.truth_raw_file, self.pose_file, self.feedback_file, self.estimate_file):
            file.close()
        self.motion.close()


def run_node(factory, args=None):
    # Keep ROS alive while Ctrl+C/SIGTERM cancels goals and observes the stop.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopping = [False]
    previous = {number:signal.signal(number,lambda *_:stopping.__setitem__(0,True))
                for number in (signal.SIGINT,signal.SIGTERM)}
    node = None
    try:
        node = factory()
        while rclpy.ok() and not stopping[0]:
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        if node is not None:
            node.close_experiment()
            node.destroy_node()
        for number,handler in previous.items():
            signal.signal(number,handler)
        rclpy.try_shutdown()


def main(args=None):
    run_node(ExplorationExperiment, args)

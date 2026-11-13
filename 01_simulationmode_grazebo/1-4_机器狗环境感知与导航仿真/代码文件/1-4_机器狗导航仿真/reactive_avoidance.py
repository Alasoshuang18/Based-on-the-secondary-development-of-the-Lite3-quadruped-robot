#!/usr/bin/python3
"""Live-cloud stop / random clear turn / resume for the current flat world.

Uses the existing gait controller through cmd_vel and stop_gait only. Never
publishes joint trajectories, changes robot poses, or consumes Gazebo truth.
"""
from collections import deque
from datetime import datetime
import json
import math
from pathlib import Path
import random
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from navigation3d_core import SafetyScan, gravity_aligned, xyz_from_cloud, yaw_of

ROOT = Path(__file__).resolve().parent


class Avoidance(Node):
    def __init__(self):
        super().__init__('lite3_reactive_avoidance', namespace='lite3_sim',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.speed = self.declare_parameter('linear_speed', .020).value
        self.turn_speed = self.declare_parameter('angular_speed', .020).value
        self.radius = self.declare_parameter('footprint_radius', .70).value
        self.lookahead = self.declare_parameter('lookahead', .60).value
        self.max_avoidances = self.declare_parameter('max_avoidances', 0).value
        seed = self.declare_parameter('random_seed', -1).value
        self.rng = random.SystemRandom() if seed < 0 else random.Random(seed)
        if not (0 < self.speed <= .020 and 0 < self.turn_speed <= .020 and
                self.max_avoidances >= 0 and .5 <= self.radius <= 1. and .2 <= self.lookahead <= 2.):
            raise ValueError('Avoidance settings outside the qualified simulation envelope')
        self.state, self.enabled, self.reason = 'IDLE', False, 'Call /lite3_sim/avoidance/start'
        self.gait, self.scan = {}, None
        self.cloud_stamp = self.imu_stamp = self.odom_stamp = None
        self.cloud_wall = self.imu_wall = self.odom_wall = self.gait_wall = 0.
        self.imu_q, self.last_yaw, self.yaw = None, None, 0.
        self.map_seen, self.odom_lost = False, True
        self.completed, self.resume_sim = 0, None
        self.after_stop, self.stop_future = 'IDLE', None
        self.stop_wall = self.turn_begin_sim = 0.
        self.target_turn, self.turn_start = 0., 0.
        self.last_report, self.last_command = 0., (0., 0.)
        self.events = deque(maxlen=30)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.command_pub = self.create_publisher(Twist, '/lite3_sim/cmd_vel', 1)
        self.status_pub = self.create_publisher(String, '/lite3_sim/avoidance/status', 1)
        self.stop_client = self.create_client(Trigger, '/lite3_sim/stop_gait')
        self.create_service(Trigger, '/lite3_sim/avoidance/start', self.start)
        self.create_service(Trigger, '/lite3_sim/avoidance/stop', self.stop_request)
        self.create_subscription(String, '/lite3_sim/gait_status', self.gait_status, 1)
        self.create_subscription(Imu, '/lite3_sim/imu', self.imu, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/lite3_sim/mid360/points', self.cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/lite3_sim/slam3d/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(OdomInfo, '/lite3_sim/slam3d/odom_info', self.odom_info, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/lite3_sim/slam3d/cloud_map', self.map_cloud,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))
        folder = ROOT/'.runtime/diagnostics'
        folder.mkdir(parents=True, exist_ok=True)
        self.log = (folder/('avoidance_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.jsonl')).open('w')
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec+message.header.stamp.nanosec*1e-9

    def sim(self):
        return self.get_clock().now().nanoseconds*1e-9

    def gait_status(self, message):
        pieces = message.data.split(';')
        self.gait = {'state': pieces[0].strip(), **dict(
            item.strip().split('=', 1) for item in pieces[1:] if '=' in item)}
        self.gait_wall = time.monotonic()

    def imu(self, message):
        q = message.orientation
        values = (q.x, q.y, q.z, q.w)
        try:
            current = yaw_of(values)
        except ValueError:
            return
        if self.last_yaw is not None:
            self.yaw += (current-self.last_yaw+math.pi)%(2*math.pi)-math.pi
        self.last_yaw, self.imu_q = current, values
        self.imu_stamp, self.imu_wall = self.stamp(message), time.monotonic()

    def cloud(self, message):
        if self.imu_q is None:
            return
        try:
            transform = self.buffer.lookup_transform('TORSO', message.header.frame_id, rclpy.time.Time())
            q, p = transform.transform.rotation, transform.transform.translation
            points = gravity_aligned(xyz_from_cloud(message), (q.x, q.y, q.z, q.w),
                                     (p.x, p.y, p.z), self.imu_q)
            scan = SafetyScan(points, self.radius, self.lookahead)
        except (ValueError, TransformException) as error:
            self.reason = 'Cloud/TF rejected: '+str(error)
            return
        self.scan, self.cloud_stamp, self.cloud_wall = scan, self.stamp(message), time.monotonic()

    def odom(self, message):
        self.odom_stamp, self.odom_wall = self.stamp(message), time.monotonic()

    def odom_info(self, message):
        self.odom_lost = message.lost

    def map_cloud(self, message):
        self.map_seen |= message.width*message.height > 0

    def health(self):
        now, sim = time.monotonic(), self.sim()
        for name, stamp, wall, max_sim in (
                ('cloud', self.cloud_stamp, self.cloud_wall, .7),
                ('IMU', self.imu_stamp, self.imu_wall, .2),
                ('lidar odometry', self.odom_stamp, self.odom_wall, .7)):
            if stamp is None or now-wall > 5 or not -.05 <= sim-stamp <= max_sim:
                return name+' missing/stale'
        if now-self.gait_wall > 3:
            return 'Gait status missing/stale'
        if self.odom_lost or not self.map_seen:
            return 'Lidar SLAM not tracking or no 3-D map received'
        if self.gait.get('state') == 'FAULT':
            return 'Gait FAULT: '+self.gait.get('reason', '')
        if self.count_publishers('/lite3_sim/cmd_vel') != 1:
            return 'Another cmd_vel publisher exists'
        return ''

    def transition(self, state, reason):
        self.state, self.reason = state, reason
        event = dict(sim=self.sim(), state=state, reason=reason,
                     completed=self.completed, target_turn_deg=math.degrees(self.target_turn))
        self.events.append(event)
        self.log.write(json.dumps(event)+'\n'); self.log.flush()
        self.get_logger().info(json.dumps(event))

    def start(self, request, response):
        reason = self.health()
        if not reason and (self.gait.get('state') != 'READY' or self.gait.get('lift_passed') != 'True'):
            reason = 'Wait for gait READY and four-foot lift_passed=True'
        if not reason and self.gait.get('mode') != 'trot':
            reason = 'This .02 m/s profile requires the 6-4 payload E in mode=trot'
        if not reason and self.enabled:
            reason = 'Already enabled'
        response.success = not reason
        response.message = reason or 'Avoidance enabled: inflated footprint, random clear turns'
        if not reason:
            self.enabled, self.completed, self.resume_sim = True, 0, None
            self.publish()
            self.transition('FORWARD', 'Qualified free-body trot and live 3-D lidar tracking')
        return response

    def begin_stop(self, after, reason):
        self.after_stop, self.stop_wall = after, time.monotonic()
        self.publish()
        self.stop_future = (self.stop_client.call_async(Trigger.Request())
                            if self.stop_client.service_is_ready() else None)
        self.transition('BRAKE', reason)

    def stop_request(self, request, response):
        self.enabled = False
        self.begin_stop('IDLE', 'Operator stop requested')
        response.success, response.message = True, 'Stopping; keep E running until READY'
        return response

    def publish(self, x=0., w=0.):
        message = Twist(); message.linear.x = x; message.angular.z = w
        self.command_pub.publish(message)
        self.last_command = x, w

    def tick(self):
        now = time.monotonic()
        reason = self.health() if self.enabled else ''
        if reason and self.state not in ('BRAKE', 'HALTED'):
            self.enabled = False
            self.begin_stop('HALTED', reason)
        if self.state in ('IDLE', 'HALTED', 'BLOCKED'):
            self.publish()
        elif self.state == 'BRAKE':
            self.publish()
            if self.gait.get('state') == 'FAULT':
                self.enabled = False
                self.transition('HALTED', 'Gait FAULT; no automatic re-enable')
            elif (self.gait.get('state') == 'READY' and self.gait_wall > self.stop_wall and
                  self.stop_future is not None and self.stop_future.done()):
                if self.after_stop == 'PLAN':
                    if self.health():
                        self.enabled = False
                        self.transition('HALTED', self.health())
                    else:
                        self.target_turn = self.scan.choose_turn(self.rng)
                        if self.target_turn is None:
                            self.target_turn = 0.
                            self.enabled = False
                            self.transition('BLOCKED', 'No observed clear turn; remain stopped')
                        else:
                            self.turn_start, self.turn_begin_sim = self.yaw, self.sim()
                            self.transition('TURN', 'Random safe direction selected after measured READY')
                elif self.after_stop == 'RESUME':
                    self.completed += 1
                    self.resume_sim = self.sim()
                    self.transition('FORWARD', 'Turn finished; fresh zero and resume')
                else:
                    self.transition(self.after_stop, 'Measured gait READY after stop')
            elif now-self.stop_wall > 40:
                self.enabled = False
                self.transition('HALTED', 'Stop did not reach READY within 40 wall seconds')
        elif self.state == 'FORWARD':
            if not self.scan.clear_path(0.):
                self.begin_stop('PLAN', 'Front capsule blocked or unobserved; stop before turning')
            elif (self.max_avoidances and self.completed >= self.max_avoidances and
                  self.resume_sim is not None and self.sim()-self.resume_sim >= 20.):
                self.enabled = False
                self.begin_stop('IDLE', 'Bounded acceptance complete after turn and 20 s resumed walking')
            else:
                self.publish(self.speed)
        elif self.state == 'TURN':
            progress = (self.yaw-self.turn_start)*math.copysign(1., self.target_turn)
            if self.scan.clearance <= .10:
                self.enabled = False
                self.begin_stop('HALTED', 'Rotation footprint no longer clear')
            elif self.sim()-self.turn_begin_sim > 250:
                self.enabled = False
                self.begin_stop('HALTED', 'Measured turn deadline exceeded')
            elif progress >= abs(self.target_turn)-.02:
                self.begin_stop('RESUME', 'Measured relative IMU turn reached target')
            else:
                self.publish(0., math.copysign(self.turn_speed, self.target_turn))
        if now-self.last_report >= .5:
            self.last_report = now
            report = dict(sim=self.sim(), state=self.state, enabled=self.enabled, reason=self.reason,
                          gait=self.gait.get('state'), gait_mode=self.gait.get('mode'),
                          completed=self.completed, target_turn_deg=math.degrees(self.target_turn),
                          turn_progress_deg=math.degrees(self.yaw-self.turn_start),
                          clearance_m=self.scan.clearance if self.scan else None,
                          front_coverage=self.scan.coverage(0.) if self.scan else None,
                          obstacles=len(self.scan.obstacles) if self.scan else 0,
                          command_v=self.last_command[0], command_w=self.last_command[1],
                          cloud_stamp=self.cloud_stamp, odom_lost=self.odom_lost,
                          map_seen=self.map_seen, recent_events=list(self.events))
            message = String(); message.data = json.dumps(report); self.status_pub.publish(message)
            self.log.write(json.dumps(report)+'\n'); self.log.flush()
            path = ROOT/'.runtime/avoidance_latest.json'
            temporary = path.with_suffix('.tmp'); temporary.write_text(message.data); temporary.replace(path)


def main():
    # Keep ROS alive while the Ctrl+C path requests a measured, normal stop.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); node = Avoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.enabled = False
        node.begin_stop('IDLE', 'Ctrl+C: stop motion before closing node')
        end = time.monotonic()+40
        while time.monotonic() < end and node.state not in ('IDLE', 'HALTED'):
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.publish(); node.log.close(); node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()

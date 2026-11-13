#!/usr/bin/env python3
"""Simulation-only slow crawl with feedback gates and latched fault handling."""
import math
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from controller_manager_msgs.srv import ListControllers
from control_msgs.msg import JointTrajectoryControllerState

from lite3_gait_core import (PhaseWalk as Crawl, MeasuredSwing, JointTargetRamp, TrackingGuard,
                             clock_step_kind, JOINTS, LEGS, STAND, HIP, SIDE,
                             L1, L2, LIMITS, clamp, smooth, measured_feet,
                             yaw_from_quaternion)


def validate_model():
    share = Path(get_package_share_directory('simulationmode_grazebo'))
    root = ET.fromstring((share/'6-2_robot_modeling/urdf/Lite3.urdf').read_text())
    joints = {j.attrib['name']: j for j in root.findall('joint')}
    for i, leg in enumerate(('FL', 'FR', 'HL', 'HR')):
        expected = [(f'{leg}_HipX_joint', (*HIP[i], 0.0), (-1.0, 0.0, 0.0)),
                    (f'{leg}_HipY_joint', (0.0, SIDE if i in (0, 2) else -SIDE, 0.0), (0.0, -1.0, 0.0)),
                    (f'{leg}_Knee_joint', (0.0, 0.0, -L1), (0.0, -1.0, 0.0)),
                    (f'{leg}_Ankle', (0.0, 0.0, -L2), None)]
        for k, (name, xyz, axis) in enumerate(expected):
            j = joints[name]
            origin = j.find('origin')
            actual = [float(x) for x in origin.attrib.get('xyz', '0 0 0').split()]
            rpy = [float(x) for x in origin.attrib.get('rpy', '0 0 0').split()]
            if len(actual) != 3 or len(rpy) != 3 or any(abs(a-b)>1e-6 for a,b in zip(actual,xyz)) or any(abs(x)>1e-6 for x in rpy):
                raise ValueError('URDF geometry differs from tested Lite3 model: '+name)
            if axis is not None:
                values = [float(x) for x in j.find('axis').attrib['xyz'].split()]
                limit = j.find('limit')
                lower, upper = LIMITS[k]
                if j.attrib['type'] != 'revolute' or len(values) != 3 or any(abs(a-b)>1e-6 for a,b in zip(values,axis)):
                    raise ValueError('Unexpected joint axis/type: '+name)
                if abs(float(limit.attrib['lower'])-lower)>1e-6 or abs(float(limit.attrib['upper'])-upper)>1e-6:
                    raise ValueError('Unexpected joint limits: '+name)
            elif j.attrib['type'] != 'fixed':
                raise ValueError('Expected fixed ankle: '+name)
        foot = root.find(f"link[@name='{leg}_FOOT']")
        collision = foot.find('collision') if foot is not None else None
        sphere = collision.find('geometry/sphere') if collision is not None else None
        if sphere is None or abs(float(sphere.attrib['radius'])-0.022)>1e-6:
            raise ValueError('Foot collision sphere is not the measured 22 mm radius: '+leg)
        origin = collision.find('origin')
        xyz = [float(value) for value in origin.attrib.get('xyz','0 0 0').split()] if origin is not None else [0.0]*3
        if len(xyz)!=3 or any(abs(value)>1e-6 for value in xyz):
            raise ValueError('Foot collision center differs from the ankle frame: '+leg)


class Gait(Node):
    def __init__(self):
        super().__init__('lite3_controller', namespace='lite3_sim',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('Lite3 feedback requires use_sim_time=true; check launch parameters and /clock')
        validate_model()
        def parameter(name, default):
            return self.declare_parameter(name, default).value
        self.joint_timeout = parameter('joint_timeout', 1.0)
        self.imu_timeout = parameter('imu_timeout', 1.0)
        self.pose_timeout = parameter('pose_timeout', 1.0)
        self.cmd_timeout = parameter('cmd_timeout', 0.5)
        self.max_speed = parameter('max_linear_speed', 0.003)
        self.max_turn = parameter('max_angular_speed', 0.01)
        self.tilt_limit = parameter('tilt_limit', 0.30)
        self.balance_gain = parameter('balance_gain', 0.35)
        self.joint_rate = parameter('joint_rate_limit', 0.8)
        self.min_lift = parameter('min_measured_lift', 0.012)
        numeric = [self.joint_timeout, self.imu_timeout, self.cmd_timeout, self.max_speed,
                   self.max_turn, self.tilt_limit, self.balance_gain, self.joint_rate,
                   self.pose_timeout, self.min_lift]
        if not all(math.isfinite(x) for x in numeric) or not (
                0.3<=self.joint_timeout<=3 and 0.3<=self.imu_timeout<=3 and
                0.3<=self.pose_timeout<=3 and 0.005<=self.min_lift<=0.025 and
                0.1<=self.cmd_timeout<=1 and 0<self.max_speed<=0.004 and
                0<self.max_turn<=0.012 and 0.1<=self.tilt_limit<=0.4 and
                0<=self.balance_gain<=0.8 and 0.2<=self.joint_rate<=1.0):
            raise ValueError('Invalid conservative gait parameters')
        self.settings = dict(cycle_period=parameter('cycle_period', 12.0),
                             duty_factor=parameter('duty_factor', 0.82),
                             settle_time=parameter('settle_time', 0.15),
                             step_height=parameter('step_height', 0.025),
                             max_stride=parameter('max_stride', 0.04),
                             support_margin=parameter('support_margin', 0.018))
        self.planner = Crawl(**self.settings)
        self.state, self.reason = 'DISARMED', 'Call /lite3_sim/enable_gait after feedback is ready'
        self.q = self.last_target = None
        self.start_q = None
        self.command_ramp = None
        self.ramp_holds = 0
        self.body_pose = self.measured = None
        self.pose_stamp = None
        self.pose_wall = 0.0
        self.measured_stamp = None
        self.initial_heading = 0.0
        self.swing = None
        self.pending_touchdown = None
        self.touchdown_samples = 0
        self.touchdown_bottom = None
        self.touchdown_stamp = None
        self.touchdown_start_stamp = None
        self.saw_swing = False
        self.swing_peak = float('-inf')
        self.swing_floor_peak = float('-inf')
        self.swing_samples = 0
        self.last_lift_stamp = None
        self.lift_passed = False
        self.require_zero_command = False
        self.tested_legs = []
        self.test_peaks = {}
        self.test_aborted = False
        self.test_finish_start = None
        self.last_actual_lift = 0.0
        self.post_swing_offset = [0.0]*12
        self.post_swing_time = 0.0
        self.roll = self.pitch = 0.0
        self.joint_wall = self.imu_wall = self.cmd_wall = self.controller_wall = 0.0
        self.joint_stamp = self.imu_stamp = None
        self.controllers_ok = False
        self.tracking = TrackingGuard()
        self.tracking_wall = 0.0
        self.vx = self.wz = self.filtered_vx = self.filtered_wz = 0.0
        self.last_sim = None
        self.clock_wall = time.monotonic()
        self.settle_elapsed = 0.0
        self.clock_gaps = 0
        self.last_clock_delta = 0.0
        self.pub = self.create_publisher(JointTrajectory, '/lite3_sim/lite3_leg_controller/joint_trajectory', 1)
        self.command_pub = self.create_publisher(JointState, '/lite3_sim/joint_commands', 1)
        self.status_pub = self.create_publisher(String, '/lite3_sim/gait_status', 1)
        self.create_subscription(JointState, '/lite3_sim/joint_state_broadcaster/joint_states', self.joints, qos_profile_sensor_data)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/lite3_sim/lite3_leg_controller/controller_state',
                                 self.controller_state, qos_profile_sensor_data)
        self.create_subscription(Imu, '/lite3_sim/imu', self.imu, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/lite3_sim/model_pose', self.pose, qos_profile_sensor_data)
        self.create_subscription(Twist, '/lite3_sim/cmd_vel', self.command, 1)
        self.create_subscription(Twist, '/lite3_sim/gait_cmd', self.command, 1)
        self.create_service(Trigger, '/lite3_sim/enable_gait', self.enable)
        self.create_service(Trigger, '/lite3_sim/test_foot_lift', self.test_foot_lift)
        self.create_service(Trigger, '/lite3_sim/stop_gait', self.stop)
        self.create_service(Trigger, '/lite3_sim/emergency_stop_gait', self.emergency)
        self.client = self.create_client(ListControllers, '/lite3_sim/controller_manager/list_controllers')
        self.pending = None
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self.tick, clock=self.steady)
        self.create_timer(1.0, self.report, clock=self.steady)
        self.get_logger().info('measured-swing-v4.1: duty=%.3f period=%.3f. DISARMED; continuous handoff, foot-lift test required.' % (self.planner.duty_factor,self.planner.cycle_time))

    def joints(self, message):
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            return
        data = dict(zip(message.name, message.position))
        if not all(j in data and math.isfinite(data[j]) for j in JOINTS):
            return
        self.q = [data[j] for j in JOINTS]
        self.joint_wall = time.monotonic()
        self.joint_stamp = message.header.stamp.sec + message.header.stamp.nanosec/1e9
        self.measure_feet()

    def pose(self, message):
        for transform in message.transforms:
            if transform.child_frame_id != 'lite3_control':
                continue
            when = transform.header.stamp.sec + transform.header.stamp.nanosec/1e9
            if transform.header.frame_id != 'lite3_world' or when <= 0:
                continue
            t, q = transform.transform.translation, transform.transform.rotation
            values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(v) for v in values):
                continue
            try:
                yaw_from_quaternion(values[3:])
            except ValueError:
                continue
            if self.pose_stamp is not None and when <= self.pose_stamp:
                continue
            self.body_pose = (values[:3], values[3:])
            self.pose_stamp = when
            self.pose_wall = time.monotonic()
            self.measure_feet()

    def measure_feet(self):
        if self.q is None or self.body_pose is None or self.joint_stamp is None:
            return
        if abs(self.pose_stamp-self.joint_stamp) > 0.05:
            return
        self.measured = measured_feet(*self.body_pose, self.q)
        self.measured_stamp = max(self.pose_stamp, self.joint_stamp)
        if (self.state in ('LIFT_TEST', 'WALKING') and self.planner.state == 'swing'
                and self.swing is not None and
                0.3 <= self.planner.swing_phase <= 0.7 and
                (self.last_lift_stamp is None or self.measured_stamp>self.last_lift_stamp)):
            leg = self.planner.leg
            height = self.measured[leg][2]-self.swing.actual_toe[2]
            self.swing_peak = max(self.swing_peak, height)
            self.swing_floor_peak = max(self.swing_floor_peak, self.measured[leg][2]-0.022)
            self.swing_samples += 1
            self.last_lift_stamp = self.measured_stamp
            self.last_actual_lift = self.measured[leg][2]-0.022
        if (self.state in ('LIFT_TEST', 'WALKING') and self.planner.state == 'settle'
                and self.pending_touchdown is not None
                and self.planner.elapsed >= self.planner.settle_time*0.5
                and self.measured_stamp > self.touchdown_start_stamp
                and (self.touchdown_stamp is None or self.measured_stamp > self.touchdown_stamp)):
            leg, _ = self.pending_touchdown
            self.touchdown_samples += 1
            self.touchdown_bottom = self.measured[leg][2]-0.022
            self.touchdown_stamp = self.measured_stamp

    def imu(self, message):
        q = message.orientation
        values = [q.x, q.y, q.z, q.w]
        norm = math.sqrt(sum(x*x for x in values))
        if message.orientation_covariance[0] < 0 or not all(math.isfinite(x) for x in values) or norm < 0.5:
            return
        x,y,z,w = [v/norm for v in values]
        self.roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        self.pitch = math.asin(clamp(2*(w*y-z*x), -1.0, 1.0))
        self.imu_wall = time.monotonic()
        self.imu_stamp = message.header.stamp.sec + message.header.stamp.nanosec/1e9

    def controller_state(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec/1e9
        sim = self.get_clock().now().nanoseconds/1e9
        # Reject queued old samples and samples from an unrelated/reset clock.
        if stamp < sim-0.5 or stamp > sim+0.1:
            return
        if self.tracking.update(message.joint_names, message.error.positions, stamp):
            self.tracking_wall = time.monotonic()
            if self.state not in ('DISARMED', 'FAULT') and self.tracking.failure:
                self.fault(self.tracking.failure)

    def command(self, message):
        if not math.isfinite(message.linear.x) or not math.isfinite(message.angular.z):
            self.fault('Non-finite velocity request')
            return
        moving = abs(message.linear.x)>1e-9 or abs(message.angular.z)>1e-9
        if self.state == 'LIFT_TEST':
            self.require_zero_command |= moving
            return
        if self.require_zero_command:
            if moving:
                return
            self.require_zero_command = False
        self.vx = clamp(message.linear.x, -self.max_speed, self.max_speed)
        self.wz = clamp(message.angular.z, -self.max_turn, self.max_turn)
        self.cmd_wall = time.monotonic()

    def readiness(self):
        now = time.monotonic()
        if not self.get_parameter('use_sim_time').value:
            return 'use_sim_time must be true'
        if self.last_sim is None or self.last_sim <= 0 or now-self.clock_wall>0.8:
            return 'Simulation clock missing or paused'
        if self.q is None or now-self.joint_wall>self.joint_timeout:
            return 'Joint feedback missing/stale'
        if now-self.imu_wall>self.imu_timeout:
            return 'IMU feedback missing/stale'
        if self.body_pose is None or now-self.pose_wall>self.pose_timeout:
            return 'Gazebo model pose missing/stale; check PosePublisher and bridge in C'
        if (self.measured is None or self.measured_stamp is None or
                self.pose_stamp is None or self.joint_stamp is None or
                abs(self.pose_stamp-self.joint_stamp)>0.05 or
                not -0.1 <= self.last_sim-self.measured_stamp <= 0.2):
            return 'Measured pose and joint states are not time-aligned'
        if not self.controllers_ok or now-self.controller_wall>3.0:
            return 'Both controllers must be active'
        if self.tracking.stamp is None or now-self.tracking_wall>self.joint_timeout:
            return 'Controller state missing/stale'
        if self.last_sim-self.tracking.stamp>0.5:
            return 'Controller state simulation timestamp is stale'
        if max(abs(self.roll), abs(self.pitch))>self.tilt_limit:
            return 'Body tilt exceeds limit; reset the model before enabling'
        return ''

    def enable(self, request, response):
        if self.state not in ('DISARMED', 'FAULT'):
            response.success, response.message = False, 'Already enabled'
            return response
        reason = self.readiness()
        if not reason and self.tracking.maximum>0.35:
            reason = f'Controller must settle before enable: {self.tracking.joint}={self.tracking.maximum:.3f} rad'
        if not reason and max(abs(a-b) for a,b in zip(self.q,STAND))>0.25:
            reason = 'Start near the verified standing pose; do not enable on a fallen model'
        if not reason and max(abs(foot[2]-0.022) for foot in self.measured)>0.03:
            reason = 'Measured foot centers are too far from z=0.022 m; inspect model geometry and ground'
        if reason:
            response.success, response.message = False, reason
            return response
        self.planner = Crawl(**self.settings)
        self.initial_heading = yaw_from_quaternion(self.body_pose[1])
        self.swing = None
        self.pending_touchdown = None
        self.touchdown_samples = 0
        self.touchdown_bottom = None
        self.touchdown_stamp = None
        self.touchdown_start_stamp = None
        self.lift_passed = False
        self.require_zero_command = False
        self.tested_legs = []
        self.test_peaks = {}
        self.test_aborted = False
        self.test_finish_start = None
        self.swing_peak = float('-inf')
        self.swing_floor_peak = float('-inf')
        self.swing_samples = 0
        self.last_lift_stamp = None
        self.post_swing_offset = [0.0]*12
        self.start_q = list(self.q)
        self.last_target = list(self.q)
        self.command_ramp = JointTargetRamp(self.last_target, rate=self.joint_rate)
        self.ramp_holds = 0
        self.settle_elapsed = 0.0
        self.filtered_vx = self.filtered_wz = self.vx = self.wz = 0.0
        self.cmd_wall = 0.0
        self.tracking.since = None
        self.tracking.failure = ''
        self.state, self.reason = 'SETTLING', 'Transitioning to standing pose for 3 simulation seconds'
        response.success, response.message = True, self.reason
        return response

    def test_foot_lift(self, request, response):
        if self.state != 'READY' or self.planner.state != 'idle':
            response.success, response.message = False, 'Wait for READY and all four feet on ground'
            return response
        if self.command_ramp is None or self.command_ramp.pending:
            response.success, response.message = False, 'Wait for the standing joint endpoint to complete'
            return response
        if max(abs(actual-target) for actual,target in zip(self.q,self.last_target)) > 0.03:
            response.success, response.message = False, 'Wait for actual joints to reach the standing endpoint'
            return response
        if self.lift_passed:
            response.success, response.message = False, 'Four legs already passed; restart gait node to retest'
            return response
        reason = self.readiness()
        if reason:
            response.success, response.message = False, reason
            return response
        if max(abs(foot[2]-0.022) for foot in self.measured) > 0.008:
            response.success, response.message = False, 'Place all four feet near z=0 ground before testing'
            return response
        self.planner = Crawl(**self.settings)
        self.initial_heading = yaw_from_quaternion(self.body_pose[1])
        self.swing = None
        self.pending_touchdown = None
        self.touchdown_samples = 0
        self.touchdown_bottom = None
        self.touchdown_stamp = None
        self.touchdown_start_stamp = None
        self.tested_legs = []
        self.test_peaks = {}
        self.test_aborted = False
        self.test_finish_start = None
        self.swing_peak = float('-inf')
        self.swing_floor_peak = float('-inf')
        self.swing_samples = 0
        self.last_lift_stamp = None
        self.lift_passed = False
        self.require_zero_command = False
        self.vx = self.wz = self.filtered_vx = self.filtered_wz = 0.0
        self.cmd_wall = 0.0
        self.state, self.reason = 'LIFT_TEST', 'Measuring HR, FR, HL, FL; no keyboard command needed'
        response.success, response.message = True, self.reason
        return response

    def stop(self, request, response):
        self.vx = self.wz = self.filtered_vx = self.filtered_wz = 0.0
        self.cmd_wall = 0.0
        self.planner.stop_requested = True
        if self.state == 'LIFT_TEST':
            self.test_aborted = True
        response.success, response.message = True, 'Finish current touchdown and recenter; ongoing keyboard input must also stop'
        return response

    def emergency(self, request, response):
        self.fault('Emergency stop requested')
        response.success, response.message = True, 'FAULT latched; joint target held; this does not recover balance'
        return response

    def fault(self, reason):
        if self.state == 'FAULT':
            return
        self.state, self.reason = 'FAULT', reason
        self.lift_passed = False
        self.vx = self.wz = self.filtered_vx = self.filtered_wz = 0.0
        self.get_logger().error(reason+'; gait stopped, explicit re-enable required')
        if self.last_target is not None:
            self.publish(self.last_target)

    def publish(self, positions):
        message = JointTrajectory()
        message.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        # Short horizon bounds commands queued on interruption. Position-only
        # interpolation is intentional; rate-limited input, no unvalidated derivatives.
        point.time_from_start.nanosec = 40_000_000
        message.points = [point]
        self.pub.publish(message)
        monitor = JointState()
        monitor.header.stamp = self.get_clock().now().to_msg()
        monitor.name = list(JOINTS)
        monitor.position = list(positions)
        self.command_pub.publish(monitor)

    def finish_swing(self, nominal):
        """Record mid-swing clearance; touchdown is checked after settling."""
        if self.swing is None:
            raise ValueError('Swing ended without measured start pose')
        if self.pending_touchdown is not None:
            raise ValueError('Previous leg touchdown has not been checked')
        leg = self.swing.leg
        corrected = self.swing.joint_target(self.planner.feet[leg], *self.body_pose)
        self.post_swing_offset = [0.0]*12
        for j in range(3):
            index = 3*leg+j
            self.post_swing_offset[index] = corrected[j]-nominal[index]
        self.swing = None
        if self.swing_samples < 3:
            raise ValueError(f'{LEGS[leg]}: no synchronized mid-swing foot-height samples')
        if self.swing_peak < self.min_lift:
            raise ValueError(f'{LEGS[leg]} actual lift {self.swing_peak*1000:.1f} mm '
                             f'< {self.min_lift*1000:.1f} mm; inspect joint execution/contact')
        if self.swing_floor_peak < 0.008:
            raise ValueError(f'{LEGS[leg]} remains near ground despite joint motion: '
                             f'maximum bottom height {self.swing_floor_peak*1000:.1f} mm')
        self.pending_touchdown = (leg, self.swing_peak)
        self.touchdown_samples = 0
        self.touchdown_bottom = None
        self.touchdown_stamp = None
        self.touchdown_start_stamp = self.measured_stamp
        if self.state == 'LIFT_TEST' and len(self.tested_legs) == 3 and leg == 0:
            self.planner.stop_requested = True
        self.reason = f'{LEGS[leg]} lifted {self.swing_peak*1000:.1f} mm; checking touchdown'

    def finish_touchdown(self):
        """Require a fresh synchronized foot near the flat ground after settle."""
        if self.pending_touchdown is None:
            raise ValueError('Settle ended without pending foot touchdown')
        leg, peak = self.pending_touchdown
        if (self.touchdown_samples < 2 or self.touchdown_stamp is None or
                not -0.10 <= self.last_sim-self.touchdown_stamp <= 0.10):
            raise ValueError(f'{LEGS[leg]}: no fresh synchronized touchdown samples')
        if abs(self.touchdown_bottom) > 0.008:
            raise ValueError(f'{LEGS[leg]} foot bottom {self.touchdown_bottom*1000:+.1f} mm '
                             'after settle; touchdown not verified')
        self.pending_touchdown = None
        self.touchdown_start_stamp = None
        if self.state == 'LIFT_TEST':
            self.tested_legs.append(leg)
            self.test_peaks[LEGS[leg]] = round(peak*1000, 1)
            if len(set(self.tested_legs)) != len(self.tested_legs):
                raise ValueError('Foot-lift test repeated a leg')
            self.reason = f'{LEGS[leg]} lift and touchdown measured; {len(self.tested_legs)}/4 legs passed'
        else:
            self.reason = f'{LEGS[leg]} lift and touchdown measured'

    def corrected_target(self, nominal, prior_phase):
        """Use measured Gazebo body pose for the moving foot only."""
        target = list(nominal)
        if prior_phase == 'swing' and self.planner.state != 'swing':
            self.finish_swing(target)
        if prior_phase == 'settle' and self.planner.state != 'settle':
            self.finish_touchdown()
        if self.planner.state == 'swing':
            leg = self.planner.leg
            if self.swing is None:
                if (self.measured is None or self.measured_stamp is None or
                        self.last_sim-self.measured_stamp>0.05):
                    raise ValueError('No recent measured toe pose at swing start')
                self.swing = MeasuredSwing(leg, self.measured[leg],
                                           self.planner.start_foot, self.initial_heading,
                                           measured_feet(*self.body_pose, self.last_target)[leg])
                self.swing_peak = float('-inf')
                self.swing_floor_peak = float('-inf')
                self.swing_samples = 0
                self.last_lift_stamp = None
            target[3*leg:3*leg+3] = self.swing.joint_target(
                self.planner.feet[leg], *self.body_pose)
        elif self.planner.state == 'settle':
            factor = 1.0-smooth(self.planner.elapsed/self.planner.settle_time)
            target = [value+offset*factor for value,offset in
                      zip(target,self.post_swing_offset)]
        else:
            self.post_swing_offset = [0.0]*12
        return target

    def verify_test_finish(self):
        """Do not declare READY while the final endpoint is still unexecuted."""
        if self.test_finish_start is None:
            self.test_finish_start = self.last_sim
        floor_error = max(abs(foot[2]-0.022) for foot in self.measured)
        endpoint_error = max(abs(actual-target) for actual,target in zip(self.q,self.last_target))
        after_endpoint = min(self.pose_stamp,self.joint_stamp) >= self.test_finish_start+0.04
        if after_endpoint and floor_error <= 0.008 and endpoint_error <= 0.03:
            self.cmd_wall = 0.0
            if self.test_aborted:
                self.state, self.reason = 'READY', 'Foot-lift test stopped; run it again before WASD'
            else:
                self.lift_passed = True
                self.state, self.reason = 'READY', 'Four measured foot lifts and touchdowns passed; send fresh velocity commands'
            return
        self.reason = (f'Waiting for final touchdown: floor_error={floor_error*1000:.1f} mm, '
                       f'endpoint_error={endpoint_error:.4f} rad, post_endpoint_feedback={after_endpoint}')
        if self.last_sim-self.test_finish_start > 0.5:
            raise ValueError(self.reason+'; final standing feedback did not settle')

    def tick(self):
        sim = self.get_clock().now().nanoseconds/1e9
        previous = self.last_sim
        if previous is None or sim != previous:
            self.clock_wall = time.monotonic()
            self.last_sim = sim
        if self.state in ('DISARMED', 'FAULT'):
            return
        reason = self.readiness()
        if reason:
            self.fault(reason)
            return
        if previous is None or sim == previous:
            return
        dt = sim-previous
        self.last_clock_delta = dt
        kind = clock_step_kind(dt)
        if kind == 'fault':
            self.fault(f'Simulation time jumped: dt={dt:+.6f} s, previous={previous:.6f}, current={sim:.6f}; check /clock publishers/reset')
            return
        if kind == 'hold':
            # A delayed timer/clock update must not fast-forward the gait.
            # Accept only if all actual simulation feedback agrees with the new
            # time. Otherwise this may be a second clock source or bad reset.
            stamps = (self.joint_stamp, self.imu_stamp, self.tracking.stamp)
            if any(stamp is None or not -0.1 <= sim-stamp <= 0.2 for stamp in stamps):
                self.fault(f'Clock gap without time-aligned feedback: dt={dt:.6f} s')
                return
            self.clock_gaps += 1
            self.publish(self.last_target)
            self.get_logger().warning(f'Forward clock sample gap dt={dt:.6f} s; holding target, planner not advanced (count={self.clock_gaps})')
            return
        if self.tracking.failure:
            self.fault(self.tracking.failure)
            return
        try:
            if self.command_ramp is None:
                raise ValueError('Joint target ramp not initialized; re-enable gait')
            if self.command_ramp.pending:
                # Do not advance swing/settle or accept another endpoint while
                # a bounded correction from the previous step is still pending.
                target = self.command_ramp.advance(dt)
                self.ramp_holds += 1
                self.last_target = target
                self.publish(target)
                return
            if self.state == 'SETTLING' and self.settle_elapsed >= 3.0:
                self.state, self.reason = 'READY', 'Standing; run /lite3_sim/test_foot_lift before WASD'
                self.publish(self.last_target)
                return
            if (self.state == 'LIFT_TEST' and self.planner.state == 'idle'
                    and (self.tested_legs == [3,1,2,0] or self.test_aborted)):
                self.verify_test_finish()
                self.publish(self.last_target)
                return
            target = None
            # Substep the planner when the remote GUI or Python scheduling stalls.
            count = max(1, math.ceil(dt/0.02))
            for _ in range(count):
                step = dt/count
                if self.state == 'SETTLING':
                    self.settle_elapsed += step
                    u = smooth(self.settle_elapsed/3.0)
                    target = [a+(b-a)*u for a,b in zip(self.start_q,STAND)]
                else:
                    prior_phase = self.planner.state
                    fresh = time.monotonic()-self.cmd_wall < self.cmd_timeout
                    if self.state == 'LIFT_TEST':
                        vx, wz = (0.0006, 0.0) if not self.test_aborted else (0.0, 0.0)
                    elif self.lift_passed and fresh:
                        vx, wz = self.vx, self.wz
                    else:
                        vx, wz = 0.0, 0.0
                    self.filtered_vx += clamp(vx-self.filtered_vx,-0.015*step,0.015*step)
                    self.filtered_wz += clamp(wz-self.filtered_wz,-0.05*step,0.05*step)
                    nominal = self.planner.update(step,self.filtered_vx,self.filtered_wz,
                        clamp(-self.balance_gain*self.roll,-0.05,0.05),
                        clamp(-self.balance_gain*self.pitch,-0.05,0.05))
                    target = self.corrected_target(nominal, prior_phase)
                    if (self.state == 'LIFT_TEST' and self.planner.state == 'idle'
                            and (self.tested_legs == [3,1,2,0] or self.test_aborted)):
                        break  # Publish recenter endpoint before final feedback verification.
                    if self.state != 'LIFT_TEST':
                        self.state = 'READY' if self.planner.state == 'idle' else 'WALKING'
                        if self.state == 'READY' and not self.lift_passed:
                            self.reason = 'Standing; run /lite3_sim/test_foot_lift before WASD'
                        elif self.state == 'WALKING' and self.planner.state == 'swing':
                            self.reason = f'{LEGS[self.planner.leg]} measured swing in progress'
            if target is not None:
                # Enforce the same rate on the published endpoint; small sensor
                # sample steps extend the phase instead of snapping the command.
                self.last_target = self.command_ramp.push(target,dt)
                self.publish(self.last_target)
        except (ValueError, ArithmeticError) as error:
            pose_age = float('nan') if self.pose_stamp is None else sim-self.pose_stamp
            joint_age = float('nan') if self.joint_stamp is None else sim-self.joint_stamp
            self.fault(f'{error}; phase={self.planner.state}; dt={dt:.6f} s; '
                       f'pose_age={pose_age:+.4f} s; joint_age={joint_age:+.4f} s')

    def report(self):
        if self.pending is not None and self.pending.done():
            try:
                controllers = {c.name:c.state for c in self.pending.result().controller}
                self.controllers_ok = all(controllers.get(n)=='active' for n in ('joint_state_broadcaster','lite3_leg_controller'))
                self.controller_wall = time.monotonic()
            except Exception:
                self.controllers_ok = False
            self.pending = None
        if self.pending is None and self.client.service_is_ready():
            self.pending = self.client.call_async(ListControllers.Request())
        leg_index = getattr(self.planner, 'leg', None)
        active_leg = ('FL','FR','HL','HR')[leg_index] if leg_index is not None else '-'
        planned_lift = max(foot[2] for foot in self.planner.feet)*1000.0
        actual_lift = (self.measured[leg_index][2]-0.022)*1000.0 if (
            leg_index is not None and self.measured is not None and
            self.measured_stamp is not None and self.last_sim is not None and
            abs(self.last_sim-self.measured_stamp)<0.1) else float('nan')
        message = String()
        lift_results = ','.join(f'{leg}:{self.test_peaks[leg]:.1f}'
                                for leg in LEGS if leg in self.test_peaks)
        message.data = (f'{self.state}; phase={self.planner.state}; leg={active_leg}; '
                        f'swing_u={self.planner.swing_phase:.3f}; contact_plan={"".join("1" if x else "0" for x in self.planner.contact_plan)}; '
                        f'planned_lift_mm={planned_lift:.2f}; actual_lift_mm={actual_lift:.2f}; '
                        f'lift_test={len(self.tested_legs)}/4; lift_results_mm={lift_results}; '
                        f'lift_passed={self.lift_passed}; roll={self.roll:.3f}; '
                        f'pitch={self.pitch:.3f}; tracking={self.tracking.maximum:.4f} rad; '
                        f'worst_joint={self.tracking.joint}; clock_dt={self.last_clock_delta:+.4f}; '
                        f'clock_gaps={self.clock_gaps}; '
                        f'ramp_pending={self.command_ramp.pending if self.command_ramp is not None else False}; '
                        f'ramp_lag={self.command_ramp.lag if self.command_ramp is not None else 0.0:.5f} rad; '
                        f'ramp_holds={self.ramp_holds}; reason={self.reason}')
        self.status_pub.publish(message)


def main():
    rclpy.init()
    node = None
    try:
        node = Gait()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.fault('Gait process exiting')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Lite3 V5.2: measured support transfer, vertical calibration, then crawl.

Position-controller experiments in Fortress, not NMPC/WBC or torque balance.
The pinned calibration and the free-base lift test have separate pass flags.
"""
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from controller_manager_msgs.srv import ListControllers
from control_msgs.msg import JointTrajectoryControllerState

from lite3_feedback_plumbing import Gait as FeedbackGait, validate_model
from lite3_gait_core import (JOINTS, LEGS, STAND, TrackingGuard, JointTargetRamp,
                             measured_feet, yaw_from_quaternion, clock_step_kind,
                             clamp, smooth, inverse, body_from_world, rotate)
from lite3_measured_core import MeasuredSupportStep, URDFCenterOfMass, support_metrics
from lite3_feedback_sync import FeedbackSynchronizer

VERSION = 'lite3-only-v5.2'
ORDER = (2, 0, 3, 1)  # HL, FL, HR, FR: four-beat walk, one swing leg.


class Gait(FeedbackGait):
    """Reuse validated ROS feedback plumbing; replace V4's virtual planner."""
    def __init__(self):
        Node.__init__(self, 'lite3_controller', namespace='lite3_sim',
                      parameter_overrides=[Parameter('use_sim_time', value=True)])
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('Lite3 controller requires use_sim_time=true; check launch parameters and /clock')
        validate_model()
        share = Path(get_package_share_directory('simulationmode_grazebo'))
        self.mass_model = URDFCenterOfMass(share/'6-2_robot_modeling/urdf/Lite3.urdf')
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.pin_body = param('pin_body', False)
        self.model_pinned = None
        self.model_mode_error = ''
        self.joint_timeout = param('joint_timeout', 1.0)
        self.imu_timeout = param('imu_timeout', 1.0)
        self.pose_timeout = param('pose_timeout', 1.0)
        self.cmd_timeout = param('cmd_timeout', 0.5)
        self.max_speed = param('max_linear_speed', 0.003)
        self.max_turn = param('max_angular_speed', 0.01)
        self.tilt_limit = param('tilt_limit', 0.18)
        self.joint_rate = param('joint_rate_limit', 0.6)
        self.height = param('step_height', 0.030)
        self.min_lift = param('min_measured_lift', 0.020)
        self.margin = param('support_margin', 0.018)
        self.shift_time = param('shift_time', 2.5)
        self.lift_time = param('lift_time', 1.5)
        self.hold_time = param('hold_time', 1.0)
        self.transfer_time = param('transfer_time', 1.0)
        self.lower_time = param('lower_time', 1.5)
        self.max_stride = param('max_stride', 0.015)
        numeric = (self.joint_timeout, self.imu_timeout, self.pose_timeout,
                   self.cmd_timeout, self.max_speed, self.max_turn, self.tilt_limit,
                   self.joint_rate, self.height, self.min_lift, self.margin,
                   self.shift_time, self.lift_time, self.hold_time,
                   self.transfer_time, self.lower_time, self.max_stride)
        if not all(math.isfinite(v) and v > 0 for v in numeric) or not (
                isinstance(self.pin_body, bool) and self.max_speed <= .004 and
                self.max_turn <= .012 and .020 <= self.height <= .035 and
                .015 <= self.min_lift <= self.height and .2 <= self.joint_rate <= .8 and
                self.shift_time >= 1.5 and self.lift_time >= 1.0 and
                self.lower_time >= 1.0 and self.hold_time >= .5 and
                .010 <= self.margin <= .025 and self.max_stride <= .020):
            raise ValueError('Invalid V5 calibration/crawl settings')
        self.state, self.stage, self.reason = 'DISARMED', 'idle', 'Call enable_gait after feedback is ready'
        self.q = self.last_target = self.body_pose = self.measured = None
        self.start_q = None
        self.feedback_sync = FeedbackSynchronizer(tolerance=.025, max_samples=200)
        self.raw_pose_stamp = self.raw_joint_stamp = None
        self.control_sample_stamp = None
        self.sync_holds = 0
        self.controller_rejected_old = self.controller_rejected_future = 0
        self.pose_stamp = self.measured_stamp = self.joint_stamp = self.imu_stamp = None
        self.pose_wall = self.joint_wall = self.imu_wall = self.cmd_wall = self.controller_wall = 0.0
        self.tracking_wall = 0.0
        self.tracking = TrackingGuard()
        self.controllers_ok = False
        self.pending = None
        self.roll = self.pitch = 0.0
        self.vx = self.wz = self.filtered_vx = self.filtered_wz = 0.0
        self.last_sim = None
        self.clock_wall = time.monotonic()
        self.clock_gaps = 0
        self.last_clock_delta = 0.0
        self.command_ramp = None
        self.ramp_holds = 0
        self.require_zero_command = False
        self.lift_passed = False
        self.calibration_passed = False
        self.tested_legs, self.test_peaks = [], {}
        self.step = None
        self.elapsed = 0.0
        self.stable_since = None
        self.sample_stamp = None
        self.hold_samples = 0
        self.hold_peak = 0.0
        self.hold_ok_since = None
        self.stopping = False
        self.index = 0
        self.metrics = {}
        self.pub = self.create_publisher(JointTrajectory, '/lite3_sim/lite3_leg_controller/joint_trajectory', 1)
        self.command_pub = self.create_publisher(JointState, '/lite3_sim/joint_commands', 1)
        self.status_pub = self.create_publisher(String, '/lite3_sim/gait_status', 1)
        self.create_subscription(JointState, '/lite3_sim/joint_state_broadcaster/joint_states', self.joints, qos_profile_sensor_data)
        self.create_subscription(JointTrajectoryControllerState, '/lite3_sim/lite3_leg_controller/controller_state', self.controller_state, qos_profile_sensor_data)
        self.create_subscription(Imu, '/lite3_sim/imu', self.imu, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/lite3_sim/model_pose', self.pose, qos_profile_sensor_data)
        self.create_subscription(String, '/lite3_sim/robot_description', self.model_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Twist, '/lite3_sim/cmd_vel', self.command, 1)
        self.create_subscription(Twist, '/lite3_sim/gait_cmd', self.command, 1)
        self.create_service(Trigger, '/lite3_sim/enable_gait', self.enable)
        self.create_service(Trigger, '/lite3_sim/test_foot_lift', self.test_foot_lift)
        self.create_service(Trigger, '/lite3_sim/stop_gait', self.stop)
        self.create_service(Trigger, '/lite3_sim/emergency_stop_gait', self.emergency)
        self.client = self.create_client(ListControllers, '/lite3_sim/controller_manager/list_controllers')
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02, self.tick, clock=self.steady)
        self.create_timer(.5, self.report, clock=self.steady)
        self.get_logger().info(f'{VERSION}: pin_body={self.pin_body}; order=HL,FL,HR,FR; height={self.height*1000:.0f} mm. DISARMED')

    def joints(self, message):
        if len(message.name)!=len(message.position) or len(set(message.name))!=len(message.name):
            return
        data = dict(zip(message.name,message.position))
        if not all(j in data and math.isfinite(data[j]) for j in JOINTS):
            return
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        try:
            pair = self.feedback_sync.add_joints(stamp,[data[j] for j in JOINTS])
        except ValueError:
            return
        if self.raw_joint_stamp is None or stamp > self.raw_joint_stamp:
            self.raw_joint_stamp,self.joint_wall = stamp,time.monotonic()
        self.accept_feedback(pair)

    def pose(self, message):
        for transform in message.transforms:
            if transform.child_frame_id!='lite3_control' or transform.header.frame_id!='lite3_world':
                continue
            stamp = transform.header.stamp.sec+transform.header.stamp.nanosec/1e9
            position,rotation = transform.transform.translation,transform.transform.rotation
            try:
                pair = self.feedback_sync.add_pose(stamp,(position.x,position.y,position.z),
                                                   (rotation.x,rotation.y,rotation.z,rotation.w))
            except ValueError:
                continue
            if self.raw_pose_stamp is None or stamp > self.raw_pose_stamp:
                self.raw_pose_stamp,self.pose_wall = stamp,time.monotonic()
            self.accept_feedback(pair)

    def accept_feedback(self, pair):
        if pair is None:
            return  # Raw asynchronous arrivals never replace a coherent snapshot.
        self.q = list(pair.q)
        self.body_pose = (pair.position,pair.quaternion)
        self.pose_stamp,self.joint_stamp = pair.pose_stamp,pair.joint_stamp
        self.measure_feet()

    def measure_feet(self):
        if (self.q is None or self.body_pose is None or self.joint_stamp is None or
                self.pose_stamp is None or abs(self.pose_stamp-self.joint_stamp) > .025):
            return
        self.measured = measured_feet(*self.body_pose, self.q)
        self.measured_stamp = min(self.pose_stamp, self.joint_stamp)

    def controller_state(self, message):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        sim = self.get_clock().now().nanoseconds/1e9
        if not math.isfinite(stamp) or stamp<=0:
            return
        if stamp < sim-.5:
            self.controller_rejected_old += 1
            return
        if stamp > sim+.1:
            self.controller_rejected_future += 1
            return
        if self.tracking.update(message.joint_names,message.error.positions,stamp):
            self.tracking_wall = time.monotonic()
            if self.state not in ('DISARMED','FAULT') and self.tracking.failure:
                self.fault(self.tracking.failure)

    def model_description(self, message):
        try:
            root = ET.fromstring(message.data)
            joint = root.find("gazebo/joint[@name='lite3_calibration_world_fixed']")
            self.model_pinned = joint is not None
            if joint is not None and (joint.get('type')!='fixed' or
                    joint.findtext('parent')!='world' or joint.findtext('child')!='TORSO'):
                self.model_mode_error = 'Unexpected calibration world constraint'
        except ET.ParseError:
            self.model_mode_error = 'Invalid robot_description XML'

    def readiness(self):
        reason = FeedbackGait.readiness(self)
        if reason:
            if reason=='Measured pose and joint states are not time-aligned':
                age = float('nan') if self.measured_stamp is None else self.last_sim-self.measured_stamp
                return f'Synchronized feedback missing/stale: pair_age={age*1000:.1f} ms; raw_pose={self.raw_pose_stamp}; raw_joint={self.raw_joint_stamp}'
            if reason.startswith('Controller state'):
                return (reason+f'; rejected_old={self.controller_rejected_old}, rejected_future={self.controller_rejected_future}; '
                        'check use_sim_time and the single /clock publisher')
            return reason
        if self.model_mode_error:
            return self.model_mode_error
        if self.model_pinned is None:
            return 'Waiting for /lite3_sim/robot_description to verify calibration mode'
        if self.model_pinned != self.pin_body:
            return 'C/E pin_body mismatch; recreate model and restart gait with matching true/false'
        if abs(self.pose_stamp-self.joint_stamp) > .025:
            return 'Invalid coherent feedback pair: skew exceeds 25 ms'
        return ''

    def command(self, message):
        if not math.isfinite(message.linear.x) or not math.isfinite(message.angular.z):
            self.fault('Non-finite velocity request')
            return
        moving = abs(message.linear.x) > 1e-9 or abs(message.angular.z) > 1e-9
        if self.pin_body or self.state == 'LIFT_TEST':
            self.require_zero_command |= moving
            return
        if self.require_zero_command:
            if moving:
                return
            self.require_zero_command = False
        self.vx = clamp(message.linear.x, -self.max_speed, self.max_speed)
        self.wz = clamp(message.angular.z, -self.max_turn, self.max_turn)
        self.cmd_wall = time.monotonic()
        if not moving:
            self.stopping = True

    def enable(self, request, response):
        if self.state not in ('DISARMED', 'FAULT'):
            response.success, response.message = False, 'Already enabled'
            return response
        reason = self.readiness()
        if not reason and self.tracking.maximum > .35:
            reason = 'Controller must settle below 0.35 rad tracking error before enable'
        if not reason and max(abs(a-b) for a,b in zip(self.q, STAND)) > .25:
            reason = 'Reset the model to its initial standing pose before enable'
        if not reason and max(abs(f[2]-.022) for f in self.measured) > .010:
            reason = 'Four foot bottoms must be within 10 mm of ground; reset model after a failed test'
        if reason:
            response.success, response.message = False, reason
            return response
        self.start_q = list(self.q)
        self.last_target = list(self.q)
        self.command_ramp = JointTargetRamp(self.q, self.joint_rate, max_correction=.10, timeout=.6)
        self.control_sample_stamp = self.measured_stamp
        self.elapsed = 0.0
        self.state, self.stage = 'SETTLING', 'stand'
        self.step = None
        self.vx = self.wz = 0.0
        self.cmd_wall = 0.0
        self.require_zero_command = False
        self.stopping = False
        self.tracking.since = None
        self.tracking.failure = ''
        self.lift_passed = self.calibration_passed = False
        self.tested_legs, self.test_peaks = [], {}
        self.reason = 'Transitioning to standing pose for 3 simulation seconds'
        response.success, response.message = True, self.reason
        return response

    def test_foot_lift(self, request, response):
        reason = self.readiness()
        if not reason and (self.state != 'READY' or self.stage != 'idle'):
            reason = 'Wait for READY/idle'
        if not reason and (self.command_ramp is None or self.command_ramp.pending or
                           max(abs(a-b) for a,b in zip(self.q,self.last_target)) > .03):
            reason = 'Wait for the standing joint endpoint to be reached'
        if not reason and max(abs(f[2]-.022) for f in self.measured) > .006:
            reason = 'All four measured foot bottoms must be within 6 mm of ground'
        if reason:
            response.success, response.message = False, reason
            return response
        self.state = 'LIFT_TEST'
        self.tested_legs, self.test_peaks = [], {}
        self.lift_passed = self.calibration_passed = False
        self.vx = self.wz = 0.0
        self.cmd_wall = 0.0
        self.stopping = False
        self.index = 0
        try:
            self.begin_step()
        except (ValueError,ArithmeticError) as error:
            self.fault('Cannot initialize foot test: '+str(error))
            response.success, response.message = False, self.reason
            return response
        response.success, response.message = True, 'Zero-stride test: HL, FL, HR, FR; lift 30 mm, hold, lower; no W needed'
        return response

    def stop(self, request, response):
        self.vx = self.wz = 0.0
        self.cmd_wall = 0.0
        self.stopping = True
        self.require_zero_command = True
        response.success, response.message = True, 'Stop requested; finish current touchdown; stop the keyboard/publisher too'
        return response

    def fault(self, reason):
        self.calibration_passed = False
        FeedbackGait.fault(self, reason)

    def phase(self, stage):
        self.stage, self.elapsed = stage, 0.0
        self.stable_since = None
        self.sample_stamp = self.measured_stamp
        self.reason = f'{LEGS[self.step.leg]} {stage}' if self.step else stage

    def begin_step(self):
        if max(abs(f[2]-.022) for f in self.measured) > .006:
            raise ValueError('Cannot start new leg until all four measured foot bottoms are within 6 mm')
        leg = ORDER[self.index]
        stride = [0.0, 0.0]
        if self.state == 'WALKING':
            yaw = yaw_from_quaternion(self.body_pose[1])
            # Feedforward placement only. Actual speed is not commanded by teleporting.
            if abs(self.wz) > .002:
                toe = body_from_world(self.measured[leg], *self.body_pose)
                rotated = rotate(toe[0], toe[1], self.wz*8.0)
                stride = list(rotate(rotated[0]-toe[0], rotated[1]-toe[1], yaw))
            else:
                stride = list(rotate(self.vx*8.0, 0.0, yaw))
            length = math.hypot(*stride)
            scale = min(1.0, self.max_stride/max(length,1e-12))
            stride = [v*scale for v in stride]
        com_body = self.mass_model.body_com(self.q)
        self.step = MeasuredSupportStep(*self.body_pose, self.q, leg,
                    stride=tuple(stride), height=self.height, com_body=com_body, margin=self.margin)
        if self.pin_body:
            self.step.target_body = tuple(self.body_pose[0])
        self.hold_samples, self.hold_peak, self.hold_ok_since = 0, 0.0, None
        self.phase('shift')

    def new_measurement(self):
        if self.measured_stamp is None or (self.sample_stamp is not None and
                                           self.measured_stamp <= self.sample_stamp):
            return False
        if self.sample_stamp is not None and self.measured_stamp-self.sample_stamp > .060:
            self.stable_since = None
            self.hold_ok_since = None
            self.hold_samples = 0
        self.sample_stamp = self.measured_stamp
        return True

    def stable(self, condition, duration=.25):
        if not self.new_measurement():
            return False
        if not condition:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = self.measured_stamp
        return self.measured_stamp-self.stable_since >= duration

    def support_ok(self):
        self.metrics = self.step.diagnostics(self.body_pose, self.q)
        self.metrics['support_margin'] = support_metrics(
            self.body_pose, self.measured, self.mass_model.body_com(self.q), self.step.leg)['support_margin']
        support = self.metrics['support_bottoms']
        a, b = self.body_pose[1], self.step.quaternion
        dot = abs(sum(x*y for x,y in zip(a,b)))/math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
        orientation_change = 2*math.acos(clamp(dot,0.0,1.0))
        if self.pin_body and (math.dist(self.body_pose[0],self.step.position) > .003 or
                             orientation_change > .015):
            raise ValueError('Pinned calibration body moved; world constraint was not effective')
        if max(support.values()) > .008:
            wrong = max(support, key=support.get)
            raise ValueError(f'Wrong support foot airborne: planned={LEGS[self.step.leg]}, '
                             f'actual={wrong}, bottom={support[wrong]*1000:.1f} mm')
        if min(support.values()) < -.008:
            raise ValueError('Measured support foot penetrates floor more than 8 mm; inspect pose/geometry')
        if not self.pin_body and self.metrics['support_margin'] < -.005:
            raise ValueError(f'Actual whole-body COM left the three-foot support triangle: margin={self.metrics["support_margin"]*1000:.1f} mm')
        return self.pin_body or self.metrics['support_margin'] >= .006

    def run_step(self, dt):
        self.elapsed += dt
        leg = self.step.leg
        pose = self.body_pose
        if self.stage == 'shift':
            target = self.step.targets('shift', min(1.0,self.elapsed/self.shift_time), pose)
            if self.stopping:
                self.phase('stop_wait')
                return self.last_target
            if self.elapsed >= self.shift_time:
                self.phase('shift_wait')
            return target
        if self.stage == 'shift_wait':
            target = self.step.targets('shift', 1.0, pose)
            if self.stopping:
                self.phase('stop_wait')
                return self.last_target
            # A virtual shift alone never proves that the swing foot was unloaded.
            self.metrics = support_metrics(pose, self.measured,
                                self.mass_model.body_com(self.q), leg)
            landed = max(abs(f[2]-.022) for f in self.measured) <= .006
            positioned = self.pin_body or (
                self.metrics['support_margin'] >= .008 and
                math.hypot(pose[0][0]-self.step.target_body[0],
                           pose[0][1]-self.step.target_body[1]) < .015 and
                max(abs(self.roll),abs(self.pitch)) < .10)
            tracked = max(abs(a-b) for a,b in zip(self.q,self.last_target)) < .03
            if self.stable(landed and positioned and tracked):
                self.step.begin_lift(pose,self.q,self.last_target)
                self.phase('lift')
            elif self.elapsed > 3.0:
                raise ValueError(f'{LEGS[leg]} support transfer not verified: '
                                 f'COM_margin={self.metrics["support_margin"]*1000:.1f} mm; '
                                 f'foot_bottoms_mm={[round((f[2]-.022)*1000,1) for f in self.measured]}')
            return target
        if self.stage in ('lift','hold','transfer','lower'):
            self.support_ok()
            duration = {'lift':self.lift_time,'hold':self.hold_time,
                        'transfer':self.transfer_time,'lower':self.lower_time}[self.stage]
            target = self.step.targets(self.stage, min(1.0,self.elapsed/duration), pose)
            if self.stage == 'lift' and self.elapsed >= duration:
                self.phase('hold')
            elif self.stage == 'hold':
                if self.new_measurement():
                    lift = self.measured[leg][2]-self.step.lift_start[2]
                    bottom = self.measured[leg][2]-.022
                    self.hold_peak = max(self.hold_peak, lift)
                    ok = lift >= self.min_lift and bottom >= self.min_lift and self.support_ok()
                    if ok:
                        self.hold_samples += 1
                        if self.hold_ok_since is None:
                            self.hold_ok_since = self.measured_stamp
                    else:
                        self.hold_ok_since = None
                if self.elapsed >= duration:
                    passed = (self.hold_samples >= 3 and self.hold_ok_since is not None and
                              self.measured_stamp-self.hold_ok_since >= .20)
                    if not passed:
                        raise ValueError(f'{LEGS[leg]} hold failed: actual_peak={self.hold_peak*1000:.1f} mm '
                                         f'< required {self.min_lift*1000:.1f} mm sustained .20 s, or support invalid')
                    if self.stopping:
                        self.step.stride = (0.0,0.0)
                    self.phase('transfer' if self.state=='WALKING' and not self.stopping else 'lower')
            elif self.stage == 'transfer' and self.elapsed >= duration:
                self.phase('lower')
            elif self.stage == 'lower' and self.elapsed >= duration:
                self.phase('land_wait')
            return target
        if self.stage == 'land_wait':
            target = self.step.targets('lower', 1.0, pose)
            landed = max(abs(f[2]-.022) for f in self.measured) <= .006
            tracked = max(abs(a-b) for a,b in zip(self.q,self.last_target)) < .03
            if self.stable(landed and tracked):
                self.test_peaks[LEGS[leg]] = self.hold_peak*1000
                if self.state=='LIFT_TEST' and not self.stopping:
                    self.tested_legs.append(leg)
                if self.stopping or (self.state=='LIFT_TEST' and len(self.tested_legs)==4):
                    self.finish_cycle()
                else:
                    self.index = (self.index+1)%4
                    self.begin_step()
                    return self.step.targets('shift', 0.0, self.body_pose)
            elif self.elapsed > 2.0:
                raise ValueError(f'{LEGS[leg]} touchdown not verified; all feet must return within 6 mm')
            return target
        if self.stage == 'stop_wait':
            landed = max(abs(f[2]-.022) for f in self.measured) <= .006
            tracked = max(abs(a-b) for a,b in zip(self.q,self.last_target)) < .03
            if self.stable(landed and tracked):
                self.finish_cycle()
            elif self.elapsed > 2.0:
                raise ValueError('Stop endpoint did not settle with all four feet on ground')
            return self.last_target
        raise ValueError('Unexpected measured gait stage: '+self.stage)

    def finish_cycle(self):
        was_test = self.state == 'LIFT_TEST' and not self.stopping and self.tested_legs == list(ORDER)
        self.lift_passed = was_test and not self.pin_body or self.lift_passed
        self.calibration_passed = was_test and self.pin_body or self.calibration_passed
        self.state, self.stage = 'READY', 'idle'
        self.step = None
        self.stopping = False
        self.cmd_wall = 0.0
        self.require_zero_command = True
        self.reason = ('Pinned calibration passed; recreate free model before WASD' if self.calibration_passed else
                       'Four free-base lifts passed; send fresh WASD commands' if self.lift_passed else
                       'Stopped after touchdown; run test_foot_lift before WASD')

    def tick(self):
        sim = self.get_clock().now().nanoseconds/1e9
        previous = self.last_sim
        if previous is None or sim != previous:
            self.clock_wall, self.last_sim = time.monotonic(), sim
        if self.state in ('DISARMED','FAULT'):
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
            self.fault(f'Simulation time jumped: dt={dt:+.6f}; check clock publishers/reset')
            return
        if kind == 'hold':
            stamps = (self.joint_stamp,self.imu_stamp,self.pose_stamp,self.tracking.stamp)
            if any(v is None or not -.1 <= sim-v <= .2 for v in stamps):
                self.fault('Clock gap without aligned feedback')
                return
            self.clock_gaps += 1
            self.publish(self.last_target)
            return
        if self.tracking.failure:
            self.fault(self.tracking.failure)
            return
        if self.control_sample_stamp is not None and self.measured_stamp <= self.control_sample_stamp:
            self.sync_holds += 1
            self.publish(self.last_target)
            return
        # Phase/ramp progress follows new coherent measurements. A gap never
        # produces a large catch-up movement when feedback finally resumes.
        progress_dt = .02 if self.control_sample_stamp is None else min(.02, self.measured_stamp-self.control_sample_stamp)
        self.control_sample_stamp = self.measured_stamp
        dt = progress_dt
        try:
            if self.command_ramp.pending:
                self.last_target = self.command_ramp.advance(dt)
                self.ramp_holds += 1
                self.publish(self.last_target)
                return
            if self.state == 'SETTLING':
                if self.start_q is None:
                    raise ValueError('Standing transition was not initialized; call enable_gait')
                self.elapsed += dt
                target = [a+(b-a)*smooth(self.elapsed/3) for a,b in zip(self.start_q,STAND)]
                if self.elapsed >= 3:
                    if max(abs(a-b) for a,b in zip(self.q,STAND)) < .03 and max(abs(f[2]-.022) for f in self.measured)<.006:
                        self.state, self.stage = 'READY','idle'
                        self.reason = 'Standing; run test_foot_lift; zero stride first'
                    elif self.elapsed > 5:
                        raise ValueError('Initial standing endpoint/ground feedback did not settle')
            elif self.state == 'READY':
                target = self.last_target
                fresh = time.monotonic()-self.cmd_wall < self.cmd_timeout
                moving = abs(self.vx)>.0005 or abs(self.wz)>.002
                if not self.pin_body and self.lift_passed and fresh and moving and not self.require_zero_command:
                    self.state, self.stopping = 'WALKING',False
                    self.begin_step()
                    target = self.step.targets('shift',0.0,self.body_pose)
            else:
                if self.state=='WALKING' and time.monotonic()-self.cmd_wall >= self.cmd_timeout:
                    self.stopping = True
                target = self.run_step(dt)
            self.last_target = self.command_ramp.push(target,dt)
            self.publish(self.last_target)
        except (ValueError,ArithmeticError) as error:
            self.fault(f'{error}; stage={self.stage}; planned_leg={LEGS[self.step.leg] if self.step else "-"}')

    def report(self):
        if self.pending is not None and self.pending.done():
            try:
                states = {c.name:c.state for c in self.pending.result().controller}
                self.controllers_ok = all(states.get(n)=='active' for n in ('joint_state_broadcaster','lite3_leg_controller'))
                self.controller_wall = time.monotonic()
            except Exception:
                self.controllers_ok = False
            self.pending = None
        if self.pending is None and self.client.service_is_ready():
            self.pending = self.client.call_async(ListControllers.Request())
        leg = LEGS[self.step.leg] if self.step else '-'
        bottoms = [round((f[2]-.022)*1000,1) for f in self.measured] if self.measured else []
        support_margin = self.metrics.get('support_margin',float('nan'))*1000
        results = ','.join(f'{name}:{self.test_peaks[name]:.1f}' for name in LEGS if name in self.test_peaks)
        message = String()
        skew = float('nan') if self.pose_stamp is None or self.joint_stamp is None else abs(self.pose_stamp-self.joint_stamp)*1000
        age = float('nan') if self.last_sim is None or self.measured_stamp is None else (self.last_sim-self.measured_stamp)*1000
        message.data = (f'{self.state}; version={VERSION}; pin_body={self.pin_body}; phase={self.stage}; leg={leg}; '
                        f'planned_lift_mm={self.height*1000 if self.stage in ("hold","transfer") else 0:.1f}; '
                        f'actual_bottoms_mm_FL_FR_HL_HR={bottoms}; support_margin_mm={support_margin:.1f}; '
                        f'lift_test={len(self.tested_legs)}/4; lift_results_mm={results}; '
                        f'lift_passed={self.lift_passed}; calibration_passed={self.calibration_passed}; '
                        f'roll={self.roll:.3f}; pitch={self.pitch:.3f}; tracking={self.tracking.maximum:.4f} rad; '
                        f'clock_dt={self.last_clock_delta:+.4f}; clock_gaps={self.clock_gaps}; '
                        f'sync_skew_ms={skew:.1f}; sync_age_ms={age:.1f}; sync_holds={self.sync_holds}; '
                        f'controller_rejected_old={self.controller_rejected_old}; '
                        f'controller_rejected_future={self.controller_rejected_future}; reason={self.reason}')
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

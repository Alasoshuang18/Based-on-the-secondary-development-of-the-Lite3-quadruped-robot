"""Lite3 V5.2 phase gates with an ideal measured plant and ROS message stubs.

The ideal plant executes requested joint positions and the commanded support
shift. These tests verify decisions and geometry, not Gazebo contact physics,
controller plugin execution or balance on the Orange Pi.
"""
import importlib.util
import math
from pathlib import Path
import sys
import time
import types
import unittest
from unittest.mock import patch

import test_gait_watchdog as feedback_tests

Twist, stubs = feedback_tests.Twist, feedback_tests.stubs


stubs['rclpy.qos'].QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
stubs['rclpy.qos'].DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL='transient_local')
SOURCE = Path(__file__).resolve().parents[1]/'src'/'lite3_controller.py'
spec = importlib.util.spec_from_file_location('measured_node_under_test', SOURCE)
implementation = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, stubs):
    spec.loader.exec_module(implementation)


class MeasuredNodeTests(unittest.TestCase):
    def node(self, pinned=False):
        old = feedback_tests.GuardTests().node()
        n = implementation.Gait.__new__(implementation.Gait)
        n.__dict__.update(old.__dict__)
        n.pin_body = n.model_pinned = pinned
        n.model_mode_error = ''
        n.height, n.min_lift, n.margin = .030, .020, .018
        n.shift_time, n.lift_time, n.hold_time = 2.5, 1.5, 1.0
        n.transfer_time, n.lower_time = 1.0, 1.5
        n.max_stride, n.joint_rate = .015, .6
        n.command_ramp = implementation.JointTargetRamp(n.q, n.joint_rate, timeout=.6)
        n.body_pose = ((0.0, 0.0, (.20+.21012)*math.cos(.65)+.022), (0.0, 0.0, 0.0, 1.0))
        n.measured = implementation.measured_feet(*n.body_pose, n.q)
        n.mass_model = types.SimpleNamespace(body_com=lambda _: (0.0, 0.0, 0.0))
        n.stage, n.elapsed = 'idle', 0.0
        n.step = None
        n.stable_since = n.sample_stamp = n.hold_ok_since = None
        n.hold_samples, n.hold_peak = 0, 0.0
        n.stopping = n.calibration_passed = False
        n.metrics = {}
        n.index = 0
        n.start_q = None
        n.feedback_sync = implementation.FeedbackSynchronizer()
        n.raw_pose_stamp = n.raw_joint_stamp = None
        n.control_sample_stamp = None
        n.sync_holds = 0
        n.controller_rejected_old = n.controller_rejected_future = 0
        return n

    def sample(self, n, increment=.05):
        stamp = (n.measured_stamp or 10.0)+increment
        n.last_sim = n.pose_stamp = n.joint_stamp = n.measured_stamp = n.imu_stamp = stamp
        now = time.monotonic()
        n.clock_wall = n.pose_wall = n.joint_wall = n.imu_wall = n.controller_wall = n.tracking_wall = now
        n.tracking.update(implementation.JOINTS, [0.0]*12, stamp)

    def execute(self, n, target):
        n.q = n.last_target = list(target)
        n.measured = implementation.measured_feet(*n.body_pose, n.q)
        self.sample(n)

    def prepare_lift(self, n, leg=None):
        n.state = 'LIFT_TEST'
        if leg is not None:
            n.index = implementation.ORDER.index(leg)
        n.begin_step()
        # The ideal plant physically completes the body shift before lift.
        if not n.pin_body:
            n.body_pose = (n.step.target_body, n.body_pose[1])
        self.execute(n, n.step.targets('shift', 1.0, n.body_pose))
        n.step.begin_lift(n.body_pose, n.q, n.last_target)
        n.phase('lift')

    def test_wrong_support_foot_does_not_certify_requested_leg(self):
        n = self.node(pinned=True)
        self.prepare_lift(n, leg=3)
        n.stage = 'hold'
        # User regression: planned HR lifts 4.4 mm, unrelated FL lifts 26.4 mm.
        for leg, height in ((0, .0264), (3, .0044)):
            foot = list(n.measured[leg]); foot[2] += height
            n.q[3*leg:3*leg+3] = implementation.inverse(
                leg, implementation.body_from_world(foot, *n.body_pose))
        n.measured = implementation.measured_feet(*n.body_pose, n.q)
        n.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=10_010_000_000))
        n.last_sim = 10.0
        n.tick()
        self.assertEqual(n.state, 'FAULT')
        self.assertIn('planned=HR, actual=FL, bottom=26.4 mm', n.reason)
        self.assertFalse(n.lift_passed)
        self.assertFalse(n.calibration_passed)

    def test_virtual_shift_without_actual_com_transfer_cannot_lift(self):
        n = self.node()
        n.state = 'LIFT_TEST'; n.begin_step(); n.phase('shift_wait')
        original_pose = n.body_pose
        for _ in range(8):
            self.sample(n)
            n.run_step(.05)
        self.assertEqual(n.stage, 'shift_wait')
        self.assertIsNone(n.step.lift_start)
        self.assertEqual(n.body_pose, original_pose)
        self.assertLess(n.metrics['support_margin'], .008)
        n.elapsed = 3.01
        with self.assertRaisesRegex(ValueError, 'support transfer not verified'):
            n.run_step(.01)

    def test_shift_requires_fresh_stable_support_samples(self):
        n = self.node()
        n.state = 'LIFT_TEST'; n.begin_step(); n.phase('shift_wait')
        n.body_pose = (n.step.target_body, n.body_pose[1])
        self.execute(n, n.step.targets('shift', 1.0, n.body_pose))
        n.run_step(.05)
        for _ in range(20):
            n.run_step(.05)  # No new measurement stamp.
        self.assertEqual(n.stage, 'shift_wait')
        for _ in range(7):
            self.sample(n)
            n.run_step(.05)
        self.assertEqual(n.stage, 'lift')
        self.assertIsNotNone(n.step.lift_start)

    def test_low_requested_leg_cannot_pass_sustained_hold(self):
        n = self.node(pinned=True); self.prepare_lift(n)
        leg = n.step.leg
        foot = list(n.step.lift_start); foot[2] += .0044
        n.q[3*leg:3*leg+3] = implementation.inverse(
            leg, implementation.body_from_world(foot, *n.body_pose))
        n.measured = implementation.measured_feet(*n.body_pose, n.q)
        n.phase('hold')
        with self.assertRaisesRegex(ValueError, 'actual_peak=4.4 mm'):
            for _ in range(24):
                self.sample(n)
                n.run_step(.05)
        self.assertEqual(n.hold_samples, 0)
        self.assertFalse(n.lift_passed)

    def run_ideal_test(self, pinned):
        n = self.node(pinned)
        response = n.test_foot_lift(None, types.SimpleNamespace())
        self.assertTrue(response.success)
        lifted = []
        for _ in range(1800):
            if n.state == 'READY':
                break
            if n.stage in ('shift_wait','lift','hold','transfer','lower','land_wait') and not pinned:
                n.body_pose = (n.step.target_body, n.body_pose[1])
            target = n.run_step(.025)
            self.execute(n, target)
            if n.stage == 'hold' and n.step.leg not in lifted:
                lifted.append(n.step.leg)
        self.assertEqual(n.state, 'READY')
        self.assertEqual(lifted, list(implementation.ORDER))
        self.assertEqual(n.tested_legs, list(implementation.ORDER))
        self.assertEqual(set(n.test_peaks), set(implementation.LEGS))
        for height in n.test_peaks.values():
            self.assertGreaterEqual(height, 20.0)
        self.assertLess(max(abs(f[2]-.022) for f in n.measured), .006)
        return n

    def test_four_measured_free_lifts_and_touchdowns_allow_walk(self):
        n = self.run_ideal_test(pinned=False)
        self.assertTrue(n.lift_passed)
        self.assertFalse(n.calibration_passed)

    def test_four_pinned_lifts_only_certify_calibration(self):
        n = self.run_ideal_test(pinned=True)
        self.assertTrue(n.calibration_passed)
        self.assertFalse(n.lift_passed)
        request = Twist(); request.linear.x = .003
        n.command(request)
        self.assertEqual(n.vx, 0.0)

    def test_touchdown_cannot_pass_with_repeated_or_airborne_sample(self):
        n = self.node(pinned=True); self.prepare_lift(n)
        n.phase('land_wait')
        self.execute(n, n.step.targets('lower', 1.0, n.body_pose))
        n.run_step(.05)
        for _ in range(15):
            n.run_step(.05)
        self.assertEqual(n.tested_legs, [])
        self.assertEqual(n.stage, 'land_wait')
        leg = n.step.leg
        foot = list(n.measured[leg]); foot[2] += .010
        n.q[3*leg:3*leg+3] = implementation.inverse(
            leg, implementation.body_from_world(foot, *n.body_pose))
        n.measured = implementation.measured_feet(*n.body_pose, n.q)
        for _ in range(7):
            self.sample(n); n.run_step(.05)
        self.assertEqual(n.tested_legs, [])

    def test_stop_during_lift_finishes_current_lower_and_starts_no_new_leg(self):
        n = self.node(pinned=True); self.prepare_lift(n)
        leg = n.step.leg
        response = n.stop(None, types.SimpleNamespace())
        self.assertTrue(response.success)
        stages = []
        for _ in range(400):
            if n.state == 'READY':
                break
            stages.append(n.stage)
            self.assertEqual(n.step.leg, leg)
            target = n.run_step(.025)
            self.execute(n, target)
        self.assertIn('lower', stages)
        self.assertIn('land_wait', stages)
        self.assertEqual(n.state, 'READY')
        self.assertEqual(n.tested_legs, [])
        self.assertFalse(n.lift_passed)
        self.assertFalse(n.calibration_passed)

    def test_fault_revokes_both_pass_flags(self):
        n = self.node(); n.lift_passed = n.calibration_passed = True
        n.fault('Feedback lost')
        self.assertFalse(n.lift_passed)
        self.assertFalse(n.calibration_passed)

    def test_model_mode_must_match_constraint_in_robot_description(self):
        n = self.node(pinned=False)
        n.model_description(types.SimpleNamespace(data=(
            '<robot><gazebo><joint name="lite3_calibration_world_fixed" type="fixed">'
            '<parent>world</parent><child>TORSO</child></joint></gazebo></robot>')))
        self.assertIn('pin_body mismatch', n.readiness())
        n.pin_body = True
        self.assertEqual(n.readiness(), '')
        n.model_description(types.SimpleNamespace(data=(
            '<robot><gazebo><joint name="lite3_calibration_world_fixed" type="revolute">'
            '<parent>world</parent><child>TORSO</child></joint></gazebo></robot>')))
        self.assertIn('Unexpected calibration', n.readiness())

    def test_pinned_body_translation_is_rejected(self):
        n = self.node(pinned=True); self.prepare_lift(n)
        position, quaternion = n.body_pose
        n.body_pose = ((position[0]+.004, position[1], position[2]), quaternion)
        n.measured = implementation.measured_feet(*n.body_pose, n.q)
        with self.assertRaisesRegex(ValueError, 'world constraint was not effective'):
            n.support_ok()

    def test_pinned_body_roll_rotation_is_rejected(self):
        n = self.node(pinned=True); self.prepare_lift(n)
        position, _ = n.body_pose
        n.body_pose = (position, (math.sin(.02/2), 0.0, 0.0, math.cos(.02/2)))
        n.measured = implementation.measured_feet(*n.body_pose, n.q)
        with self.assertRaisesRegex(ValueError, 'world constraint was not effective'):
            n.support_ok()

    def test_quaternion_sign_change_does_not_fake_body_rotation(self):
        n = self.node(pinned=True); self.prepare_lift(n)
        position, quaternion = n.body_pose
        n.body_pose = (position, tuple(-v for v in quaternion))
        self.assertTrue(n.support_ok())

    def test_pose_and_joint_must_align_within_25_ms(self):
        n = self.node(); old = n.measured_stamp
        n.pose_stamp += .030
        n.measure_feet()
        self.assertEqual(n.measured_stamp, old)
        self.assertIn('25 ms', n.readiness())

    def test_large_clock_reset_revokes_tests_and_latches_fault(self):
        n = self.node(); n.lift_passed = True
        n.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=12_000_000_000))
        self.sample(n, 2.0)
        n.last_sim = 10.0
        n.tick()
        self.assertEqual(n.state, 'FAULT')
        self.assertIn('Simulation time jumped', n.reason)
        self.assertFalse(n.lift_passed)

    def test_controller_tracking_failure_stops_before_new_leg(self):
        n = self.node(); n.tracking.failure = 'Controller tracking error: HR_Knee_joint=0.900 rad'
        n.tick()
        self.assertEqual(n.state, 'FAULT')
        self.assertIn('HR_Knee_joint', n.reason)

    def send_joints(self, n, stamp, positions):
        sec = int(stamp)
        header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=sec,nanosec=round((stamp-sec)*1e9)))
        n.joints(types.SimpleNamespace(header=header,name=list(implementation.JOINTS),position=list(positions)))

    def send_pose(self, n, stamp, position=None):
        sec = int(stamp)
        position = position or n.body_pose[0]
        transform = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=sec,nanosec=round((stamp-sec)*1e9)),frame_id='lite3_world'),
            child_frame_id='lite3_control',transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=position[0],y=position[1],z=position[2]),
                rotation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
        n.pose(types.SimpleNamespace(transforms=[transform]))

    def test_callback_order_keeps_last_coherent_snapshot_ready(self):
        n = self.node()
        self.send_pose(n,10.0)
        self.send_joints(n,10.0,implementation.STAND)
        original = (list(n.q),n.body_pose,n.measured_stamp)
        self.send_pose(n,10.06,(.001,0.,n.body_pose[0][2]))
        n.last_sim=10.06
        self.assertGreater(n.raw_pose_stamp-n.raw_joint_stamp,.025)
        self.assertEqual((n.q,n.body_pose,n.measured_stamp),original)
        self.assertEqual(n.readiness(),'')
        newer=list(implementation.STAND);newer[1]-=.01
        self.send_joints(n,10.06,newer)
        self.assertEqual(n.q,newer)
        self.assertAlmostEqual(n.measured_stamp,10.06)
        self.assertEqual(n.body_pose[0][0],.001)

    def test_duplicate_feedback_holds_phase_and_pending_ramp(self):
        n = self.node()
        n.state='SETTLING';n.start_q=list(n.q)
        n.control_sample_stamp=n.measured_stamp
        goal=list(n.last_target);goal[1]+=.04
        n.command_ramp.push(goal,.01)
        before=(n.elapsed,list(n.command_ramp.current),n.command_ramp.elapsed)
        n.tick()
        self.assertEqual(n.state,'SETTLING')
        self.assertEqual((n.elapsed,n.command_ramp.current,n.command_ramp.elapsed),before)
        self.assertEqual(n.sync_holds,1)

    def test_a_real_pair_stream_stall_latches_fault(self):
        n = self.node()
        self.send_pose(n,10.)
        self.send_joints(n,10.,implementation.STAND)
        # Pose source keeps arriving, but no joint counterpart newer than 10.
        self.send_pose(n,10.3)
        n.get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=10_300_000_000))
        n.last_sim=10.29
        n.tick()
        self.assertEqual(n.state,'FAULT')
        self.assertIn('Synchronized feedback missing/stale',n.reason)
        self.assertEqual(n.measured_stamp,10.0)

    def test_resumed_pair_does_not_fast_forward_transition(self):
        n = self.node()
        n.state='SETTLING';n.start_q=list(n.q)
        n.control_sample_stamp=10.0
        self.send_pose(n,10.12)
        self.send_joints(n,10.12,implementation.STAND)
        n.last_sim=10.11
        n.get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=10_120_000_000))
        n.tick()
        self.assertAlmostEqual(n.elapsed,.02)
        self.assertEqual(n.state,'SETTLING')

    def test_missing_stand_initialization_has_explicit_fault(self):
        n=self.node();n.state='SETTLING';n.start_q=None
        n.tick()
        self.assertEqual(n.state,'FAULT')
        self.assertIn('transition was not initialized',n.reason)

    def test_controller_wrong_epoch_is_counted_without_refresh(self):
        n=self.node()
        def message(stamp):
            sec=int(stamp)
            return types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=sec,nanosec=round((stamp-sec)*1e9))),
                                         joint_names=list(implementation.JOINTS),error=types.SimpleNamespace(positions=[0.0]*12))
        old_wall=n.tracking_wall
        n.controller_state(message(1791946635.0))
        self.assertEqual(n.controller_rejected_future,1)
        self.assertEqual(n.tracking_wall,old_wall)
        n.controller_state(message(1.0))
        self.assertEqual(n.controller_rejected_old,1)
        self.assertEqual(n.tracking_wall,old_wall)

    def test_use_sim_time_false_cannot_enable(self):
        n=self.node();n.state='DISARMED'
        n.get_parameter=lambda _:types.SimpleNamespace(value=False)
        response=n.enable(None,types.SimpleNamespace())
        self.assertFalse(response.success)
        self.assertIn('use_sim_time must be true',response.message)


if __name__ == '__main__':
    unittest.main()

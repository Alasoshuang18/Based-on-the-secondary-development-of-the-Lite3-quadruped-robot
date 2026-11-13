"""Exercise node safety decisions with message/ROS stubs, not a ROS executor.

Does not test middleware, Gazebo bridges, or plugin execution.
"""
from pathlib import Path
import sys
import time
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0)
        self.angular = types.SimpleNamespace(z=0.0)


stubs = {}
def module(name, **attrs):
    result = types.ModuleType(name)
    result.__dict__.update(attrs)
    stubs[name] = result
    return result


module('rclpy', ok=lambda: True)
module('rclpy.node', Node=object)
module('rclpy.parameter', Parameter=lambda name, value=None: types.SimpleNamespace(name=name,value=value))
module('rclpy.clock', Clock=object, ClockType=object)
module('rclpy.qos', qos_profile_sensor_data=None)
module('ament_index_python')
module('ament_index_python.packages', get_package_share_directory=lambda _: '')
for parent in ('geometry_msgs','sensor_msgs','trajectory_msgs','std_msgs','std_srvs','controller_manager_msgs','control_msgs','tf2_msgs'):
    module(parent)
module('geometry_msgs.msg', Twist=Twist)
module('sensor_msgs.msg', JointState=object, Imu=object)
module('trajectory_msgs.msg', JointTrajectory=object, JointTrajectoryPoint=object)
module('std_msgs.msg', String=object)
module('std_srvs.srv', Trigger=object)
module('controller_manager_msgs.srv', ListControllers=object)
module('control_msgs.msg', JointTrajectoryControllerState=object)
module('tf2_msgs.msg', TFMessage=object)
with patch.dict(sys.modules, stubs):
    import lite3_feedback_plumbing as implementation


class GuardTests(unittest.TestCase):
    def node(self):
        n = implementation.Gait.__new__(implementation.Gait)
        now = time.monotonic()
        n.q = implementation.STAND[:]
        n.last_target = n.q[:]
        n.last_sim = 10.0
        n.clock_wall = n.joint_wall = n.imu_wall = n.controller_wall = now
        n.joint_timeout = n.imu_timeout = 1.0
        n.pose_timeout = 1.0
        n.min_lift = 0.012
        n.cmd_timeout = 0.5
        n.tilt_limit = 0.3
        n.max_speed = 0.003
        n.max_turn = 0.01
        n.balance_gain = 0.35
        n.joint_rate = 0.8
        n.command_ramp = implementation.JointTargetRamp(n.last_target, rate=n.joint_rate)
        n.ramp_holds = 0
        n.controllers_ok = True
        n.tracking = implementation.TrackingGuard()
        n.tracking.update(implementation.JOINTS,[0.0]*12,10.0)
        n.tracking_wall = now
        n.joint_stamp = n.imu_stamp = 10.0
        n.pose_stamp = n.measured_stamp = 10.0
        n.pose_wall = now
        n.body_pose = ((0.0,0.0,0.348), (0.0,0.0,0.0,1.0))
        n.measured = [(0.0,0.0,0.022)]*4
        n.clock_gaps = 0
        n.last_clock_delta = 0.0
        n.roll = n.pitch = 0.0
        n.state, n.reason = 'READY',''
        n.vx = n.wz = n.filtered_vx = n.filtered_wz = 0.0
        n.cmd_wall = now
        n.settings = {}
        n.planner = implementation.Crawl()
        n.lift_passed = False
        n.require_zero_command = False
        n.test_aborted = False
        n.test_finish_start = None
        n.tested_legs = []
        n.test_peaks = {}
        n.swing = None
        n.pending_touchdown = None
        n.touchdown_samples = 0
        n.touchdown_bottom = None
        n.touchdown_stamp = None
        n.touchdown_start_stamp = None
        n.swing_peak = float('-inf')
        n.swing_samples = 0
        n.post_swing_offset = [0.0]*12
        n.initial_heading = 0.0
        n.get_parameter = lambda _: types.SimpleNamespace(value=True)
        n.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=10_010_000_000))
        n.errors, n.sent = [],[]
        n.get_logger = lambda: types.SimpleNamespace(error=n.errors.append,warning=n.errors.append)
        n.publish = lambda q: n.sent.append(list(q))
        return n

    def test_feedback_requirements_and_tilt_gate(self):
        for field, value, message in [
                ('clock_wall',0.0,'clock'), ('joint_wall',0.0,'Joint'),
                ('imu_wall',0.0,'IMU'), ('controllers_ok',False,'controllers'),
                ('roll',0.4,'tilt')]:
            n = self.node()
            setattr(n,field,value)
            self.assertIn(message,n.readiness())

    def test_enable_discards_old_velocity_request(self):
        n = self.node()
        n.state='DISARMED'
        n.vx=0.003
        response=types.SimpleNamespace()
        n.enable(None,response)
        self.assertTrue(response.success)
        self.assertEqual(n.state,'SETTLING')
        self.assertEqual(n.cmd_wall,0.0)
        self.assertEqual(n.vx,0.0)

    def test_refuse_enable_from_nonstanding_joint_pose(self):
        n=self.node();n.state='FAULT';n.q[1]=-1.5
        response=types.SimpleNamespace()
        n.enable(None,response)
        self.assertFalse(response.success)
        self.assertEqual(n.state,'FAULT')

    def test_lost_imu_latches_fault_and_holds(self):
        n=self.node();n.imu_wall=0.0
        n.tick()
        self.assertEqual(n.state,'FAULT')
        self.assertEqual(n.sent,[implementation.STAND])
        n.imu_wall=time.monotonic()
        n.tick()
        self.assertEqual(n.state,'FAULT')

    def test_clock_reset_latches_fault(self):
        n=self.node();n.last_sim=11.0
        n.tick()
        self.assertEqual(n.state,'FAULT')
        self.assertIn('time jumped',n.reason)

    def test_nonfinite_request_faults_and_finite_request_clamps(self):
        n=self.node();command=Twist();command.linear.x=100
        n.command(command)
        self.assertEqual(n.vx,0.003)
        command.angular.z=float('nan')
        n.command(command)
        self.assertEqual(n.state,'FAULT')

    def test_emergency_stop_latched(self):
        n=self.node();response=types.SimpleNamespace()
        n.emergency(None,response)
        self.assertTrue(response.success)
        self.assertEqual(n.state,'FAULT')

    def test_asynchronous_joint_sample_does_not_override_coherent_error(self):
        n=self.node()
        n.q[1] += 0.4  # Old joint sample differs from future sent target.
        n.tick()
        self.assertNotEqual(n.state,'FAULT')

    def test_sustained_controller_error_triggers_fault(self):
        n=self.node()
        for i in range(12):
            n.tracking.update(implementation.JOINTS,[0.4]+[0.0]*11,10.001+i*0.03)
        n.get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=10_341_000_000))
        n.last_sim=10.331
        n.pose_stamp=n.measured_stamp=n.joint_stamp=10.33
        n.tick()
        self.assertEqual(n.state,'FAULT')
        self.assertIn('FL_HipX_joint',n.reason)

    def test_controller_state_loss_faults(self):
        n=self.node();n.tracking_wall=0.0
        n.tick()
        self.assertEqual(n.state,'FAULT')
        self.assertIn('Controller state',n.reason)

    def test_forward_gap_with_aligned_feedback_holds_without_advancing(self):
        n=self.node()
        n.get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=10_350_000_000))
        n.joint_stamp=n.imu_stamp=10.33
        n.pose_stamp=n.measured_stamp=10.33
        n.tracking.update(implementation.JOINTS,[0]*12,10.33)
        elapsed=n.planner.elapsed
        n.tick()
        self.assertEqual(n.state,'READY')
        self.assertEqual(n.planner.elapsed,elapsed)
        self.assertEqual(n.last_sim,10.35)
        self.assertEqual(n.clock_gaps,1)
        self.assertEqual(n.sent,[implementation.STAND])

    def test_forward_gap_with_unaligned_feedback_faults(self):
        n=self.node()
        n.get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=10_350_000_000))
        n.tick()
        self.assertEqual(n.state,'FAULT')
        self.assertIn('time-aligned',n.reason)

    def test_large_clock_jump_keeps_fault_protection(self):
        n=self.node()
        n.get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=12_000_000_000))
        n.tracking.update(implementation.JOINTS,[0]*12,12.0)
        n.pose_stamp=n.measured_stamp=n.joint_stamp=12.0
        n.tick()
        self.assertEqual(n.state,'FAULT')
        self.assertIn('dt=+2.000000',n.reason)

    def test_velocity_cannot_start_walk_before_measured_lift_test(self):
        n=self.node()
        command=Twist();command.linear.x=0.003
        n.command(command)
        n.tick()
        self.assertEqual(n.state,'READY')
        self.assertEqual(n.planner.state,'idle')
        for actual, expected in zip(n.sent[-1],implementation.STAND):
            self.assertAlmostEqual(actual,expected)

    def test_swing_start_preserves_future_endpoint_not_delayed_feedback(self):
        n=self.node()
        while n.planner.state!='swing':
            nominal=n.planner.update(0.004,0.0006,0.0)
        n.last_target=list(nominal)
        n.q=list(nominal)
        n.q[10]-=0.02
        n.body_pose=((n.planner.body[0],n.planner.body[1],n.planner.body[2]+0.022),
                     (0.0,0.0,0.0,1.0))
        n.measured=implementation.measured_feet(*n.body_pose,n.q)
        target=n.corrected_target(nominal,'shift')
        self.assertLess(max(abs(a-b) for a,b in zip(target,n.last_target)),1e-9)
        self.assertEqual(n.swing.actual_toe,tuple(n.measured[3]))
        self.assertGreater(abs(target[10]-n.q[10]),0.019)

    def test_pending_correction_holds_planner_phase_and_rate(self):
        n=self.node()
        goal=list(n.last_target);goal[10]+=0.025
        n.last_target=n.command_ramp.push(goal,0.004)
        prior=list(n.last_target)
        elapsed=n.planner.elapsed
        n.tick()
        self.assertEqual(n.planner.elapsed,elapsed)
        self.assertEqual(n.ramp_holds,1)
        self.assertLessEqual(max(abs(a-b) for a,b in zip(n.sent[-1],prior)),0.008+1e-9)
        self.assertNotEqual(n.state,'FAULT')

    def test_test_service_refuses_pending_standing_endpoint(self):
        n=self.node()
        goal=list(n.last_target);goal[10]+=0.025
        n.last_target=n.command_ramp.push(goal,0.004)
        response=types.SimpleNamespace()
        n.test_foot_lift(None,response)
        self.assertFalse(response.success)
        self.assertEqual(n.state,'READY')

    def test_four_leg_pass_waits_for_actual_endpoint_and_pending_ramp(self):
        n=self.node();n.state='LIFT_TEST';n.tested_legs=[3,1,2,0]
        goal=list(n.last_target);goal[10]+=0.05
        n.last_target=n.command_ramp.push(goal,0.004)
        n.tick()
        self.assertEqual(n.state,'LIFT_TEST')
        self.assertFalse(n.lift_passed)
        while n.command_ramp.pending:
            n.last_target=n.command_ramp.advance(0.004)
        n.verify_test_finish()
        self.assertEqual(n.state,'LIFT_TEST')
        self.assertFalse(n.lift_passed)
        n.q=list(n.last_target)
        n.verify_test_finish()
        self.assertFalse(n.lift_passed)  # Old samples cannot certify final endpoint execution.
        n.last_sim=n.pose_stamp=n.joint_stamp=10.08
        n.verify_test_finish()
        self.assertEqual(n.state,'READY')
        self.assertTrue(n.lift_passed)

    def test_stop_during_pending_correction_keeps_landing_request(self):
        n=self.node();n.state='LIFT_TEST';n.planner.state='swing';n.planner.elapsed=0.4
        goal=list(n.last_target);goal[10]+=0.025
        n.last_target=n.command_ramp.push(goal,0.004)
        response=types.SimpleNamespace()
        n.stop(None,response)
        n.tick()
        self.assertEqual(n.planner.state,'swing')
        self.assertEqual(n.planner.elapsed,0.4)
        self.assertTrue(n.planner.stop_requested)
        self.assertTrue(n.test_aborted)

    def test_large_correction_fault_reports_joint_and_sample_ages(self):
        n=self.node()
        n.last_target[10]-=0.4
        n.command_ramp=implementation.JointTargetRamp(n.last_target,rate=n.joint_rate)
        n.tick()
        self.assertEqual(n.state,'FAULT')
        self.assertIn('HR_HipY_joint',n.reason)
        self.assertIn('pose_age=',n.reason)
        self.assertIn('joint_age=',n.reason)

    def test_pose_and_joint_stamps_must_align(self):
        n=self.node()
        n.pose_stamp=10.3
        self.assertIn('time-aligned',n.readiness())

    def test_bad_real_foot_lift_fails_test(self):
        n=self.node()
        response=types.SimpleNamespace()
        n.test_foot_lift(None,response)
        self.assertTrue(response.success)
        n.planner.leg=3
        n.planner.state='settle'
        n.swing=implementation.MeasuredSwing(3,n.measured[3],
                                             n.planner.feet[3],0.0)
        n.swing_samples=5
        n.swing_peak=0.001
        n.swing_floor_peak=0.001
        with self.assertRaisesRegex(ValueError,'actual lift'):
            n.finish_swing(implementation.STAND[:])

    def test_foot_lift_test_refuses_ungrounded_start(self):
        n=self.node()
        n.measured[0]=(0.0,0.0,0.04)
        response=types.SimpleNamespace()
        n.test_foot_lift(None,response)
        self.assertFalse(response.success)
        self.assertEqual(n.state,'READY')

    def test_touchdown_must_be_fresh_near_ground_before_leg_passes(self):
        n=self.node();n.state='LIFT_TEST'
        n.pending_touchdown=(3,0.018)
        n.touchdown_samples=3
        n.touchdown_stamp=9.98
        n.touchdown_bottom=0.018
        with self.assertRaisesRegex(ValueError,'touchdown not verified'):
            n.finish_touchdown()
        self.assertEqual(n.tested_legs,[])
        n.touchdown_bottom=0.003
        n.touchdown_stamp=9.8
        with self.assertRaisesRegex(ValueError,'no fresh synchronized touchdown'):
            n.finish_touchdown()
        n.touchdown_stamp=9.98
        n.finish_touchdown()
        self.assertEqual(n.tested_legs,[3])
        self.assertEqual(n.test_peaks['HR'],18.0)
        self.assertIsNone(n.pending_touchdown)

    def test_key_held_during_lift_test_requires_release(self):
        n=self.node();n.state='LIFT_TEST'
        held=Twist();held.linear.x=0.003
        n.command(held)
        self.assertTrue(n.require_zero_command)
        n.state='READY';n.lift_passed=True
        n.command(held)
        self.assertEqual(n.vx,0.0)
        n.command(Twist())
        self.assertFalse(n.require_zero_command)
        n.command(held)
        self.assertEqual(n.vx,0.003)

    def test_model_pose_requires_world_frame_and_increasing_sim_stamp(self):
        n=self.node()
        n.body_pose=n.measured=n.pose_stamp=None
        def pose(stamp,frame,x):
            t=types.SimpleNamespace(x=x,y=0.0,z=0.348)
            q=types.SimpleNamespace(x=0.0,y=0.0,z=0.0,w=1.0)
            header=types.SimpleNamespace(frame_id=frame,stamp=types.SimpleNamespace(
                sec=int(stamp),nanosec=int((stamp-int(stamp))*1e9)))
            transform=types.SimpleNamespace(child_frame_id='lite3_control',
                                             header=header,transform=types.SimpleNamespace(
                                                 translation=t,rotation=q))
            return types.SimpleNamespace(transforms=[transform])
        n.pose(pose(10.0,'other_world',1.0))
        self.assertIsNone(n.body_pose)
        n.pose(pose(10.0,'lite3_world',0.2))
        self.assertEqual(n.body_pose[0][0],0.2)
        self.assertIsNotNone(n.measured)
        n.pose(pose(9.0,'lite3_world',9.0))
        self.assertEqual(n.body_pose[0][0],0.2)


if __name__=='__main__':
    unittest.main()

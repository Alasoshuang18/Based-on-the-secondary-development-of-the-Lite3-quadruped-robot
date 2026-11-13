"""Regress command handoff and sampled-pose rate limits without ROS/Gazebo."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from lite3_gait_core import (JOINTS, STAND, JointTargetRamp, MeasuredSwing, smooth,
                             forward, measured_feet, world_from_body)


class TargetRampTests(unittest.TestCase):
    def test_phase_zero_preserves_command_endpoint_not_delayed_feedback(self):
        leg = 3
        actual_q = STAND[:]
        commanded_q = STAND[:]
        commanded_q[3*leg+1] += 0.02
        body = (0.12, -0.04, -forward(leg, actual_q[9:])[2]+0.022)
        orientation = (0.0, 0.0, 0.0, 1.0)
        actual_toe = measured_feet(body, orientation, actual_q)[leg]
        commanded_toe = measured_feet(body, orientation, commanded_q)[leg]
        planned_toe = (*forward(leg, actual_q[9:])[:2], 0.0)
        swing = MeasuredSwing(leg, actual_toe, planned_toe, 0.0,
                              commanded_toe=commanded_toe)
        endpoint = swing.joint_target(planned_toe, body, orientation)
        for actual, wanted in zip(endpoint, commanded_q[9:]):
            self.assertAlmostEqual(actual, wanted, places=10)
        self.assertEqual(swing.actual_toe, tuple(actual_toe))
        self.assertGreater(abs(endpoint[1]-actual_q[10]), 0.019)
        lifted = (*planned_toe[:2], 0.025)
        self.assertAlmostEqual(swing.world_target(lifted)[2]-commanded_toe[2],
                               0.025, places=10)
        # Actual clearance acceptance keeps the measured baseline, even when
        # the command endpoint is a slightly different future configuration.
        self.assertAlmostEqual(swing.actual_toe[2], 0.022, places=10)

    def test_coordinated_increment_retains_joint_direction_and_rate(self):
        start = STAND[:]
        goal = start[:]
        increments = (0.024, -0.012, 0.006)
        for index, delta in enumerate(increments):
            goal[index] += delta
        ramp = JointTargetRamp(start, rate=0.8)
        result = ramp.push(goal, 0.004)
        self.assertTrue(ramp.pending)
        self.assertAlmostEqual(max(abs(a-b) for a,b in zip(result,start)),
                               0.8*0.004, places=10)
        for index, delta in enumerate(increments):
            self.assertAlmostEqual((result[index]-start[index])/delta,
                                   (result[0]-start[0])/increments[0], places=10)
        previous = list(result)
        for _ in range(20):
            result = ramp.advance(0.004)
            self.assertLessEqual(max(abs(a-b) for a,b in zip(result,previous)),
                                 0.8*0.004+1e-12)
            previous = list(result)
            if not ramp.pending:
                break
        self.assertFalse(ramp.pending)
        self.assertAlmostEqual(ramp.lag, 0.0, places=10)
        for actual, wanted in zip(ramp.current,goal):
            self.assertAlmostEqual(actual,wanted,places=10)

    def test_four_ms_commands_bound_twenty_ms_pose_staircases(self):
        leg = 3
        body = (0.0,0.0,-forward(leg,STAND[9:])[2]+0.022)
        orientation = (0.0,0.0,0.0,1.0)
        toe = measured_feet(body,orientation,STAND)[leg]
        planned = (*forward(leg,STAND[9:])[:2],0.0)
        swing = MeasuredSwing(leg,toe,planned,0.0,commanded_toe=toe)
        ramp = JointTargetRamp(STAND,rate=0.8)
        previous = STAND[:]
        pending_seen = False
        for tick in range(40):
            if tick % 5 == 0:
                sampled_body = (body[0]+0.002*(tick//5+1),body[1],body[2])
                goal = STAND[:]
                goal[9:] = swing.joint_target(planned,sampled_body,orientation)
                result = ramp.push(goal,0.004)
            else:
                result = ramp.advance(0.004)
            pending_seen |= ramp.pending
            self.assertLessEqual(max(abs(a-b) for a,b in zip(result,previous)),
                                 0.8*0.004+1e-12)
            previous = list(result)
        self.assertTrue(pending_seen)
        for _ in range(50):
            if not ramp.pending:
                break
            ramp.advance(0.004)
        self.assertFalse(ramp.pending)
        reached = world_from_body(forward(leg,ramp.current[9:]),
                                  sampled_body,orientation)
        for actual,wanted in zip(reached,toe):
            self.assertAlmostEqual(actual,wanted,places=9)

    def test_large_correction_rejects_with_joint_identity(self):
        ramp = JointTargetRamp(STAND,rate=0.8,max_correction=0.10)
        goal = STAND[:]
        goal[9] += 0.11
        with self.assertRaisesRegex(ValueError,JOINTS[9]):
            ramp.push(goal,0.004)
        self.assertEqual(ramp.current,STAND)

    def test_fast_touchdown_offset_extends_settle_without_exceeding_rate(self):
        start=STAND[:];start[11]+=0.10
        ramp=JointTargetRamp(start,rate=0.8)
        phase=elapsed=0.0
        previous=list(start)
        held=False
        for _ in range(200):
            if ramp.pending:
                target=ramp.advance(0.004)
                held=True
            else:
                phase=min(0.15,phase+0.004)
                goal=STAND[:]
                goal[11]+=0.10*(1-smooth(phase/0.15))
                target=ramp.push(goal,0.004)
            elapsed+=0.004
            self.assertLessEqual(max(abs(a-b) for a,b in zip(target,previous)),0.8*0.004+1e-12)
            previous=list(target)
            if phase>=0.15 and not ramp.pending:
                break
        self.assertTrue(held)
        self.assertGreater(elapsed,0.15)
        self.assertEqual(ramp.current,STAND)

    def test_pending_goal_has_bounded_completion_time(self):
        ramp = JointTargetRamp(STAND,rate=0.2,timeout=0.05)
        goal = STAND[:]
        goal[0] += 0.02
        ramp.push(goal,0.004)
        self.assertTrue(ramp.pending)
        with self.assertRaises(ValueError):
            for _ in range(20):
                ramp.advance(0.004)

    def test_pending_endpoint_cannot_be_replaced_to_reset_timeout(self):
        ramp=JointTargetRamp(STAND,rate=0.8)
        first=STAND[:]
        first[0]+=0.04
        ramp.push(first,0.004)
        before=list(ramp.current)
        elapsed=ramp.elapsed
        second=STAND[:]
        second[0]-=0.04
        with self.assertRaisesRegex(ValueError,'pending joint endpoint'):
            ramp.push(second,0.004)
        self.assertEqual(ramp.current,before)
        self.assertEqual(ramp.elapsed,elapsed)
        self.assertEqual(ramp.goal,first)

    def test_invalid_timing_and_nonfinite_targets_rejected(self):
        for dt in (0.0,-0.004,0.201,float('nan'),float('inf')):
            with self.subTest(dt=dt):
                ramp=JointTargetRamp(STAND)
                with self.assertRaises(ValueError):
                    ramp.push(STAND,dt)
        for value in (float('nan'),float('inf'),float('-inf')):
            with self.subTest(value=value):
                goal=STAND[:]
                goal[0]=value
                with self.assertRaises(ValueError):
                    JointTargetRamp(STAND).push(goal,0.004)

    def test_unreachable_measured_ik_is_not_hidden_by_ramp(self):
        leg=3
        body=(0.0,0.0,-forward(leg,STAND[9:])[2]+0.022)
        orientation=(0.0,0.0,0.0,1.0)
        toe=measured_feet(body,orientation,STAND)[leg]
        planned=(*forward(leg,STAND[9:])[:2],0.0)
        swing=MeasuredSwing(leg,toe,planned,0.0,commanded_toe=toe)
        impossible=(planned[0]+2.0,planned[1],0.025)
        with self.assertRaisesRegex(ValueError,'Unreachable foot target'):
            swing.joint_target(impossible,body,orientation)


if __name__=='__main__':
    unittest.main()

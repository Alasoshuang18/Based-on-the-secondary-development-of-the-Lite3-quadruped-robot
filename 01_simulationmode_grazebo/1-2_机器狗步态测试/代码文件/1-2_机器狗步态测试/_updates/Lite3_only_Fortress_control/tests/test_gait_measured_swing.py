"""Kinematics checks for the real-pose swing correction; Gazebo must still be tested."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from lite3_gait_core import (MeasuredSwing, PhaseWalk, STAND, forward,
                             measured_feet, world_from_body, yaw_from_quaternion)


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


class MeasuredSwingTests(unittest.TestCase):
    def test_real_body_height_does_not_push_standing_foot_22_mm_into_floor(self):
        stand_height = -forward(0, STAND[:3])[2]
        body = (0.12, -0.04, stand_height+0.022)
        orientation = quaternion(0, 0, 0.03)
        toe = measured_feet(body, orientation, STAND)[0]
        self.assertAlmostEqual(toe[2]-0.022, 0.0, places=9)
        planned = (*forward(0, STAND[:3])[:2], 0.0)
        swing = MeasuredSwing(0, toe, planned, 0.03)
        command = swing.joint_target(planned, body, orientation)
        for a, b in zip(command, STAND[:3]):
            self.assertAlmostEqual(a, b, places=9)

    def test_measured_body_drift_still_targets_world_clearance(self):
        stand_height = -forward(0, STAND[:3])[2]
        initial_body = (0.12, -0.04, stand_height+0.022)
        initial_q = quaternion(0, 0, 0.03)
        toe_start = measured_feet(initial_body, initial_q, STAND)[3]
        swing = MeasuredSwing(3, toe_start, forward(3, STAND[9:]),
                              yaw_from_quaternion(initial_q))
        planned = list(forward(3, STAND[9:]))
        planned[0] += 0.012
        planned[2] = 0.025
        actual_body = (0.126, -0.044, stand_height+0.019)
        actual_q = quaternion(0.012, -0.01, 0.05)
        target_joints = swing.joint_target(planned, actual_body, actual_q)
        reached = world_from_body(forward(3, target_joints), actual_body, actual_q)
        desired = swing.world_target(planned)
        for a, b in zip(reached, desired):
            self.assertAlmostEqual(a, b, places=8)
        self.assertAlmostEqual(reached[2]-0.022, 0.025, places=8)

    def test_invalid_body_orientation_is_rejected(self):
        toe = forward(0, STAND[:3])
        swing = MeasuredSwing(0, toe, toe, 0.0)
        with self.assertRaises(ValueError):
            swing.joint_target(toe, (0, 0, 0.35), (0, 0, 0, 0))

    def test_full_ideal_four_leg_cycle_keeps_target_continuous(self):
        planner = PhaseWalk()
        previous_q = STAND[:]
        swing = None
        peaks = [0.0]*4
        completed = []
        for _ in range(5200):
            prior = planner.state
            nominal = planner.update(0.01, 0.003, 0.0)
            body = (planner.body[0], planner.body[1], planner.body[2]+0.022)
            orientation = quaternion(0, 0, planner.yaw)
            toes = measured_feet(body, orientation, previous_q)
            target = list(nominal)
            if planner.state == 'swing':
                if swing is None:
                    swing = MeasuredSwing(planner.leg, toes[planner.leg],
                                          planner.start_foot, 0.0)
                index = planner.leg
                target[3*index:3*index+3] = swing.joint_target(
                    planner.feet[index], body, orientation)
                reached = world_from_body(forward(index,target[3*index:3*index+3]),
                                          body,orientation)
                peaks[index] = max(peaks[index], reached[2]-0.022)
            elif prior == 'swing':
                completed.append(swing.leg)
                swing = None
            self.assertLess(max(abs(a-b) for a,b in zip(target,previous_q)), 0.0081)
            previous_q = target
            if len(completed) == 4:
                break
        self.assertEqual(completed,[3,1,2,0])
        for peak in peaks:
            self.assertGreater(peak,0.024)


if __name__ == '__main__':
    unittest.main()

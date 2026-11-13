"""Measured-world geometry checks; these do not substitute for Gazebo tests."""
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from lite3_gait_core import (STAND, body_from_world, forward, inverse,
                            measured_feet, world_from_body)
from lite3_measured_core import MeasuredSupportStep, URDFCenterOfMass, triangle_margin


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


class MeasuredSupportTests(unittest.TestCase):
    def setUp(self):
        self.body = (0.12, -0.08, -forward(0, STAND[:3])[2]+0.022)
        self.quaternion = quaternion(0, 0, 0.17)

    def test_zero_shift_returns_measured_joints_without_22_mm_ground_bias(self):
        for leg in range(4):
            step = MeasuredSupportStep(self.body, self.quaternion, STAND, leg)
            start = step.targets('shift', 0.0, (self.body, self.quaternion))
            for a, b in zip(start, STAND):
                self.assertAlmostEqual(a, b, places=9)
            self.assertTrue(all(abs(f[2]-0.022)<1e-9 for f in step.anchors))

    def test_real_com_offset_is_inside_correct_three_foot_polygon(self):
        # A forward/left torso COM offset must affect the support shift target.
        for leg in range(4):
            step = MeasuredSupportStep(self.body, self.quaternion, STAND, leg,
                                       com_body=(0.03, -0.02, 0.01))
            com = world_from_body(step.com_body, step.target_body, step.quaternion)
            support = [f for i, f in enumerate(step.anchors) if i != leg]
            self.assertGreaterEqual(triangle_margin(com, support), 0.018-1e-9)
            self.assertAlmostEqual(step.target_body[2], self.body[2], places=9)

    def test_support_world_anchors_are_fixed_during_shift(self):
        step = MeasuredSupportStep(self.body, self.quaternion, STAND, 3)
        for u in (0, 0.25, 0.5, 0.75, 1):
            q = step.targets('shift', u, (self.body, self.quaternion))
            reached = measured_feet(step.command_body(u), self.quaternion, q)
            for a, b in zip(reached, step.anchors):
                for x, y in zip(a, b):
                    self.assertAlmostEqual(x, y, places=8)

    def test_wrong_leg_cannot_supply_expected_leg_lift(self):
        step = MeasuredSupportStep(self.body, self.quaternion, STAND, 3)
        pose = (step.target_body, self.quaternion)
        q = step.targets('shift', 1, pose)
        step.begin_lift(pose, q, q)
        toes = measured_feet(*pose, q)
        wrong_toe = (toes[0][0], toes[0][1], toes[0][2]+0.0264)
        q[:3] = inverse(0, body_from_world(wrong_toe, *pose))
        diagnostics = step.diagnostics(pose, q)
        self.assertAlmostEqual(diagnostics['actual_lift'], 0, places=8)
        self.assertAlmostEqual(diagnostics['support_bottoms']['FL'], 0.0264, places=8)

    def test_below_ground_command_anchor_cannot_erase_requested_clearance(self):
        step = MeasuredSupportStep(self.body, self.quaternion, STAND, 3)
        pose = (step.target_body, self.quaternion)
        actual = step.targets('shift', 1, pose)
        commanded = list(actual)
        toe = measured_feet(*pose, actual)[3]
        commanded[9:] = inverse(3, body_from_world((toe[0], toe[1], toe[2]-0.0206), *pose))
        step.begin_lift(pose, actual, commanded)
        start = step.targets('lift', 0, pose)
        for a, b in zip(start, commanded):
            self.assertAlmostEqual(a, b, places=9)
        lifted = step.targets('lift', 1, pose)
        diagnostics = step.diagnostics(pose, lifted)
        self.assertAlmostEqual(diagnostics['actual_lift'], 0.03, places=8)
        self.assertAlmostEqual(diagnostics['bottoms'][3], 0.03, places=8)

    def test_all_four_legs_target_the_named_foot_with_measured_tilt(self):
        for leg in range(4):
            step = MeasuredSupportStep(self.body, self.quaternion, STAND, leg)
            pose = (step.target_body, self.quaternion)
            q = step.targets('shift', 1, pose)
            step.begin_lift(pose, q, q)
            tilted_pose = ((pose[0][0]+0.002, pose[0][1]-0.001, pose[0][2]-0.003),
                           quaternion(0.025, -0.02, 0.18))
            lifted = step.targets('hold', 0.5, tilted_pose)
            reached = measured_feet(*tilted_pose, lifted)[leg]
            target = step.swing_world('hold', 0.5)
            for a, b in zip(reached, target):
                self.assertAlmostEqual(a, b, places=8)
            self.assertAlmostEqual(reached[2]-0.022, 0.03, places=8)

    def test_lift_hold_transfer_lower_boundaries_are_continuous(self):
        step = MeasuredSupportStep(self.body, self.quaternion, STAND, 1,
                                   stride=(0.009, -0.002))
        pose = (step.target_body, self.quaternion)
        q = step.targets('shift', 1, pose)
        step.begin_lift(pose, q, q)
        boundaries = ((('lift', 1), ('hold', 0)),
                      (('hold', 1), ('transfer', 0)),
                      (('transfer', 1), ('lower', 0)),
                      (('lower', 1), ('settle', 0)))
        for first, second in boundaries:
            for a, b in zip(step.targets(*first, pose), step.targets(*second, pose)):
                self.assertAlmostEqual(a, b, places=9)
        lowered = measured_feet(*pose, step.targets('lower', 1, pose))[1]
        self.assertAlmostEqual(lowered[2]-0.022, 0, places=8)

    def test_triangle_margin_handles_vertex_order_and_outside_com(self):
        triangle = ((0, 0), (1, 0), (0, 1))
        self.assertAlmostEqual(triangle_margin((0.1, 0.1), triangle), 0.1)
        self.assertAlmostEqual(triangle_margin((0.1, 0.1), triangle[::-1]), 0.1)
        self.assertLess(triangle_margin((0.8, 0.8), triangle), 0)

    def test_invalid_target_and_unlatched_lift_fail_explicitly(self):
        step = MeasuredSupportStep(self.body, self.quaternion, STAND, 0)
        with self.assertRaises(ValueError):
            step.targets('lift', 0, (self.body, self.quaternion))
        with self.assertRaises(ValueError):
            step.targets('shift', float('nan'), (self.body, self.quaternion))
        with self.assertRaises(ValueError):
            MeasuredSupportStep(self.body, self.quaternion, STAND, 0, height=0.2)

    def test_successive_hl_fl_hr_fr_lifts_keep_three_support_anchors_and_rate(self):
        # Ideal endpoint execution checks geometry only, with the measured
        # transform following the shift. Actual physics is checked on board.
        body, orientation, joints = self.body, self.quaternion, list(STAND)
        com = (-0.0072, 0.00016, -0.02009)
        for leg in (2, 0, 3, 1):
            step = MeasuredSupportStep(body, orientation, joints, leg, com_body=com)
            previous = list(joints)
            for tick in range(251):
                u = tick/250
                body = step.command_body(u)
                joints = step.targets('shift', u, (body, orientation))
                self.assertLess(max(abs(a-b) for a,b in zip(joints,previous)), 0.0061)
                previous = list(joints)
            pose = (body, orientation)
            step.begin_lift(pose, joints, joints)
            for stage in ('lift', 'lower'):
                for tick in range(151):
                    joints = step.targets(stage, tick/150, pose)
                    self.assertLess(max(abs(a-b) for a,b in zip(joints,previous)), 0.0061)
                    previous = list(joints)
                    metrics = step.diagnostics(pose, joints)
                    self.assertLess(metrics['max_support_floor_error'], 1e-8)
                    if stage == 'lift' and tick == 150:
                        self.assertAlmostEqual(metrics['target_leg_height'], 0.030, places=8)
            self.assertTrue(all(abs(f[2]-0.022)<1e-8 for f in measured_feet(*pose,joints)))


class URDFMassTests(unittest.TestCase):
    def model(self, text):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        path = Path(folder.name)/'model.urdf'
        path.write_text(text, encoding='utf-8')
        return URDFCenterOfMass(path)

    def test_all_link_masses_and_negative_joint_axis_move_full_body_com(self):
        model = self.model('''<robot name="fixture">
          <link name="TORSO"><inertial><mass value="2"/><origin xyz="0.2 0 0"/></inertial></link>
          <link name="SHANK"><inertial><mass value="1"/><origin xyz="0 0 -1"/></inertial></link>
          <link name="FOOT"><inertial><mass value="1"/><origin xyz="0 0 -0.2"/></inertial></link>
          <joint name="knee" type="revolute"><parent link="TORSO"/><child link="SHANK"/>
             <axis xyz="0 -1 0"/></joint>
          <joint name="ankle" type="fixed"><parent link="SHANK"/><child link="FOOT"/>
             <origin xyz="0 0 -1"/></joint>
        </robot>''')
        self.assertAlmostEqual(model.total_mass, 4)
        at_zero = model.body_com({'knee': 0})
        for actual, expected in zip(at_zero, (0.1, 0, -0.55)):
            self.assertAlmostEqual(actual, expected, places=9)
        at_bent = model.body_com({'knee': math.pi/2})
        for actual, expected in zip(at_bent, (0.65, 0, 0)):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_origin_rotation_and_prismatic_axis_are_in_joint_frame(self):
        model = self.model('''<robot name="fixture">
          <link name="TORSO"/>
          <link name="BODY"><inertial><mass value="1"/><origin xyz="1 0 0"/></inertial></link>
          <joint name="slide" type="prismatic"><parent link="TORSO"/><child link="BODY"/>
            <origin xyz="0.2 0.3 0.4" rpy="0 0 1.5707963267948966"/>
            <axis xyz="1 0 0"/></joint>
        </robot>''')
        com = model.body_com({'slide': 0.5})
        for actual, expected in zip(com, (0.2, 1.8, 0.4)):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_missing_feedback_does_not_guess_an_articulated_com(self):
        model = self.model('''<robot name="fixture">
          <link name="TORSO"><inertial><mass value="1"/></inertial></link>
          <link name="HIP"><inertial><mass value="1"/></inertial></link>
          <joint name="hip" type="revolute"><parent link="TORSO"/><child link="HIP"/>
             <axis xyz="-1 0 0"/></joint>
        </robot>''')
        with self.assertRaisesRegex(ValueError, 'Missing measured joint'):
            model.body_com({})

    def test_nonphysical_mass_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Invalid URDF mass'):
            self.model('''<robot name="fixture"><link name="TORSO">
              <inertial><mass value="-1"/></inertial></link></robot>''')


if __name__ == '__main__':
    unittest.main()

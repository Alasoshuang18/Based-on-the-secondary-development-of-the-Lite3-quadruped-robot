import ast
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
CONTROL_SRC = next(p for p in (
    ROOT.parent/'6-3_robot_sport/Lite3_only_Fortress_control/src',
    ROOT.parent/'6-3_robot_sport/src',
) if (p/'lite3_measured_core.py').is_file())
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(CONTROL_SRC))
from lite3_gait_core import STAND
from lite3_measured_core import DiagonalTrot
from payload_speed_trial import controller_tree, planner_class, recenter_error


class SpeedTrialTests(unittest.TestCase):
    def test_recenter_does_not_integrate_a_displacement_already_in_reference(self):
        self.assertEqual(recenter_error((0.,0.),(0.,.010),(0.,.010),1.,0.),[0.,0.])

    def test_recenter_preserves_parallel_velocity_reference(self):
        self.assertEqual(recenter_error((0.,0.),(.020,0.),(0.,.010),1.,0.),[0.,.010])

    def test_stalled_body_does_not_wind_up_and_still_fails_the_actual_com_gate(self):
        t,pose=self.fixture(planner_class(DiagonalTrot,.020))
        with self.assertRaisesRegex(ValueError,'double support did not settle'):
            for i in range(1,65):
                t.update(.02,1.+i*.02,pose,STAND,(0.,0.),(0.,0.),0.,0.,
                         com_body=(.015,0.,.05))
                self.assertLess(math.hypot(*t.reference[:2]),.015)

    def fixture(self, cls):
        pose = ((0., 0., .022+(.20+.21012)*math.cos(.65)), (0., 0., 0., 1.))
        return cls(pose, STAND, 1., period=1., duty=.60, height=.025, joint_rate=3.), pose

    def test_twenty_mm_profile_matches_original_planner(self):
        a, pose = self.fixture(DiagonalTrot)
        b, _ = self.fixture(planner_class(DiagonalTrot, .020, recenter_gain=2.))
        for i in range(1, 5):
            args = (.02, 1.+i*.02, pose, STAND, (0., 0.), (0., 0.), .020, 0.)
            self.assertEqual(a.update(*args), b.update(*args))
            self.assertEqual(a.reference, b.reference)

    def test_com_recenter_converges_faster_with_same_transfer_speed_bound(self):
        a, pose = self.fixture(planner_class(DiagonalTrot, .020, recenter_gain=2.))
        b, _ = self.fixture(planner_class(DiagonalTrot, .020, recenter_gain=3.))
        args=(.02,1.02,pose,STAND,(0.,0.),(0.,0.),0.,0.)
        a.update(*args,com_body=(.008,0.,.05))
        b.update(*args,com_body=(.008,0.,.05))
        old,new=map(lambda t:math.hypot(*t.support_recenter),(a,b))
        self.assertAlmostEqual(new/old,1.5)
        self.assertLessEqual(new,.020*.02)

    def test_trial_extends_speed_but_keeps_original_deceleration(self):
        t, pose = self.fixture(planner_class(DiagonalTrot, .025))
        t.speed = .024
        t.update(.02, 1.02, pose, STAND, (0., 0.), (0., 0.), .025, 0.)
        self.assertAlmostEqual(t.speed, .0244)
        t.update(.02, 1.04, pose, STAND, (0., 0.), (0., 0.), .025, 0., stopping=True)
        self.assertAlmostEqual(t.speed, .024)

    def test_support_slip_guard_still_fails_at_twenty_mm(self):
        t, pose = self.fixture(planner_class(DiagonalTrot, .025))
        t.stage = 'swing'; t.starts = {i:t.anchors[i] for i in t.swing}
        t.ends = dict(t.starts); t.peaks = {i:0. for i in t.swing}; t.clear_time = dict(t.peaks)
        t.anchors[3] = (t.anchors[3][0]+.021, *t.anchors[3][1:])
        # Swing FL+HR means FR+HL support; choose the FR support anchor instead.
        t.anchors[1] = (t.anchors[1][0]+.021, *t.anchors[1][1:])
        with self.assertRaisesRegex(ValueError, 'support foot slipped more than 20 mm: FR'):
            t.update(.02, 1.02, pose, STAND, (0., 0.), (0., 0.), 0., 0.)

    def test_rejects_unknown_base_and_excess_speed(self):
        with self.assertRaises(ValueError): controller_tree("VERSION='unknown'", '<test>')
        with self.assertRaises(ValueError): planner_class(DiagonalTrot, .03)

    def test_node_ast_changes_only_the_specific_trot_speed_ceiling(self):
        source=(CONTROL_SRC/'lite3_controller.py').read_text()
        actual=controller_tree(source,'<test>')
        expected=ast.parse(source.replace('self.trot_speed <= .020', 'self.trot_speed <= .025'))
        self.assertEqual(ast.dump(actual),ast.dump(expected))


if __name__ == '__main__': unittest.main()

import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
CONTROL_SRC = next(p for p in (
    ROOT.parent/'6-3_robot_sport/Lite3_only_Fortress_control/src',
    ROOT.parent/'6-3_robot_sport/src',
) if (p/'lite3_measured_core.py').is_file())
sys.path[:0] = [str(ROOT), str(CONTROL_SRC)]
from lite3_gait_core import STAND, measured_feet
from lite3_measured_core import DiagonalTrot
from payload_stance_control import stance_class


class StanceTests(unittest.TestCase):
    def fixture(self):
        pose=((0.,0.,.022+(.20+.21012)*math.cos(.65)),(0.,0.,0.,1.))
        t=stance_class(DiagonalTrot)(pose,STAND,1.,period=1.,duty=.60,height=.025,joint_rate=3.)
        return t,pose

    def test_world_anchor_targets_do_not_follow_a_distant_virtual_body(self):
        t,pose=self.fixture()
        anchors=list(t.anchors)
        t.reference[0]=.018
        t.command_pose=((.018,0.,pose[0][2]),pose[1])
        q=t.joint_targets(pose,.02)
        actual=measured_feet(*pose,q)
        self.assertAlmostEqual(t.stance_requested_xy,.018)
        self.assertAlmostEqual(t.stance_applied_xy,.010)
        self.assertEqual(t.reference[0],.018)
        self.assertEqual(t.anchors,anchors)
        for a,b in zip(actual,anchors):
            self.assertAlmostEqual(math.dist(a[:2],b[:2]),.010)

    def test_small_corrections_are_unchanged_and_reference_watchdog_remains(self):
        t,pose=self.fixture()
        t.command_pose=((.003,.001,pose[0][2]),pose[1])
        t.joint_targets(pose,.02)
        self.assertEqual(t.command_pose[0][:2],(.003,.001))
        t.reference[0]=.05
        with self.assertRaisesRegex(ValueError,'follow reference within 40 mm'):
            t.update(.02,1.02,pose,STAND,(0.,0.),(0.,0.),.020,0.)

    def test_twenty_mm_support_guard_and_air_phase_time_guard_are_retained(self):
        t,pose=self.fixture()
        t.stage='swing'; t.starts={i:t.anchors[i] for i in t.swing}
        t.ends=dict(t.starts); t.peaks={i:0. for i in t.swing}; t.clear_time=dict(t.peaks)
        t.anchors[1]=(t.anchors[1][0]+.021,*t.anchors[1][1:])
        with self.assertRaisesRegex(ValueError,'support foot slipped more than 20 mm: FR'):
            t.update(.02,1.02,pose,STAND,(0.,0.),(0.,0.),.020,0.)
        t,pose=self.fixture()
        t.stage='swing'
        with self.assertRaisesRegex(ValueError,'not a verified near-ground landing'):
            t.update(.05,1.05,pose,STAND,(0.,0.),(0.,0.),.020,0.)


if __name__=='__main__': unittest.main()

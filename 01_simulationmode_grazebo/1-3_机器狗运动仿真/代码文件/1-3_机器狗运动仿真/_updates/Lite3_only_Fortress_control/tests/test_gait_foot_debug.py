import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from foot_debug_core import world_feet, PosePairs
from lite3_gait_core import Crawl, STAND, forward


class FootTests(unittest.TestCase):
    def test_stand_bottom_on_ground(self):
        height=-forward(0,STAND[:3])[2]+0.022
        for point in world_feet((0,0,height),(0,0,0,1),STAND):
            self.assertAlmostEqual(point[2]-0.022,0,places=10)

    def test_planned_swing_really_changes_world_height(self):
        gait=Crawl()
        maxima=[0]*4
        for _ in range(2500):
            q=gait.update(0.01,0.003,0)
            pose=list(gait.body);pose[2]+=0.022
            quat=(0,0,math.sin(gait.yaw/2),math.cos(gait.yaw/2))
            feet=world_feet(pose,quat,q)
            for i,f in enumerate(feet):
                bottom=f[2]-0.022
                self.assertAlmostEqual(bottom,gait.feet[i][2],places=8)
                maxima[i]=max(maxima[i],bottom)
        self.assertTrue(all(abs(h-gait.step_height)<1e-5 for h in maxima))

    def test_body_yaw_rotation_and_translation(self):
        base=world_feet((0,0,0),(0,0,0,1),STAND)
        moved=world_feet((1,2,3),(0,0,math.sqrt(.5),math.sqrt(.5)),STAND)
        for a,b in zip(base,moved):
            self.assertAlmostEqual(b[0],1-a[1],places=9)
            self.assertAlmostEqual(b[1],2+a[0],places=9)
            self.assertAlmostEqual(b[2],3+a[2],places=9)

    def test_pair_retains_array_stamp_when_tf_stamp_absent(self):
        pairs=PosePairs();values=[(0,0,0,0,0,0,1),(1,2,.35,0,0,0,1)]
        self.assertIsNone(pairs.names(['ground_plane','lite3_control'],values))
        self.assertEqual(pairs.time(123.4,values),(123.4,(1,2,.35),(0,0,0,1)))

    def test_array_first_order_and_mismatch(self):
        pairs=PosePairs();a=[(1,0,.35,0,0,0,1)];b=[(2,0,.35,0,0,0,1)]
        self.assertIsNone(pairs.time(5,a))
        self.assertIsNone(pairs.names(['lite3_control'],b))
        self.assertEqual(pairs.names(['lite3_control'],a)[0],5)

    def test_zero_stamp_never_guesses_time(self):
        pairs=PosePairs();values=[(0,0,.35,0,0,0,1)]
        self.assertIsNone(pairs.names(['lite3_control'],values))
        self.assertIsNone(pairs.time(0,values))


if __name__=='__main__':
    unittest.main()

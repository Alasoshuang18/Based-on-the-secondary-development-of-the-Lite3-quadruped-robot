"""Kinematic phase tests; no Gazebo dynamics validation."""
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from lite3_gait_core import PhaseWalk, bezier_swing, STAND, LIMITS


class PhaseTests(unittest.TestCase):
    def test_bezier_endpoints_and_midpoint(self):
        self.assertEqual(bezier_swing(0),(0,0,0,0))
        self.assertEqual(bezier_swing(1),(1,0,0,0))
        x,z,dx,dz=bezier_swing(.5)
        self.assertEqual((x,z,dz),(.5,1,0))
        self.assertGreater(dx,0)

    def test_bezier_velocity_matches_finite_difference(self):
        for u in (.1,.3,.49,.51,.7,.9):
            x,z,dx,dz=bezier_swing(u)
            a,b=bezier_swing(u-1e-6),bezier_swing(u+1e-6)
            self.assertAlmostEqual(dx,(b[0]-a[0])/2e-6,places=7)
            self.assertAlmostEqual(dz,(b[1]-a[1])/2e-6,places=7)

    def test_measured_schedule_duty_and_offsets(self):
        g=PhaseWalk(); starts=[]; counts=[0]*4
        previous='idle'
        for i in range(6000):
            g.update(.01,.003,0)
            for j,c in enumerate(g.contact_plan):
                counts[j]+=not c
            if g.state=='swing' and previous!='swing': starts.append((g.leg,i*.01))
            previous=g.state
        self.assertEqual([i for i,t in starts[:4]],[3,1,2,0])
        for j in range(4):
            self.assertAlmostEqual(1-counts[j]/6000,.82,delta=.005)
        first=dict(starts[:4])
        for j in range(4):
            offset=((first[j]-first[0])/g.cycle_time)%1
            self.assertAlmostEqual(offset,g.OFFSETS[j],delta=.003)

    def test_all_directions_limits_rate_and_stop(self):
        for v,w in ((.003,0),(-.003,0),(0,.01),(0,-.01)):
            g=PhaseWalk(); last=STAND[:]
            for i in range(30000):
                q=g.update(.01,v,w)
                self.assertLess(max(abs(a-b) for a,b in zip(q,last))/.01,.8)
                self.assertGreaterEqual(sum(g.contact_plan),3)
                for k,x in enumerate(q):
                    lo,hi=LIMITS[k%3]
                    self.assertTrue(math.isfinite(x) and lo+.015<=x<=hi-.015)
                last=q
            self.assertGreater(g.body[0]*v if v else g.yaw*w,0)
            for _ in range(500): g.update(.01,0,0)
            self.assertEqual(g.state,'idle')
            self.assertEqual(g.contact_plan,[True]*4)

    def test_invalid_timing_rejected(self):
        for opts in ({'duty_factor':.5},{'duty_factor':.75},{'cycle_period':1},
                     {'cycle_period':float('nan')}):
            with self.assertRaises(ValueError): PhaseWalk(**opts)

    def test_reversals_with_attitude_correction(self):
        commands=[(.003,0),(0,0),(-.003,0),(0,.01),(0,0),(0,-.01)]
        for correction in (-.05,.05):
            g=PhaseWalk();last=STAND[:]
            for i in range(30000):
                v,w=commands[(i//1500)%6]
                q=g.update(.01,v,w,correction,correction)
                self.assertLess(max(abs(a-b) for a,b in zip(q,last))/.01,.8)
                last=q


if __name__=='__main__': unittest.main()

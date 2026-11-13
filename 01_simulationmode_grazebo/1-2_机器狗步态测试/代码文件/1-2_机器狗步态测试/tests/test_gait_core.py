"""Numerical tests only; does not prove Gazebo contact/balance stability."""
import math
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from lite3_gait_core import Crawl, forward, inverse, STAND, LIMITS, support_target, swing_profile


class KinematicsTests(unittest.TestCase):
    def test_forward_inverse_roundtrip_all_legs(self):
        rng = random.Random(21)
        for leg in range(4):
            for _ in range(500):
                q = [rng.uniform(-0.3, 0.3), rng.uniform(-1.1, -0.2), rng.uniform(0.7, 2.2)]
                restored = inverse(leg, forward(leg, q))
                for expected, actual in zip(q, restored):
                    self.assertAlmostEqual(expected, actual, places=9)

    def test_reject_unreachable_target(self):
        with self.assertRaises(ValueError):
            inverse(0, (3, 0, -0.3))

    def test_idle_holds_verified_stand(self):
        gait = Crawl()
        for _ in range(500):
            q = gait.update(0.01, 0, 0)
            for actual, expected in zip(q, STAND):
                self.assertAlmostEqual(actual, expected, places=9)


class CrawlTests(unittest.TestCase):
    def check(self, q, previous):
        self.assertEqual(len(q), 12)
        for i, value in enumerate(q):
            self.assertTrue(math.isfinite(value))
            lower, upper = LIMITS[i % 3]
            self.assertGreaterEqual(value, lower+0.015)
            self.assertLessEqual(value, upper-0.015)
        self.assertLess(max(abs(a-b) for a,b in zip(q,previous))/0.01, 0.8)

    def test_transfer_only_after_clearance_and_vertical_touchdown(self):
        for i in range(1001):
            t = i/1000
            x,z = swing_profile(t)
            if 0 < x < 1:
                self.assertAlmostEqual(z, 1.0)
            if t <= 0.30:
                self.assertAlmostEqual(x, 0.0)
            if t >= 0.70:
                self.assertAlmostEqual(x, 1.0)
        self.assertEqual(swing_profile(0), (0.0,0.0))
        self.assertEqual(swing_profile(1), (1.0,0.0))

    def test_inset_support_margin_and_smaller_initial_shift(self):
        gait = Crawl()
        for omitted in range(4):
            points = [f[:2] for i,f in enumerate(gait.feet) if i != omitted]
            target = support_target(points, [0,0], 0.018)
            centroid = [sum(p[k] for p in points)/3 for k in (0,1)]
            self.assertLess(math.hypot(*target), math.hypot(*centroid)/2)
            for i in range(3):
                a,b = points[i],points[(i+1)%3]
                distance = abs((b[0]-a[0])*(target[1]-a[1])-
                               (b[1]-a[1])*(target[0]-a[0]))/math.dist(a,b)
                self.assertGreaterEqual(distance,0.018-1e-10)

    def test_stance_anchors_fixed_through_swing(self):
        gait = Crawl()
        previous = [f[:] for f in gait.feet]
        for _ in range(3000):
            gait.update(0.01,0.003,0)
            for i in range(4):
                if i != gait.leg:
                    self.assertEqual(gait.feet[i], previous[i])
            previous = [f[:] for f in gait.feet]

    def test_300_second_directional_rollouts(self):
        for v,w in ((0.003,0),(-0.003,0),(0,0.01),(0,-0.01)):
            gait = Crawl()
            last = STAND[:]
            for _ in range(30000):
                q = gait.update(0.01,v,w)
                self.check(q,last)
                self.assertLessEqual(sum(f[2]>1e-7 for f in gait.feet),1)
                last = q
            if v:
                self.assertGreater(gait.body[0]*v,0.001)
            else:
                self.assertGreater(gait.yaw*w,0.005)

    def test_stop_in_each_phase_lands_and_recenters(self):
        for duration in (0.4, 1.8, 2.65, 4.0, 8.0):
            gait = Crawl()
            for _ in range(round(duration/0.01)):
                gait.update(0.01,0.003,0)
            last = gait.update(0.01,0,0)
            for _ in range(400):
                q = gait.update(0.01,0,0)
                self.check(q,last)
                last = q
            self.assertEqual(gait.state,'idle')
            self.assertTrue(all(f[2] == 0.0 for f in gait.feet))

    def test_reversals_and_bounded_balance_correction(self):
        commands = [(0.003,0),(0,0),(-0.003,0),(0,0.01),(0,0),(0,-0.01)]
        for correction in (-0.05,0.05):
            gait = Crawl()
            last = STAND[:]
            for i in range(60000):
                v,w = commands[(i//1500)%len(commands)]
                q = gait.update(0.01,v,w,correction,correction)
                self.check(q,last)
                last = q

    def test_time_step_guard(self):
        with self.assertRaises(ValueError):
            Crawl().update(1.0,0.003,0)


if __name__ == '__main__':
    unittest.main()

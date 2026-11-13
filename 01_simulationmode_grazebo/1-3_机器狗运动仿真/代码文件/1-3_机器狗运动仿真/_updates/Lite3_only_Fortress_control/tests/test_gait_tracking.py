from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from lite3_gait_core import TrackingGuard, JOINTS, clock_step_kind


class TrackingTests(unittest.TestCase):
    def sample(self, guard, stamp, error):
        return guard.update(JOINTS,[error]+[0.0]*11,stamp)

    def test_transient_recovers_without_fault(self):
        g=TrackingGuard()
        self.sample(g,1.0,0.4)
        self.sample(g,1.1,0.4)
        self.sample(g,1.2,0.0)
        self.assertEqual(g.failure,'')
        self.assertIsNone(g.since)

    def test_persistent_error_faults_with_joint_name(self):
        g=TrackingGuard()
        for i in range(12):
            self.sample(g,1+i*0.03,0.4)
        self.assertIn('FL_HipX_joint',g.failure)

    def test_extreme_error_faults_immediately(self):
        g=TrackingGuard();self.sample(g,1,0.9)
        self.assertIn('immediate',g.failure)

    def test_repeated_timestamp_cannot_accumulate_duration(self):
        g=TrackingGuard();self.sample(g,1,0.4)
        for _ in range(100):
            self.assertFalse(self.sample(g,1,0.4))
        self.assertEqual(g.failure,'')
        self.assertFalse(self.sample(g,0.9,0.4))

    def test_gap_does_not_count_as_sustained_error(self):
        g=TrackingGuard();self.sample(g,1,0.4);self.sample(g,2,0.4)
        self.assertEqual(g.failure,'')
        self.assertEqual(g.since,2)

    def test_shuffled_names_preserve_error_identity(self):
        g=TrackingGuard()
        names=JOINTS[::-1]
        g.update(names,[0.9]+[0]*11,1)
        self.assertEqual(g.joint,'HR_Knee_joint')

    def test_invalid_data_does_not_refresh_state(self):
        g=TrackingGuard();self.sample(g,1,0)
        self.assertFalse(self.sample(g,2,float('nan')))
        self.assertFalse(g.update(JOINTS,[0]*11,2))
        self.assertEqual(g.stamp,1)

    def test_clock_step_classification(self):
        for delta in (0.0,0.02,0.2):
            self.assertEqual(clock_step_kind(delta),'normal')
        for delta in (0.21,0.5,1.0):
            self.assertEqual(clock_step_kind(delta),'hold')
        for delta in (-0.001,1.01,float('nan')):
            self.assertEqual(clock_step_kind(delta),'fault')


if __name__=='__main__':
    unittest.main()

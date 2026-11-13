import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from payload_trot_diagnostics import TrotTrace, snapshot


def fixture():
    anchors = [(0., 0., .022)] * 4
    feet = list(anchors)
    feet[3] = (.021, 0., .022)
    trot = NS(anchors=anchors, swing=(1, 2), stage='swing', elapsed=.2,
              completed=42, planned=anchors, reference=[0., 0., .35], yaw=0.,
              reference_error=.003, velocity=[.02, 0.], speed=.02, turn=0.)
    return NS(trot=trot, measured=feet, last_sim=12., measured_stamp=12.,
              body_pose=((0., 0., .35), (0., 0., 0., 1.)), roll=0., pitch=0.,
              vx=.02, wz=0., stopping=False, q=[0.] * 12,
              last_target=[0.] * 12, imu_stamp=12., tracking=NS(maximum=0.))


class TraceTests(unittest.TestCase):
    def test_identifies_slipped_support_without_counting_swing_legs(self):
        row = snapshot(fixture())
        self.assertEqual(row['swing'], ['FR', 'HL'])
        self.assertEqual(row['support_offset_mm'], {'FL': 0., 'HR': 21.})

    def test_is_bounded_and_preserves_mutated_planner_history(self):
        node = fixture()
        trace = TrotTrace(capacity=2)
        for x in (.01, .02, .03):
            node.trot.reference[0] = x
            trace.append(node)
        node.trot.reference[0] = 99.
        self.assertEqual([r['reference_world_m'][0] for r in trace.rows], [.02, .03])

    def test_fault_evidence_is_valid_json_and_does_not_change_inputs(self):
        node = fixture()
        node.tracking.maximum = float('nan')
        trace = TrotTrace()
        trace.append(node)
        with tempfile.TemporaryDirectory() as directory:
            path = trace.save_fault(directory, 'HR support foot slipped')
            report = json.loads(path.read_text())
        self.assertEqual(report['slip_limit_mm'], 20.)
        self.assertIsNone(report['samples'][0]['tracking_error_rad'])
        self.assertEqual(node.trot.reference, [0., 0., .35])
        self.assertEqual(node.trot.speed, .02)

    def test_peak_statistics_survive_ring_buffer_eviction(self):
        node = fixture()
        trace = TrotTrace(capacity=1)
        trace.append(node)
        node.measured[3] = (0., 0., .022)
        trace.append(node)
        self.assertEqual(trace.rows[0]['support_offset_mm']['HR'], 0.)
        self.assertEqual(trace.maximum_support_offset_mm['HR'], 21.)


if __name__ == '__main__':
    unittest.main()

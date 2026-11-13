"""Regressions for coherent feedback under independent callback delivery.

These tests exercise timestamp pairing and retained snapshot freshness only;
they do not validate Gazebo physics, transport latency or controller execution.
"""
from dataclasses import FrozenInstanceError
import math
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from lite3_feedback_sync import FeedbackSynchronizer


POSITION = (0.0, 0.0, 0.348)
QUATERNION = (0.0, 0.0, 0.0, 1.0)


def joints(value=0.0):
    return tuple(value for _ in range(12))


class FeedbackSyncTests(unittest.TestCase):
    def test_no_snapshot_until_both_components_exist(self):
        pose_only = FeedbackSynchronizer()
        joint_only = FeedbackSynchronizer()
        self.assertIsNone(pose_only.add_pose(1.0, POSITION, QUATERNION))
        self.assertIsNone(joint_only.add_joints(1.0, joints()))
        self.assertIsNone(pose_only.latest)
        self.assertIsNone(joint_only.latest)

    def test_snapshot_preserves_two_stamps_and_conservative_freshness(self):
        sync = FeedbackSynchronizer()
        sync.add_joints(10.0, joints(0.2))
        pair = sync.add_pose(10.01, POSITION, QUATERNION)
        self.assertEqual(pair.q, joints(0.2))
        self.assertEqual(pair.position, POSITION)
        self.assertEqual(pair.quaternion, QUATERNION)
        self.assertEqual(pair.joint_stamp, 10.0)
        self.assertEqual(pair.pose_stamp, 10.01)
        self.assertEqual(pair.stamp, 10.0)
        self.assertAlmostEqual(pair.skew, 0.01)
        with self.assertRaises(FrozenInstanceError):
            pair.joint_stamp = 20.0

    def test_delayed_joint_uses_buffered_pose_despite_raw_latest_skew(self):
        # The newest raw pose is 40 ms ahead, but the earlier matching pose
        # remains usable. Rejecting raw_latest alone caused the user's FAULT.
        sync = FeedbackSynchronizer()
        sync.add_joints(1.04, joints(0.04))
        sync.add_pose(1.04, POSITION, QUATERNION)
        sync.add_pose(1.06, (0.01, 0.0, 0.348), QUATERNION)
        sync.add_pose(1.10, (0.02, 0.0, 0.348), QUATERNION)
        pair = sync.add_joints(1.06, joints(0.06))
        self.assertGreater(sync.poses[-1][0]-sync.joints[-1][0], 0.025)
        self.assertEqual(pair.pose_stamp, 1.06)
        self.assertEqual(pair.joint_stamp, 1.06)
        self.assertEqual(pair.position, (0.01, 0.0, 0.348))
        self.assertEqual(pair.q, joints(0.06))
        self.assertEqual(pair.skew, 0.0)

    def test_delayed_pose_can_use_earlier_buffered_joint(self):
        sync = FeedbackSynchronizer()
        sync.add_joints(2.0, joints(2.0))
        sync.add_joints(2.02, joints(2.02))
        sync.add_joints(2.07, joints(2.07))
        pair = sync.add_pose(2.02, POSITION, QUATERNION)
        self.assertEqual(pair.joint_stamp, 2.02)
        self.assertEqual(pair.q, joints(2.02))
        self.assertGreater(sync.joints[-1][0]-pair.pose_stamp, 0.025)

    def test_100_hz_joints_and_50_hz_pose_with_reordered_callbacks(self):
        rng = random.Random(51)
        callbacks = []
        for index in range(201):
            stamp = 1.0+0.01*index
            callbacks.append((stamp+rng.uniform(0.0, 0.018), 'joint', stamp))
        for index in range(101):
            stamp = 1.0+0.02*index
            callbacks.append((stamp+rng.uniform(0.0, 0.045), 'pose', stamp))
        sync = FeedbackSynchronizer()
        emitted = []
        for _, kind, stamp in sorted(callbacks):
            if kind == 'pose':
                pair = sync.add_pose(stamp, (stamp, 0.0, 0.348), QUATERNION)
            else:
                pair = sync.add_joints(stamp, joints(stamp))
            if pair is not None:
                emitted.append(pair)
                self.assertLessEqual(pair.skew, 0.025+1e-12)
                self.assertEqual(pair.q, joints(pair.joint_stamp))
                self.assertEqual(pair.position[0], pair.pose_stamp)
        # A 45 ms pose arrival delay can let newer 20 ms samples overtake
        # earlier ones. Skipping that older backlog is correct; consuming a
        # pose twice merely to reach an arbitrary output count is not.
        self.assertGreater(len(emitted), 1)
        self.assertTrue(all(a.stamp < b.stamp for a, b in zip(emitted, emitted[1:])))
        self.assertTrue(all(a.pose_stamp < b.pose_stamp for a, b in zip(emitted, emitted[1:])))
        self.assertTrue(all(a.joint_stamp < b.joint_stamp for a, b in zip(emitted, emitted[1:])))
        self.assertEqual(len({p.pose_stamp for p in emitted}), len(emitted))
        self.assertEqual(len({p.joint_stamp for p in emitted}), len(emitted))
        # Under this bounded jitter, coherent inputs must continue across the
        # full interval, with no gap approaching the node's 200 ms watchdog.
        self.assertAlmostEqual(emitted[0].stamp, 1.0)
        self.assertLessEqual(max(b.stamp-a.stamp for a, b in zip(emitted, emitted[1:])),
                             0.08+1e-12)
        self.assertAlmostEqual(sync.latest.stamp, 3.0)

    def test_matching_backlog_can_arrive_in_reverse_order(self):
        sync = FeedbackSynchronizer()
        for stamp in (1.0, 1.02, 1.04, 1.06):
            sync.add_pose(stamp, (stamp, 0.0, 0.348), QUATERNION)
        pair = sync.add_joints(1.06, joints(1.06))
        self.assertEqual(pair.stamp, 1.06)
        for stamp in (1.04, 1.02, 1.0):
            self.assertIsNone(sync.add_joints(stamp, joints(stamp)))
            self.assertIs(sync.latest, pair)
        sync.add_joints(1.08, joints(1.08))
        advanced = sync.add_pose(1.08, POSITION, QUATERNION)
        self.assertEqual(advanced.stamp, 1.08)

    def test_out_of_order_old_pair_never_replaces_latest(self):
        sync = FeedbackSynchronizer()
        sync.add_pose(20.0, POSITION, QUATERNION)
        original = sync.add_joints(20.0, joints(0.2))
        self.assertIsNone(sync.add_pose(10.0, POSITION, QUATERNION))
        self.assertIsNone(sync.add_joints(10.0, joints(0.1)))
        self.assertIs(sync.latest, original)
        self.assertEqual(sync.latest.q, joints(0.2))

    def test_duplicate_timestamp_cannot_refresh_or_replace_content(self):
        sync = FeedbackSynchronizer()
        sync.add_pose(4.0, POSITION, QUATERNION)
        original = sync.add_joints(4.0, joints(0.1))
        self.assertIsNone(sync.add_pose(4.0, (99.0, 0.0, 0.0), QUATERNION))
        self.assertIsNone(sync.add_joints(4.0, joints(99.0)))
        self.assertIs(sync.latest, original)
        self.assertEqual(sync.poses[0][1][0], POSITION)
        self.assertEqual(sync.joints[0][1], joints(0.1))

    def test_unmatched_updates_retain_snapshot_without_faking_freshness(self):
        sync = FeedbackSynchronizer()
        sync.add_pose(5.0, POSITION, QUATERNION)
        original = sync.add_joints(5.0, joints())
        for stamp in (5.10, 5.20, 5.30, 5.40):
            self.assertIsNone(sync.add_pose(stamp, POSITION, QUATERNION))
        self.assertIs(sync.latest, original)
        self.assertEqual(sync.latest.stamp, 5.0)
        self.assertGreater(sync.mismatch_skips, 0)

    def test_new_joint_cannot_reuse_already_consumed_pose(self):
        sync = FeedbackSynchronizer()
        sync.add_pose(2.01, POSITION, QUATERNION)
        original = sync.add_joints(2.0, joints(2.0))
        # min(pair stamps) could advance here, but the pose is not new.
        self.assertIsNone(sync.add_joints(2.01, joints(2.01)))
        self.assertIs(sync.latest, original)
        pair = sync.add_pose(2.02, POSITION, QUATERNION)
        self.assertIsNotNone(pair)
        self.assertGreater(pair.pose_stamp, original.pose_stamp)
        self.assertGreater(pair.joint_stamp, original.joint_stamp)

    def test_new_pose_cannot_reuse_already_consumed_joint(self):
        sync = FeedbackSynchronizer()
        sync.add_joints(3.01, joints(3.01))
        original = sync.add_pose(3.0, POSITION, QUATERNION)
        self.assertIsNone(sync.add_pose(3.01, POSITION, QUATERNION))
        self.assertIs(sync.latest, original)
        pair = sync.add_joints(3.02, joints(3.02))
        self.assertIsNotNone(pair)
        self.assertGreater(pair.pose_stamp, original.pose_stamp)
        self.assertGreater(pair.joint_stamp, original.joint_stamp)

    def test_different_clock_epochs_never_pair(self):
        sync = FeedbackSynchronizer()
        for index in range(50):
            self.assertIsNone(sync.add_pose(10.0+0.02*index, POSITION, QUATERNION))
            self.assertIsNone(sync.add_joints(1791946635.0+0.01*index, joints()))
        self.assertIsNone(sync.latest)
        self.assertGreater(sync.mismatch_skips, 0)

    def test_pairing_tolerance_boundary_not_relaxed(self):
        boundary = FeedbackSynchronizer(tolerance=0.025)
        boundary.add_pose(1.0, POSITION, QUATERNION)
        self.assertIsNotNone(boundary.add_joints(1.025, joints()))
        outside = FeedbackSynchronizer(tolerance=0.025)
        outside.add_pose(1.0, POSITION, QUATERNION)
        self.assertIsNone(outside.add_joints(1.02501, joints()))
        self.assertIsNone(outside.latest)

    def test_buffers_remain_sorted_and_bounded_during_backlog(self):
        sync = FeedbackSynchronizer(max_samples=5)
        for stamp in (1.08, 1.0, 1.10, 1.06, 1.02, 1.12, 1.04):
            sync.add_pose(stamp, POSITION, QUATERNION)
            sync.add_joints(stamp, joints(stamp))
        expected = [1.04, 1.06, 1.08, 1.10, 1.12]
        self.assertEqual([s[0] for s in sync.poses], expected)
        self.assertEqual([s[0] for s in sync.joints], expected)
        self.assertLessEqual(len(sync.poses), 5)
        self.assertLessEqual(len(sync.joints), 5)
        self.assertEqual(sync.latest.stamp, 1.12)

    def test_input_mutation_cannot_modify_retained_snapshot(self):
        sync = FeedbackSynchronizer()
        position, quaternion, q = list(POSITION), list(QUATERNION), list(joints(0.1))
        sync.add_pose(1.0, position, quaternion)
        pair = sync.add_joints(1.0, q)
        position[0], quaternion[3], q[0] = 10.0, 0.0, 10.0
        self.assertEqual(pair.position, POSITION)
        self.assertEqual(pair.quaternion, QUATERNION)
        self.assertEqual(pair.q, joints(0.1))

    def test_invalid_feedback_is_rejected_before_contaminating_buffers(self):
        sync = FeedbackSynchronizer()
        for stamp in (0.0, -1.0, math.nan, math.inf):
            with self.subTest(stamp=stamp):
                with self.assertRaises(ValueError):
                    sync.add_pose(stamp, POSITION, QUATERNION)
                with self.assertRaises(ValueError):
                    sync.add_joints(stamp, joints())
        for position, quaternion in (((0.0, 0.0), QUATERNION),
                                     ((math.nan, 0.0, 0.348), QUATERNION),
                                     (POSITION, (0.0, 0.0, 0.0, 0.0)),
                                     (POSITION, (0.0, 0.0, 0.0, math.inf))):
            with self.assertRaises(ValueError):
                sync.add_pose(1.0, position, quaternion)
        for q in (joints()[:11], (math.nan,)+joints()[1:], (math.inf,)+joints()[1:]):
            with self.assertRaises(ValueError):
                sync.add_joints(1.0, q)
        self.assertEqual(sync.poses, [])
        self.assertEqual(sync.joints, [])
        self.assertIsNone(sync.latest)


if __name__ == '__main__':
    unittest.main()

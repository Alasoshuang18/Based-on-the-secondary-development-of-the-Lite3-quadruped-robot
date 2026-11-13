"""Bounded timestamp pairing for asynchronous pose and joint callbacks.

Keep the 25 ms measurement tolerance. The latest received messages need not
be each other's matching sample; a fresh coherent snapshot is the control input.
"""
from bisect import bisect_left, bisect_right
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PairedFeedback:
    position: tuple
    quaternion: tuple
    q: tuple
    pose_stamp: float
    joint_stamp: float

    @property
    def stamp(self):
        # Conservative freshness: neither component is newer than this bound.
        return min(self.pose_stamp, self.joint_stamp)

    @property
    def skew(self):
        return abs(self.pose_stamp-self.joint_stamp)


class FeedbackSynchronizer:
    def __init__(self, tolerance=.025, max_samples=200):
        if not math.isfinite(tolerance) or not 0 < tolerance <= .05:
            raise ValueError('Invalid feedback pairing tolerance')
        if not isinstance(max_samples,int) or max_samples < 2:
            raise ValueError('Invalid feedback buffer size')
        self.tolerance, self.max_samples = tolerance, max_samples
        self.poses, self.joints = [], []
        self.latest = None
        self.mismatch_skips = 0

    def _insert(self, samples, stamp, value):
        if not math.isfinite(stamp) or stamp <= 0:
            raise ValueError('Feedback timestamp must be finite and positive')
        index = bisect_left([sample[0] for sample in samples], stamp)
        if index < len(samples) and samples[index][0] == stamp:
            return False  # Repeated timestamps cannot invent new measurements.
        samples.insert(index, (stamp,value))
        if len(samples) > self.max_samples:
            del samples[:len(samples)-self.max_samples]
        return True

    def add_pose(self, stamp, position, quaternion):
        if (len(position)!=3 or len(quaternion)!=4 or
                not all(math.isfinite(v) for v in (*position,*quaternion)) or
                sum(v*v for v in quaternion) < .25):
            raise ValueError('Invalid pose feedback')
        if not self._insert(self.poses,stamp,(tuple(position),tuple(quaternion))):
            return None
        return self._pair()

    def add_joints(self, stamp, positions):
        if len(positions)!=12 or not all(math.isfinite(v) for v in positions):
            raise ValueError('Invalid twelve-joint feedback')
        if not self._insert(self.joints,stamp,tuple(positions)):
            return None
        return self._pair()

    def _pair(self):
        if not self.poses or not self.joints:
            return None
        stamps = [sample[0] for sample in self.joints]
        candidate = None
        rank = None
        minimum = self.latest.stamp if self.latest is not None else -math.inf
        first_unused = 0 if self.latest is None else bisect_right(stamps,self.latest.joint_stamp)
        for pose_stamp,(position,quaternion) in reversed(self.poses):
            if self.latest is not None and pose_stamp <= self.latest.pose_stamp:
                break
            index = bisect_left(stamps,pose_stamp,lo=first_unused)
            adjacent = [j for j in (index-1,index) if first_unused <= j < len(stamps)]
            if not adjacent:
                continue
            selected = min(adjacent,key=lambda j: abs(stamps[j]-pose_stamp))
            joint_stamp,q = self.joints[selected]
            pair = PairedFeedback(position,quaternion,q,pose_stamp,joint_stamp)
            if pair.skew > self.tolerance+1e-12 or pair.stamp <= minimum:
                continue
            quality = (pair.stamp,-pair.skew)
            if rank is None or quality > rank:
                candidate,rank = pair,quality
        if candidate is None:
            self.mismatch_skips += 1
            return None
        self.latest = candidate
        return candidate

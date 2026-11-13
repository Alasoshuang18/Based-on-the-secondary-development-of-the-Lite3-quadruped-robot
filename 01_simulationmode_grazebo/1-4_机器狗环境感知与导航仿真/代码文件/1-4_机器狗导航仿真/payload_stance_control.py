"""Measured-pose stance servo for the 6-4 position-interface gait.

Keep the world anchors and the original absolute-reference watchdog. Bound the
XY pose used for stance IK relative to the measured body: an advancing virtual
body must not command every grounded foot far away from its world anchor while
the planner waits for the next support line to settle.
"""
import math


def stance_class(original, limit=.010):
    if not math.isfinite(limit) or not 0 < limit <= .010:
        raise ValueError('Invalid measured stance XY correction bound')

    class MeasuredStanceTrot(original):
        def joint_targets(self, pose, dt):
            target_position, orientation = self.command_pose
            measured_position = pose[0]
            delta = [target_position[k]-measured_position[k] for k in range(2)]
            requested = math.hypot(*delta)
            scale = min(1., limit/max(requested, 1e-12))
            applied = [measured_position[k]+scale*delta[k] for k in range(2)]
            self.stance_requested_xy = requested
            self.stance_applied_xy = requested*scale
            if scale < 1.:
                self.stance_pose_clips = getattr(self, 'stance_pose_clips', 0)+1
            # Height, attitude PD, yaw, foot anchors and absolute reference are
            # untouched. All role crossfades and joint guards run in the base.
            self.command_pose = ((*applied, target_position[2]), orientation)
            return super().joint_targets(pose, dt)

    return MeasuredStanceTrot

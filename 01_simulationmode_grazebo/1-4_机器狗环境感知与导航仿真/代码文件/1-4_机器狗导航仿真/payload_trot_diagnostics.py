"""Bounded, read-only snapshots of the reused trot planner (no ROS dependency)."""
from collections import deque
import json
import math
from pathlib import Path
import time

LEGS = ('FL', 'FR', 'HL', 'HR')


def snapshot(node):
    trot = getattr(node, 'trot', None)
    if trot is None:
        return None
    feet = getattr(node, 'measured', None)
    anchors = trot.anchors
    support = [i for i in range(4) if i not in trot.swing]
    slips = {} if feet is None else {
        LEGS[i]: 1000 * math.dist(feet[i][:2], anchors[i][:2]) for i in support}
    tracking = getattr(node, 'tracking', None)
    return {
        'sim': node.last_sim, 'measurement_stamp': node.measured_stamp,
        'phase': trot.stage, 'phase_elapsed': trot.elapsed,
        'completed_half_steps': trot.completed, 'swing': [LEGS[i] for i in trot.swing],
        'body_pose': node.body_pose, 'roll': node.roll, 'pitch': node.pitch,
        'feet_world_m': feet, 'anchors_world_m': anchors,
        'support_offset_mm': slips, 'planned_feet_world_m': trot.planned,
        'reference_world_m': trot.reference, 'reference_yaw': trot.yaw,
        'reference_error_m': trot.reference_error,
        'stance_requested_xy_m': getattr(trot, 'stance_requested_xy', None),
        'stance_applied_xy_m': getattr(trot, 'stance_applied_xy', None),
        'stance_pose_clips': getattr(trot, 'stance_pose_clips', 0),
        'filtered_body_velocity': trot.velocity,
        'request_vx': node.vx, 'request_wz': node.wz,
        'planner_speed': trot.speed, 'planner_turn': trot.turn,
        'stopping': node.stopping, 'joint_positions_rad': node.q,
        'last_command_rad': node.last_target,
        'tracking_error_rad': getattr(tracking, 'maximum', None),
        'imu_stamp': node.imu_stamp,
    }


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, deque)):
        return [finite_json(v) for v in value]
    return value


class TrotTrace:
    def __init__(self, capacity=300):
        self.rows = deque(maxlen=capacity)
        self.maximum_support_offset_mm = {leg: 0. for leg in LEGS}
        self.maximum_reference_error_m = 0.
        self.maximum_stance_applied_m = 0.

    def append(self, node):
        row = snapshot(node)
        if row is not None:
            for leg, offset in row['support_offset_mm'].items():
                self.maximum_support_offset_mm[leg] = max(self.maximum_support_offset_mm[leg], offset)
            self.maximum_reference_error_m = max(self.maximum_reference_error_m, row['reference_error_m'])
            self.maximum_stance_applied_m = max(self.maximum_stance_applied_m,
                                                row['stance_applied_xy_m'] or 0.)
            # Deep copy: planner containers are mutated after this callback.
            self.rows.append(json.loads(json.dumps(finite_json(row), allow_nan=False)))

    def save_fault(self, directory, reason):
        return self.save(directory, reason, fault=True)

    def save(self, directory, reason, fault=False):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / (('trot_fault_' if fault else 'trot_run_') + str(time.time_ns()) + '.json')
        data = {'reason': reason, 'slip_limit_mm': 20.0,
                'fault': fault, 'maximum_support_offset_mm': self.maximum_support_offset_mm,
                'maximum_reference_error_m': self.maximum_reference_error_m,
                'maximum_stance_applied_m': self.maximum_stance_applied_m,
                'note': 'FK foot-centre offsets; no independent contact-force/slip measurement',
                'samples': list(self.rows)}
        path.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False), encoding='utf-8')
        return path

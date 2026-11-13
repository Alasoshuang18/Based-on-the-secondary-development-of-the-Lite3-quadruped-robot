"""Geometry for flat-floor reactive avoidance, with an inflated robot footprint."""
import math
import random

import numpy as np


def quaternion_matrix(quaternion):
    x, y, z, w = map(float, quaternion)
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid orientation quaternion')
    x, y, z, w = (v/norm for v in (x, y, z, w))
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def yaw_of(quaternion):
    matrix = quaternion_matrix(quaternion)
    return math.atan2(matrix[1, 0], matrix[0, 0])


def xyz_from_cloud(message):
    """Decode XYZ with field offsets, byte order and row padding respected."""
    if message.width <= 0 or message.height <= 0:
        return np.empty((0, 3))
    if message.width*message.height > 100000:
        raise ValueError('Input cloud exceeds the current sensor budget')
    fields = {field.name: field for field in message.fields}
    formats, offsets = [], []
    endian = '>' if message.is_bigendian else '<'
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.datatype not in (7, 8) or field.count != 1:
            raise ValueError('Cloud needs scalar float XYZ fields')
        size = 4 if field.datatype == 7 else 8
        if field.offset < 0 or field.offset+size > message.point_step:
            raise ValueError('Invalid point field bounds')
        formats.append(endian+('f4' if size == 4 else 'f8'))
        offsets.append(field.offset)
    if message.row_step < message.point_step*message.width or len(message.data) < message.row_step*message.height:
        raise ValueError('Invalid cloud buffer or row stride')
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=formats, offsets=offsets,
                          itemsize=message.point_step))
    view = np.ndarray((message.height, message.width), dtype=dtype,
                      buffer=message.data, strides=(message.row_step, message.point_step))
    xyz = np.column_stack([view[name].reshape(-1) for name in ('x', 'y', 'z')])
    return xyz[np.isfinite(xyz).all(axis=1)]


def gravity_aligned(points, sensor_rotation, sensor_translation, imu_quaternion):
    """Sensor -> TORSO -> gravity-levelled frame preserving the current heading."""
    torso = points@quaternion_matrix(sensor_rotation).T+np.asarray(sensor_translation)
    rotation = quaternion_matrix(imu_quaternion)
    yaw = yaw_of(imu_quaternion)
    c, s = math.cos(yaw), math.sin(yaw)
    inverse_heading = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    return torso@(inverse_heading@rotation).T


class SafetyScan:
    """Conservative capsule checks, not a global planner or a terrain classifier.

    Input is expressed at TORSO in gravity-levelled heading axes. Ground is
    identified by the verified flat world's nominal 0.35 m body height.
    Unknown angular coverage is rejected rather than interpreted as free.
    """
    def __init__(self, points, radius=.70, lookahead=.60, body_height=.35):
        if not (.5 <= radius <= 1. and .2 <= lookahead <= 2.):
            raise ValueError('Invalid footprint or stopping lookahead')
        self.radius, self.lookahead = radius, lookahead
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        ranges = np.linalg.norm(points[:, :2], axis=1)
        points = points[(ranges >= .15) & (ranges <= 12.)]
        self.visible = np.zeros(72, dtype=bool)
        if len(points):
            angles = np.arctan2(points[:, 1], points[:, 0])
            bins = np.minimum(71, ((angles+math.pi)/(2*math.pi)*72).astype(int))
            self.visible[bins] = True
        heights = points[:, 2]+body_height
        self.obstacles = points[(heights >= .08) & (heights <= 1.6), :2]
        self.clearance = (float(np.linalg.norm(self.obstacles, axis=1).min())-radius
                          if len(self.obstacles) else float('inf'))

    def coverage(self, angle):
        centres = (np.arange(72)+.5)/72*2*math.pi-math.pi
        delta = (centres-angle+math.pi)%(2*math.pi)-math.pi
        return float(self.visible[np.abs(delta) <= math.radians(30)].mean())

    def clear_path(self, angle, length=None):
        if self.coverage(angle) < .75:
            return False
        length = self.lookahead if length is None else length
        if not len(self.obstacles):
            return True
        direction = np.array([math.cos(angle), math.sin(angle)])
        projection = np.clip(self.obstacles@direction, 0., length)
        distances = np.linalg.norm(self.obstacles-projection[:, None]*direction, axis=1)
        return bool((distances > self.radius).all())

    def choose_turn(self, rng=None):
        if self.clearance <= .10:
            return None  # A rotation sweeps the entire footprint disk.
        rng = rng or random.SystemRandom()
        candidates = [math.radians(rng.uniform(70., 120.))*sign
                      for sign in (-1, 1) for _ in range(8)]
        rng.shuffle(candidates)
        return next((angle for angle in candidates if self.clear_path(angle)), None)

import math
from pathlib import Path
import random
import struct
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation3d_core import SafetyScan, gravity_aligned, xyz_from_cloud


def floor():
    angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    return np.column_stack((4*np.cos(angles), 4*np.sin(angles), np.full(360, -.35)))


class SafetyTests(unittest.TestCase):
    def test_empty_and_unobserved_space_is_not_free(self):
        scan = SafetyScan(np.empty((0, 3)))
        self.assertFalse(scan.clear_path(0))
        self.assertIsNone(scan.choose_turn(random.Random(8)))

    def test_ground_is_observed_but_not_an_obstacle(self):
        scan = SafetyScan(floor())
        self.assertEqual(len(scan.obstacles), 0)
        self.assertTrue(scan.clear_path(0))

    def test_front_wall_blocks_inflated_body_and_random_turn_avoids_it(self):
        wall = np.column_stack((np.full(61, 1.15), np.linspace(-1, 1, 61), np.zeros(61)))
        scan = SafetyScan(np.vstack((floor(), wall)))
        self.assertFalse(scan.clear_path(0))
        for seed in range(20):
            selected = scan.choose_turn(random.Random(seed))
            self.assertIsNotNone(selected)
            self.assertTrue(scan.clear_path(selected))
            self.assertGreaterEqual(abs(selected), math.radians(70))

    def test_narrow_gap_does_not_fit_inflated_footprint(self):
        scan = SafetyScan(np.vstack((floor(), [[.5, .6, .1], [.5, -.6, .1]])))
        self.assertFalse(scan.clear_path(0))

    def test_close_side_obstacle_prevents_rotation(self):
        scan = SafetyScan(np.vstack((floor(), [[0, .75, .1]])))
        self.assertIsNone(scan.choose_turn(random.Random(4)))

    def test_big_endian_padded_cloud_uses_offsets_and_discards_nan(self):
        fields = [NS(name=name, offset=offset, datatype=7, count=1)
                  for name, offset in (('z', 0), ('x', 8), ('y', 12))]
        data = struct.pack('>4f', 3, 99, 1, 2)+b'PADDING!'
        data += struct.pack('>4f', float('nan'), 99, 4, 5)+b'PADDING!'
        message = NS(width=1, height=2, row_step=24, point_step=16,
                     fields=fields, data=data, is_bigendian=True)
        np.testing.assert_allclose(xyz_from_cloud(message), [[1, 2, 3]])

    def test_bad_cloud_buffer_is_rejected(self):
        message = NS(width=1, height=1, row_step=12, point_step=12,
                     fields=[NS(name=n, offset=4*i, datatype=7, count=1)
                             for i, n in enumerate('xyz')], data=b'', is_bigendian=False)
        with self.assertRaises(ValueError):
            xyz_from_cloud(message)

    def test_imu_yaw_does_not_rotate_forward_collision_cone(self):
        result = gravity_aligned(np.array([[1., 0, 0]]), (0, 0, 0, 1),
                                 (.22, 0, .175), (0, 0, math.sin(.7), math.cos(.7)))
        np.testing.assert_allclose(result, [[1.22, 0, .175]], atol=1e-12)


if __name__ == '__main__':
    unittest.main()

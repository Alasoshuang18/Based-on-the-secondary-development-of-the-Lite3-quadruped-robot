#!/usr/bin/python3
"""Export the actual optimized XYZ map and ask RTAB-Map to save its database."""
from datetime import datetime
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty

from navigation3d_core import xyz_from_cloud


def main():
    root = Path(__file__).resolve().parent
    rclpy.init(); node = rclpy.create_node('save_lite3_slam3d')
    received = []
    node.create_subscription(PointCloud2, '/lite3_sim/slam3d/cloud_map', received.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))
    try:
        end = time.monotonic()+20
        while not received and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        if not received:
            raise RuntimeError('No actual 3-D map received; nothing exported')
        message = received[-1]
        xyz = xyz_from_cloud(message)
        if not len(xyz):
            raise RuntimeError('3-D map has no finite XYZ points')
        folder = root/'maps'; folder.mkdir(exist_ok=True)
        path = folder/('cloud_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.pcd')
        header = (f'# Actual RTAB-Map map, frame={message.header.frame_id}\nVERSION .7\n'
                  f'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(xyz)}\n'
                  f'HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n')
        with path.open('wb') as output:
            output.write(header.encode('ascii')); output.write(xyz.astype('<f4').tobytes())
        client = node.create_client(Empty, '/lite3_sim/slam3d/rtabmap/backup')
        backed_up = False
        if client.wait_for_service(timeout_sec=5):
            future = client.call_async(Empty.Request()); end = time.monotonic()+30
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.1)
            backed_up = future.done() and future.exception() is None
        report = dict(pcd=str(path), frame=message.header.frame_id, points=len(xyz),
                      minimum_xyz=xyz.min(axis=0).tolist(), maximum_xyz=xyz.max(axis=0).tolist(),
                      database_backup_completed=backed_up,
                      database_backups=[str(p) for p in folder.glob('*.db.back')])
        (root/'.runtime/slam3d_saved_map.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
        if not backed_up:
            raise RuntimeError('PCD saved, but database backup was not confirmed')
    finally:
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()

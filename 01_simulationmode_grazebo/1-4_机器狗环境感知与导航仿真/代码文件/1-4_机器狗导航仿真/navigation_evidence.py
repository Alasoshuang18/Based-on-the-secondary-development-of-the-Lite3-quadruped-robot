#!/usr/bin/python3
"""Read-only live ROS/X11 evidence window. Captures are real application pixels."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import subprocess
import time

import numpy as np
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication, QHBoxLayout, QLabel, QMainWindow, QPlainTextEdit, QVBoxLayout, QWidget
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rtabmap_msgs.msg import MapGraph, OdomInfo
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from navigation3d_core import xyz_from_cloud

ROOT = Path(__file__).resolve().parent


class Evidence(QMainWindow):
    def __init__(self, auto_capture=False):
        super().__init__()
        self.setWindowTitle('Lite3 Live Navigation Evidence')
        self.resize(1180, 800)
        self.node = rclpy.create_node('lite3_live_evidence')
        self.auto_capture = auto_capture
        self.gait, self.avoid, self.odom, self.info = {}, {}, {}, {}
        self.gait_wall = self.avoid_wall = 0.
        self.validation = {}
        self.map_stats, self.graph = {}, {}
        self.cloud_count, self.odom_count, self.map_count = 0, 0, 0
        self.section = '9.2  Live 6-DoF lidar SLAM / runtime verification'
        self.previous_state = None
        self.last_render = 0.
        self.captured = set()
        self.header = QLabel(self.section)
        self.header.setFont(QFont('DejaVu Sans', 17, QFont.Bold))
        self.left, self.right = QPlainTextEdit(), QPlainTextEdit()
        for box in (self.left, self.right):
            box.setReadOnly(True); box.setFont(QFont('DejaVu Sans Mono', 10))
        self.footer = QLabel('LIVE ROS2 data | read-only | no cmd_vel or joint-command publisher')
        body = QWidget(); layout = QVBoxLayout(body); columns = QHBoxLayout()
        layout.addWidget(self.header); columns.addWidget(self.left); columns.addWidget(self.right)
        layout.addLayout(columns); layout.addWidget(self.footer); self.setCentralWidget(body)
        self.node.create_subscription(String, '/lite3_sim/gait_status', self.on_gait, 1)
        self.node.create_subscription(String, '/lite3_sim/avoidance/status', self.on_avoid, 1)
        self.node.create_subscription(String, '/lite3_sim/evidence/section', self.on_section, 1)
        self.node.create_subscription(String, '/lite3_sim/avoidance/validation', self.on_validation, 1)
        self.node.create_subscription(Odometry, '/lite3_sim/slam3d/odom', self.on_odom, qos_profile_sensor_data)
        self.node.create_subscription(OdomInfo, '/lite3_sim/slam3d/odom_info', self.on_info, qos_profile_sensor_data)
        self.node.create_subscription(PointCloud2, '/lite3_sim/mid360/points', self.on_cloud, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(PointCloud2, '/lite3_sim/slam3d/cloud_map', self.on_map, latched)
        self.node.create_subscription(MapGraph, '/lite3_sim/slam3d/mapGraph', self.on_graph, latched)
        self.timer = QTimer(self); self.timer.timeout.connect(self.poll); self.timer.start(30)

    def on_gait(self, message):
        self.gait_wall = time.monotonic()
        parts = message.data.split(';')
        self.gait = dict(state=parts[0].strip(), **dict(
            item.strip().split('=', 1) for item in parts[1:] if '=' in item))

    def on_section(self, message):
        self.section = message.data[:160]; self.header.setText(self.section)

    def on_validation(self, message):
        self.validation = json.loads(message.data)

    def on_avoid(self, message):
        self.avoid_wall = time.monotonic()
        self.avoid = json.loads(message.data)
        state = self.avoid.get('state')
        if self.auto_capture and state != self.previous_state:
            completed = self.avoid.get('completed', 0)
            key = ('10.4_resume' if state == 'FORWARD' and completed else
                   '10.1_forward' if state == 'FORWARD' else
                   '10.3_turn' if state == 'TURN' else
                   '10.2_brake' if state == 'BRAKE' and not completed else
                   '10.5_stopped' if state == 'IDLE' and completed else None)
            if key and key not in self.captured:
                self.captured.add(key)
                QTimer.singleShot(250, lambda label=key: self.capture(label))
        self.previous_state = state

    def on_odom(self, message):
        self.odom_count += 1
        p, q = message.pose.pose.position, message.pose.pose.orientation
        self.odom = dict(stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                         frame=message.header.frame_id, xyz=[p.x, p.y, p.z],
                         quaternion=[q.x, q.y, q.z, q.w])

    def on_info(self, message):
        self.info = dict(lost=message.lost, inliers_ratio=message.icp_inliers_ratio,
                         processing_seconds=message.time_estimation)

    def on_cloud(self, message):
        self.cloud_count += 1

    def on_map(self, message):
        self.map_count += 1
        xyz = xyz_from_cloud(message)
        self.map_stats = dict(frame=message.header.frame_id, points=len(xyz),
                              minimum_xyz=np.round(xyz.min(axis=0), 3).tolist() if len(xyz) else [],
                              maximum_xyz=np.round(xyz.max(axis=0), 3).tolist() if len(xyz) else [])

    def on_graph(self, message):
        self.graph = dict(nodes=len(message.poses_id), links=len(message.links))

    def render(self):
        package_path = ROOT/'.runtime/slam3d_packages.json'
        package = json.loads(package_path.read_text()) if package_path.exists() else {}
        memory = dict(line.split(':', 1) for line in Path('/proc/meminfo').read_text().splitlines())
        gait, avoid, validation = self.gait, self.avoid, self.validation
        gait_source = 'LIVE / age %.1f wall s' % (time.monotonic()-self.gait_wall) if gait else 'No live gait publisher'
        avoid_source = 'LIVE / age %.1f wall s' % (time.monotonic()-self.avoid_wall) if avoid else 'No live avoidance publisher'
        result_path = ROOT/'.runtime/diagnostics/navigation3d_acceptance.json'
        if result_path.exists():
            result = json.loads(result_path.read_text())
            if not gait:
                gait = result.get('final_gait', {})
                gait_source = 'RECORDED ACCEPTANCE / E stopped'
            if not avoid:
                avoid = result.get('end_avoidance', {})
                avoid_source = 'RECORDED ACCEPTANCE / F stopped'
            if not validation:
                validation = result
        left = [datetime.now().astimezone().strftime('%Y-%m-%d %H:%M:%S %Z'),
                '\n3-D lidar-only SLAM (no Gazebo pose input)',
                'RTAB-Map ICP 6-DoF -> pose graph -> cloud_map',
                '\nPRIVATE DEPLOYMENT',
                f"Download: {package.get('download_bytes', 0)/2**20:.2f} MiB",
                f"Extracted: {package.get('installed_bytes', 0)/2**20:.2f} MiB",
                f"Available RAM: {int(memory['MemAvailable'].split()[0])/2**20:.2f} GiB",
                '\nLIVE TOPIC COUNTS',
                f'Input clouds / ICP odometry / 3-D maps: {self.cloud_count} / {self.odom_count} / {self.map_count}',
                '\nACTUAL 3-D MAP', json.dumps(self.map_stats),
                '\nGRAPH', json.dumps(self.graph),
                '\nICP TRACKING', json.dumps(self.info, indent=2),
                '\nACTUAL LIDAR ODOMETRY', json.dumps(self.odom)]
        keys = ('state', 'mode', 'lift_test', 'lift_passed', 'trot_half_steps', 'roll',
                'pitch', 'tracking', 'sync_age_ms', 'reason')
        right = ['EXISTING PAYLOAD GAIT / '+gait_source,
                 f"{gait.get('state')} / {gait.get('mode')}; lift={gait.get('lift_test')}; passed={gait.get('lift_passed')}",
                 f"roll={gait.get('roll')}; pitch={gait.get('pitch')}; half_steps={gait.get('trot_half_steps')}",
                 'Reason: '+str(gait.get('reason')),
                 '\nREACTIVE AVOIDANCE / '+avoid_source, json.dumps({k:avoid.get(k) for k in (
                     'state','enabled','reason','completed','target_turn_deg','turn_progress_deg',
                     'clearance_m','command_v','command_w','odom_lost')}, indent=2),
                 '\nINDEPENDENT PHYSICS VALIDATION (NOT CONTROL INPUT)',
                 json.dumps({k:validation.get(k) for k in (
                     'robot_obstacle_contacts','positive_probe_contacts',
                     'min_inflated_body_clearance_m','max_abs_roll_pitch_deg',
                     'external_validation_stop')}, indent=2),
                 '\nRECENT ACTUAL EVENTS']
        for event in avoid.get('recent_events', [])[-4:]:
            right.append(f"{event.get('sim', 0):.2f} {event.get('state')} {event.get('reason')}")
        saved = ROOT/'.runtime/slam3d_saved_map.json'
        if saved.exists():
            report = json.loads(saved.read_text())
            right += ['\nSAVED OUTPUT', Path(report['pcd']).name,
                      f"Points: {report['points']}; DB saved: {report['database_backup_completed']}"]
        self.left.setPlainText('\n'.join(left)); self.right.setPlainText('\n'.join(right))

    def poll(self):
        if not rclpy.ok():
            QApplication.quit()
            return
        for _ in range(15):
            rclpy.spin_once(self.node, timeout_sec=0.)
        if time.monotonic()-self.last_render > .5:
            self.last_render = time.monotonic(); self.render()

    def capture(self, section):
        self.render(); self.raise_(); self.activateWindow(); QApplication.processEvents()
        folder = ROOT/'images/slam3d'; folder.mkdir(parents=True, exist_ok=True)
        path = folder/(section+'_evidence.png')
        image = self.screen().grabWindow(int(self.winId()))
        if image.isNull() or not image.save(str(path), 'PNG'):
            raise RuntimeError('Live evidence window capture failed')
        metadata = dict(section=section, file=path.name, title=self.windowTitle(),
                        utc=datetime.now(timezone.utc).isoformat(),
                        ros_state=self.avoid.get('state'), sim=self.avoid.get('sim'),
                        sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        with (folder/'capture_manifest.jsonl').open('a') as output:
            output.write(json.dumps(metadata)+'\n')
        # Capture only observed, uniquely named Gazebo/RViz X11 application windows.
        for title, suffix in (('Gazebo', 'gazebo'),):
            try:
                subprocess.Popen(['/usr/bin/python3', str(ROOT/'capture_x11.py'),
                                  '--title', title, '--section', section+'_'+suffix],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except OSError:
                pass
        print('CAPTURE', json.dumps(metadata), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--auto-capture', action='store_true')
    args = parser.parse_args()
    app = QApplication([]); rclpy.init(); window = Evidence(args.auto_capture); window.show()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    try:
        app.exec_()
    finally:
        window.node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()

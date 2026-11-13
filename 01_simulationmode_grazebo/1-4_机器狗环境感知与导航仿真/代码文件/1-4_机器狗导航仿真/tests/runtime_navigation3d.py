#!/usr/bin/python3
"""Independent simulator acceptance; Gazebo poses are used only here, not by SLAM/avoidance."""
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from rtabmap_msgs.msg import MapGraph
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from navigation3d_core import xyz_from_cloud


class Recorder:
    def __init__(self, contact_probe=False):
        self.node = rclpy.create_node('navigation3d_independent_acceptance')
        self.avoid, self.gait, self.pose = {}, {}, {}
        self.poses = 0
        self.min_clearance = float('inf')
        self.max_angles = [0., 0.]
        self.contact_frames = {'front': 0, 'left': 0}
        self.robot_contacts = 0
        self.probe_contacts = 0
        self.probe_by_side = {'front': 0, 'left': 0}
        self.map_points = self.graph_nodes = 0
        self.start = None
        self.external_stop = None
        self.started = False
        self.last_state = None
        self.last_progress = 0.
        self.pub = self.node.create_publisher(String, '/lite3_sim/avoidance/validation', 1)
        self.client = self.node.create_client(Trigger, '/lite3_sim/avoidance/stop')
        self.node.create_subscription(String, '/lite3_sim/avoidance/status', self.on_avoid, 10)
        self.node.create_subscription(String, '/lite3_sim/gait_status', self.on_gait, 10)
        self.node.create_subscription(TFMessage, '/lite3_sim/model_pose', self.on_pose, qos_profile_sensor_data)
        contact_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        for side in self.contact_frames:
            self.node.create_subscription(Contacts, '/lite3_sim/obstacle_contacts/'+side,
                                          lambda msg, side=side: self.on_contacts(side, msg), contact_qos)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(PointCloud2, '/lite3_sim/slam3d/cloud_map', self.on_map, latched)
        self.node.create_subscription(MapGraph, '/lite3_sim/slam3d/mapGraph', self.on_graph, latched)
        self.folder = ROOT/'.runtime/diagnostics'; self.folder.mkdir(parents=True, exist_ok=True)
        self.trace = (self.folder/'navigation3d_acceptance_trace.jsonl').open('w')
        self.contact_probe = contact_probe

    def on_avoid(self, msg):
        self.avoid = json.loads(msg.data)
        if self.avoid.get('enabled') and not self.started:
            self.started = True
            self.start = dict(self.pose)
        if self.avoid.get('state') != self.last_state:
            print('STATE', self.avoid.get('state'), self.avoid.get('reason'), flush=True)
            self.last_state = self.avoid.get('state')

    def on_gait(self, msg):
        parts = msg.data.split(';')
        self.gait = {'state': parts[0].strip(), **dict(p.strip().split('=', 1)
                     for p in parts[1:] if '=' in p)}

    def on_pose(self, msg):
        for t in msg.transforms:
            if t.child_frame_id != 'lite3_control':
                continue
            p, q = t.transform.translation, t.transform.rotation
            roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
            pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
            self.pose = dict(sim=t.header.stamp.sec+t.header.stamp.nanosec*1e-9,
                             x=p.x, y=p.y, z=p.z, roll=roll, pitch=pitch,
                             yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            if self.started:
                distances = []
                for cx, cy, hx, hy in ((3., 0., .15, 1.), (0., 3., 1., .15)):
                    distances.append(math.hypot(max(abs(p.x-cx)-hx, 0.),
                                                 max(abs(p.y-cy)-hy, 0.))-.70)
                self.min_clearance = min(self.min_clearance, *distances)
                self.max_angles = [max(self.max_angles[0], abs(math.degrees(roll))),
                                   max(self.max_angles[1], abs(math.degrees(pitch)))]
                self.poses += 1
                self.trace.write(json.dumps(dict(kind='pose', **self.pose))+'\n')

    def on_contacts(self, side, msg):
        self.contact_frames[side] += 1
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if any('lite3_contact_probe_' in n for n in names):
                self.probe_contacts += 1
                self.probe_by_side[side] += 1
            if any('lite3_control' in n for n in names):
                self.robot_contacts += 1
                self.trace.write(json.dumps(dict(kind='robot_contact', names=names))+'\n')

    def on_map(self, msg):
        points = xyz_from_cloud(msg)
        self.map_points = len(points)

    def on_graph(self, msg):
        self.graph_nodes = len([i for i in msg.poses_id if i > 0])

    def spin(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.02)

    def report(self):
        return dict(started=self.started, pose=self.pose,
                    avoidance_state=self.avoid.get('state'),
                    completed=self.avoid.get('completed', 0), gait_state=self.gait.get('state'),
                    robot_obstacle_contacts=self.robot_contacts,
                    obstacle_contact_frames=self.contact_frames,
                    positive_probe_contacts=self.probe_contacts,
                    min_inflated_body_clearance_m=self.min_clearance if self.started else None,
                    max_abs_roll_pitch_deg=self.max_angles,
                    map_points=self.map_points, graph_nodes=self.graph_nodes,
                    external_validation_stop=self.external_stop)

    def safety_stop(self, reason):
        if self.external_stop:
            return
        self.external_stop = reason
        print('INDEPENDENT VALIDATOR STOP (test fails):', reason, flush=True)
        if self.client.service_is_ready():
            self.client.call_async(Trigger.Request())

    def check_contact_instrumentation(self):
        self.spin(2)
        self.check_one_contact('front', '2.84 0 .55')
        self.check_one_contact('left', '0 2.84 .55')
        report = dict(positive_probe_contacts=self.probe_by_side,
                      sensor_frames=self.contact_frames, robot_contacts=self.robot_contacts)
        (self.folder/'obstacle_contact_instrumentation.json').write_text(json.dumps(report, indent=2))
        if not all(self.probe_by_side.values()):
            raise RuntimeError('Both contact channels must pass the positive control')
        print('CONTACT_INSTRUMENTATION_VERIFIED', report, flush=True)

    def check_one_contact(self, side, location):
        name = 'lite3_contact_probe_'+str(int(time.time()))
        sdf = ('<sdf version="1.8"><model name="'+name+'"><pose>'+location+' 0 0 0</pose>'
               '<link name="probe"><inertial><mass>.01</mass><inertia><ixx>.00001</ixx>'
               '<iyy>.00001</iyy><izz>.00001</izz></inertia></inertial><collision name="probe_collision">'
               '<geometry><sphere><radius>.05</radius></sphere></geometry></collision></link></model></sdf>')
        env = dict(os.environ); env.pop('IGN_IP', None)
        create = ['ign', 'service', '-s', '/world/lite3_world/create', '--reqtype',
                  'ignition.msgs.EntityFactory', '--reptype', 'ignition.msgs.Boolean',
                  '--timeout', '5000', '--req', 'sdf: '+json.dumps(sdf)]
        reply = subprocess.run(create, env=env, capture_output=True, text=True, timeout=10)
        print('CONTACT_PROBE_CREATE', reply.stdout.strip(), flush=True)
        try:
            self.spin(5)
        finally:
            remove = ['ign', 'service', '-s', '/world/lite3_world/remove', '--reqtype',
                      'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean', '--timeout',
                      '5000', '--req', 'name: '+json.dumps(name)+' type: MODEL']
            reply = subprocess.run(remove, env=env, capture_output=True, text=True, timeout=10)
            print('CONTACT_PROBE_REMOVE', reply.stdout.strip(), flush=True)
            self.spin(1)
        if not self.probe_by_side[side] or 'true' not in reply.stdout:
            raise RuntimeError('Contact monitoring not verified or probe not removed: '+side)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contact-probe', action='store_true')
    parser.add_argument('--wall-limit', type=float, default=2400.)
    args = parser.parse_args()
    rclpy.init(); recorder = Recorder(args.contact_probe)
    began = time.monotonic()
    try:
        if args.contact_probe:
            recorder.check_contact_instrumentation()
        while time.monotonic()-began < args.wall_limit:
            recorder.spin(.2)
            if recorder.started:
                if recorder.robot_contacts:
                    recorder.safety_stop('Actual robot/obstacle contact detected')
                elif recorder.min_clearance < .10:
                    recorder.safety_stop('Inflated body within 0.10 m of an obstacle')
                elif max(recorder.max_angles) > 7:
                    recorder.safety_stop('Measured attitude exceeds 7 degrees')
                if recorder.gait.get('state') == 'FAULT' or recorder.avoid.get('state') in ('HALTED', 'BLOCKED'):
                    break
                if recorder.avoid.get('state') == 'IDLE' and recorder.avoid.get('completed', 0) >= 1:
                    break
            if time.monotonic()-recorder.last_progress > 10:
                recorder.last_progress = time.monotonic()
                report = recorder.report()
                msg = String(); msg.data = json.dumps(report); recorder.pub.publish(msg)
                (recorder.folder/'navigation3d_acceptance_live.json').write_text(json.dumps(report, indent=2))
                print('PROGRESS', json.dumps(report), flush=True)
        else:
            recorder.safety_stop('Acceptance wall-time budget exceeded')
        result = recorder.report()
        result['start'] = recorder.start
        result['end_avoidance'] = recorder.avoid
        result['final_gait'] = recorder.gait
        result['wall_seconds'] = time.monotonic()-began
        result['passed'] = bool(result['started'] and result['completed'] >= 1 and
                                result['avoidance_state'] == 'IDLE' and result['gait_state'] == 'READY' and
                                not recorder.external_stop and not recorder.robot_contacts and
                                recorder.min_clearance > .15 and max(recorder.max_angles) < 5 and
                                recorder.map_points > 0 and recorder.graph_nodes > 1)
        (recorder.folder/'navigation3d_acceptance.json').write_text(json.dumps(result, indent=2))
        print('RESULT', json.dumps(result), flush=True)
    except BaseException as error:
        recorder.safety_stop(str(error))
        result = recorder.report(); result.update(passed=False, error=str(error))
        (recorder.folder/'navigation3d_acceptance.json').write_text(json.dumps(result, indent=2))
        raise
    finally:
        recorder.trace.close(); recorder.node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()

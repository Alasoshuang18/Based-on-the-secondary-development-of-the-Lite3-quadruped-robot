#!/usr/bin/env python3
"""Bounded physical acceptance, using simulator poses, not command integration.

Run only in the 6-4 simulation with one payload controller and no other cmd_vel
publisher. Each run writes its actual result under 6-4/.runtime/diagnostics.
No controller settings, model files, or physics parameters are changed.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


ROOT = Path(__file__).resolve().parents[1]


class Acceptance:
    def __init__(self):
        self.node = rclpy.create_node('payload_endurance_acceptance')
        self.status = {}
        self.pose = None
        self.pose_wall = self.status_wall = 0.
        self.unwrapped_yaw = 0.
        self.last_yaw = None
        self.last_stamp = None
        self.rows = []
        self.capture = False
        self.node.create_subscription(String, '/lite3_sim/gait_status', self.state, 10)
        self.node.create_subscription(TFMessage, '/lite3_sim/model_pose', self.position,
                                      qos_profile_sensor_data)
        self.pub = self.node.create_publisher(Twist, '/lite3_sim/cmd_vel', 1)

    def state(self, message):
        parts = message.data.split(';')
        self.status = {'state': parts[0].strip(), **dict(
            item.strip().split('=', 1) for item in parts[1:] if '=' in item)}
        self.status_wall = time.monotonic()
        if self.capture:
            self.rows.append({'kind': 'status', 'wall': self.status_wall,
                              'pose_stamp': self.last_stamp, 'data': self.status})

    def position(self, message):
        for item in message.transforms:
            if item.child_frame_id != 'lite3_control':
                continue
            p, q = item.transform.translation, item.transform.rotation
            stamp = item.header.stamp.sec+1e-9*item.header.stamp.nanosec
            if self.last_stamp is not None and stamp <= self.last_stamp:
                continue
            roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
            pitch = math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x))))
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            if self.last_yaw is not None:
                self.unwrapped_yaw += (yaw-self.last_yaw+math.pi)%(2*math.pi)-math.pi
            else:
                self.unwrapped_yaw = yaw
            self.last_yaw, self.last_stamp = yaw, stamp
            self.pose_wall = time.monotonic()
            self.pose = dict(stamp=stamp, x=p.x, y=p.y, z=p.z, roll=roll,
                             pitch=pitch, yaw=yaw, unwrapped_yaw=self.unwrapped_yaw)
            if self.capture:
                self.rows.append({'kind': 'pose', 'wall': self.pose_wall, **self.pose})

    def spin(self, duration=.05):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.01)

    def velocity(self, x=0., w=0.):
        message = Twist()
        message.linear.x, message.angular.z = x, w
        self.pub.publish(message)

    def wait(self, predicate, timeout, label, allow_fault=False):
        end = time.monotonic()+timeout
        last_print = 0.
        while time.monotonic() < end:
            self.spin()
            if not allow_fault and self.status.get('state') == 'FAULT':
                raise RuntimeError('FAULT: '+str(self.status))
            if predicate():
                return
            if time.monotonic()-last_print >= 20:
                print(label, self.status.get('state'), self.status.get('phase'),
                      self.status.get('leg'), flush=True)
                last_print = time.monotonic()
        raise RuntimeError(label+' timed out: '+str(self.status))

    def service(self, name, require_success=True):
        client = self.node.create_client(Trigger, '/lite3_sim/'+name)
        try:
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError('Service unavailable: '+name)
            future = client.call_async(Trigger.Request())
            self.wait(future.done, 20, name, allow_fault=True)
            result = future.result()
            print(name, result.success, result.message, flush=True)
            if require_success and not result.success:
                raise RuntimeError(name+': '+result.message)
            return result.success
        finally:
            self.node.destroy_client(client)

    def prepare(self):
        self.wait(lambda: bool(self.pose and self.status), 20, 'discover', True)
        if self.status.get('mode') != 'trot' or self.status.get('pin_body') != 'False':
            raise RuntimeError('Requires free-body mode=trot')
        if self.node.count_publishers('/lite3_sim/cmd_vel') != 1:
            raise RuntimeError('Another cmd_vel publisher exists; no motion started')
        if self.node.count_publishers('/lite3_sim/lite3_leg_controller/joint_trajectory') != 1:
            raise RuntimeError('Requires exactly one joint-command publisher')
        if self.status.get('state') == 'DISARMED':
            self.service('enable_gait')
            self.wait(lambda: self.status.get('state') == 'READY', 80, 'standing')
        if self.status.get('state') != 'READY':
            raise RuntimeError('Controller must be READY or DISARMED')
        if self.status.get('lift_passed') != 'True':
            self.service('test_foot_lift')
            self.wait(lambda: self.status.get('state') == 'READY' and
                      self.status.get('lift_passed') == 'True', 420, 'four-foot qualification')

    def stop(self):
        self.velocity()
        self.service('stop_gait', require_success=False)
        self.wait(lambda: self.status.get('state') in ('READY', 'FAULT'),
                  80, 'touchdown/stop', allow_fault=True)

    def run(self, case, seconds, speed, turn):
        self.rows = []
        self.capture = True
        result = {'case': case, 'requested_sim_seconds': seconds if case in (
            'forward', 'backward') else None, 'requested_turn_deg': 90 if case in (
            'left90', 'right90') else None, 'speed_m_s': speed, 'turn_rad_s': turn}
        x, w = {'forward': (speed, 0.), 'backward': (-speed, 0.),
                'left90': (0., turn), 'right90': (0., -turn)}[case]
        start = None
        wall_start = time.monotonic()
        last_print = 0.
        try:
            self.velocity()
            self.spin(.5)
            while time.monotonic()-wall_start < 3600:
                self.velocity(x, w)
                self.spin(.05)
                if self.status.get('state') == 'FAULT':
                    raise RuntimeError(self.status.get('reason', str(self.status)))
                if time.monotonic()-self.pose_wall > 5 or time.monotonic()-self.status_wall > 5:
                    raise RuntimeError('Wall-time feedback watchdog exceeded 5 seconds')
                if self.status.get('state') == 'WALKING' and start is None:
                    # Exclude the initial four-support recenter from walking drift.
                    start = dict(self.pose)
                    result['start'] = start
                if start:
                    elapsed = self.pose['stamp']-start['stamp']
                    angle = math.degrees(self.pose['unwrapped_yaw']-start['unwrapped_yaw'])
                    if time.monotonic()-last_print >= 20:
                        print('PROGRESS', case, 'sim_s', round(elapsed, 2), 'yaw_deg',
                              round(angle, 2), 'half_steps', self.status.get('trot_half_steps'),
                              'z', round(self.pose['z'], 4), flush=True)
                        last_print = time.monotonic()
                    if ((w == 0. and elapsed >= seconds) or
                            (w != 0. and angle*math.copysign(1, w) >= 90)):
                        result['target_reached'] = True
                        break
                    if w and elapsed > 700:
                        raise RuntimeError('90-degree target not reached within 700 simulation seconds')
            else:
                raise RuntimeError('Case exceeded one-hour wall-time budget')
        except BaseException as error:
            result['error'] = str(error)
        finally:
            try:
                self.stop()
            except BaseException as error:
                result['stop_error'] = str(error)
            self.capture = False
            result['end'] = dict(self.pose) if self.pose else None
            result['final_status'] = dict(self.status)
            result['wall_seconds'] = time.monotonic()-wall_start
            poses = [r for r in self.rows if r['kind'] == 'pose' and start and
                     r['stamp'] >= start['stamp']]
            if poses:
                result['max_abs_roll_pitch_deg'] = [max(abs(math.degrees(r[k])) for r in poses)
                                                   for k in ('roll', 'pitch')]
                result['z_range_m'] = [min(r['z'] for r in poses), max(r['z'] for r in poses)]
                end = result['end']
                dx, dy = end['x']-start['x'], end['y']-start['y']
                result['forward_lateral_m'] = [dx*math.cos(start['yaw'])+dy*math.sin(start['yaw']),
                                               -dx*math.sin(start['yaw'])+dy*math.cos(start['yaw'])]
                result['net_yaw_deg'] = math.degrees(end['unwrapped_yaw']-start['unwrapped_yaw'])
                result['active_sim_seconds'] = end['stamp']-start['stamp']
            result['passed'] = bool(result.get('target_reached') and not result.get('error') and
                                    not result.get('stop_error') and self.status.get('state') == 'READY')
            if result['passed']:
                result['passed'] &= max(result.get('max_abs_roll_pitch_deg', [999])) <= 5
                if w:
                    result['passed'] &= abs(abs(result['net_yaw_deg'])-90) <= 3
                else:
                    result['passed'] &= result['forward_lateral_m'][0]*math.copysign(1, x) > 0
            folder = ROOT/'.runtime/diagnostics'
            folder.mkdir(parents=True, exist_ok=True)
            (folder/f'endurance_{case}.json').write_text(json.dumps(result, indent=2))
            with (folder/f'endurance_{case}_trace.jsonl').open('w') as output:
                for row in self.rows:
                    output.write(json.dumps(row)+'\n')
            print('RESULT', json.dumps(result), flush=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['all', 'forward', 'backward', 'left90', 'right90'], default='all')
    parser.add_argument('--seconds', type=float, default=120.)
    parser.add_argument('--speed', type=float, default=.002)
    parser.add_argument('--turn', type=float, default=.01)
    args = parser.parse_args()
    if not (0 < args.seconds <= 600 and 0 < args.speed <= .020 and 0 < args.turn <= .020):
        parser.error('Requested motion outside the existing test envelope')
    rclpy.init()
    test = Acceptance()
    def interrupted(number, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGINT, interrupted)
    try:
        test.prepare()
        cases = ['forward', 'backward', 'left90', 'right90'] if args.case == 'all' else [args.case]
        for case in cases:
            if not test.run(case, args.seconds, args.speed, args.turn)['passed']:
                return 1  # Preserve a failure; do not automatically re-enable.
        return 0
    finally:
        test.node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

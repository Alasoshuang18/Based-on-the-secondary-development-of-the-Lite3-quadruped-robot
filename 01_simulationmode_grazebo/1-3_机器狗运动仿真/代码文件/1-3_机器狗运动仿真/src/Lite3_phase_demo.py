#!/usr/bin/env python3
"""Fixed-body Lite3 lift/hold/lower test using complete trajectory ACTION goals.

No model/config/environment changes. Does not prove free-body walking.
"""
import argparse
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET


def smooth(u):
    return u*u*u*(10+u*(-15+6*u))


def lift_points(core, legs, height):
    bases = [core.forward(i, core.STAND[3*i:3*i+3]) for i in range(4)]
    points = []
    for n in range(101):
        t = n*.05
        # 2 seconds up, 1 second hold, 2 seconds down.
        fraction = smooth(t/2) if t <= 2 else 1.0 if t <= 3 else 1-smooth((t-3)/2)
        q = list(core.STAND)
        for i in legs:
            x, y, z = bases[i]
            q[3*i:3*i+3] = core.inverse(i, (x, y, z+height*fraction))
        points.append((t, q))
    return points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gait', choices=['walk', 'trot'], default='walk')
    parser.add_argument('--height', type=float, default=.030)
    parser.add_argument('--cycles', type=int, default=1)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--core-dir', default=str(Path.home()/'catkin_ws_lite3dog/src/06_simulationmode_grazebo/6-3_robot_sport/src'))
    args = parser.parse_args()
    if not math.isfinite(args.height) or not .010 <= args.height <= .030 or not 1 <= args.cycles <= 3:
        parser.error('height 0.010..0.030 m; cycles 1..3')
    sys.path.insert(0, str(Path(args.core_dir).resolve()))
    import lite3_gait_core as core
    groups = [(2,), (0,), (3,), (1,)] if args.gait == 'walk' else [(0, 3), (1, 2)]
    sequences = [(legs, lift_points(core, legs, args.height)) for legs in groups]
    bases = [core.forward(i, core.STAND[3*i:3*i+3]) for i in range(4)]
    if args.self_test:
        for legs, points in sequences:
            previous_time = -1
            previous_q = None
            for t, q in points:
                assert t > previous_time
                fraction = smooth(t/2) if t <= 2 else 1 if t <= 3 else 1-smooth((t-3)/2)
                for i in range(4):
                    expected = args.height*fraction if i in legs else 0
                    assert abs(core.forward(i, q[3*i:3*i+3])[2]-bases[i][2]-expected) < 1e-8
                if previous_q is not None:
                    assert max(abs(a-b) for a,b in zip(q,previous_q))/(t-previous_time) < .5
                previous_time, previous_q = t, q
            assert max(abs(a-b) for a,b in zip(points[-1][1],core.STAND)) < 1e-8
        print('PASS: complete 5s lift/hold/lower trajectories; 30mm default peak; unchanged support legs; joint rate <0.5rad/s')
        return

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.srv import GetParameters
    from controller_manager_msgs.srv import ListControllers
    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    rclpy.init(args=[])
    node = rclpy.create_node('lite3_segment_demo', namespace='lite3_sim',
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    data = {'q': None, 'stamp': None, 'wall': 0}
    def joint(message):
        if len(message.name) != len(message.position):
            return
        values = dict(zip(message.name, message.position))
        if all(j in values and math.isfinite(values[j]) for j in core.JOINTS):
            data.update(q=[values[j] for j in core.JOINTS],
                        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                        wall=time.monotonic())
    subscription = node.create_subscription(JointState,
        '/lite3_sim/joint_state_broadcaster/joint_states', joint, qos_profile_sensor_data)
    client = ActionClient(node, FollowJointTrajectory,
                         '/lite3_sim/lite3_leg_controller/follow_joint_trajectory')
    goal_handle = None
    def future_result(future, timeout=15):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('ROS reply timeout')
        return future.result()
    def call(srv, name, request):
        service = node.create_client(srv, name)
        if not service.wait_for_service(timeout_sec=10):
            raise RuntimeError('Missing service: '+name)
        result = future_result(service.call_async(request))
        node.destroy_client(service)
        return result
    def command(points, label, moving_legs=()):
        nonlocal goal_handle
        if node.count_publishers('/lite3_sim/lite3_leg_controller/joint_trajectory'):
            raise RuntimeError('Another topic command publisher is running; stop old demos/E first')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(core.JOINTS)
        for t,q in points:
            point = JointTrajectoryPoint()
            point.positions = list(q)
            ns = round(t*1e9)
            point.time_from_start = Duration(sec=ns//1000000000, nanosec=ns%1000000000)
            goal.trajectory.points.append(point)
        # One complete goal: no repeated replacement of future endpoints.
        goal_handle = future_result(client.send_goal_async(goal))
        if not goal_handle.accepted:
            goal_handle = None
            raise RuntimeError('Controller rejected trajectory: '+label)
        finished = goal_handle.get_result_async()
        deadline = time.monotonic()+120
        previous_sim = node.get_clock().now().nanoseconds/1e9
        clock_changed = time.monotonic()
        last_report = 0
        peaks = [0.0]*4
        print('Accepted '+label+'; waiting for actual feedback and action result', flush=True)
        while not finished.done():
            rclpy.spin_once(node, timeout_sec=.02)
            now = node.get_clock().now().nanoseconds/1e9
            if now < previous_sim:
                raise RuntimeError('Simulation time reset')
            if now != previous_sim:
                previous_sim, clock_changed = now, time.monotonic()
            if time.monotonic() > deadline or time.monotonic()-clock_changed > 10:
                raise RuntimeError('Action timeout or simulation paused for 10 wall seconds')
            if (time.monotonic()-data['wall'] > 2 or data['stamp'] is None or
                    not -.1 <= now-data['stamp'] <= .2):
                raise RuntimeError('Joint feedback stale')
            if node.count_publishers('/lite3_sim/lite3_leg_controller/joint_trajectory'):
                raise RuntimeError('Competing topic command appeared')
            lifts = [1000*(core.forward(i,data['q'][3*i:3*i+3])[2]-bases[i][2]) for i in range(4)]
            peaks = [max(a,b) for a,b in zip(peaks,lifts)]
            if time.monotonic()-last_report > 1:
                last_report = time.monotonic()
                print(f'{label}: actual_relative_z_mm={[round(v,1) for v in lifts]} peak_mm={[round(v,1) for v in peaks]}', flush=True)
        result = future_result(finished)
        goal_handle = None
        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError(f'Action failed: status={result.status} error={result.result.error_code} {result.result.error_string}')
        if max(abs(a-b) for a,b in zip(data['q'],core.STAND)) > .03:
            raise RuntimeError('Action ended but actual joints have not returned to STAND within 0.03rad')
        if moving_legs and any(peaks[i] < args.height*1000*.8 for i in moving_legs):
            raise RuntimeError(f'Actual lift insufficient: peak_mm={peaks}; require 80% of requested height')
        if moving_legs and any(peaks[i] > 8 for i in range(4) if i not in moving_legs):
            raise RuntimeError('Unexpected support-leg motion above 8mm')
        print('PASS '+label, flush=True)
    try:
        response = call(GetParameters, '/lite3_sim/robot_state_publisher/get_parameters', GetParameters.Request(names=['robot_description']))
        root = ET.fromstring(response.values[0].string_value)
        pin = root.find(".//joint[@name='lite3_calibration_world_fixed']")
        if pin is None or pin.get('type') != 'fixed' or pin.findtext('parent') != 'world' or pin.findtext('child') != 'TORSO':
            raise RuntimeError('Requires existing pin_body:=true Lite3 model')
        response = call(ListControllers, '/lite3_sim/controller_manager/list_controllers', ListControllers.Request())
        active = {c.name for c in response.controller if c.state == 'active'}
        if not {'joint_state_broadcaster','lite3_leg_controller'} <= active:
            raise RuntimeError('Both existing controllers must be active: '+str(active))
        if node.count_publishers('/lite3_sim/gait_status'):
            raise RuntimeError('Old E gait node is still running')
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('Existing FollowJointTrajectory action unavailable')
        deadline = time.monotonic()+15
        while data['q'] is None or node.get_clock().now().nanoseconds <= 0:
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic() > deadline:
                raise RuntimeError('No joint feedback or /clock')
        initial = list(data['q'])
        if max(abs(a-b) for a,b in zip(initial,core.STAND)) > .4:
            raise RuntimeError('Initial pose too far from reference; use known single STAND command first')
        stand = [(n*.05,[a+(b-a)*smooth(n/60) for a,b in zip(initial,core.STAND)]) for n in range(61)]
        command(stand, 'STAND')
        for cycle in range(args.cycles):
            for legs,points in sequences:
                command(points, f'cycle={cycle+1} legs='+','.join(core.LEGS[i] for i in legs), legs)
        print('COMPLETE: actual lift tests passed and joints returned to STAND. Fixed-body demo only.', flush=True)
    finally:
        if goal_handle is not None and goal_handle.accepted and rclpy.ok():
            try:
                future_result(goal_handle.cancel_goal_async(), timeout=3)
            except Exception:
                pass
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Canceled; controller may hold a raised foot. Use the known single STAND command to restore.')
    except (RuntimeError, ValueError, ImportError) as error:
        print('STOP: '+str(error), file=sys.stderr)
        sys.exit(1)


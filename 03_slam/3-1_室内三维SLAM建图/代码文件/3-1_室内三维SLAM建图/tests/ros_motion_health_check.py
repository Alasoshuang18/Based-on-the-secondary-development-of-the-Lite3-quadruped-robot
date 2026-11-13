"""Synthetic ROS interface regression in an isolated domain; no robot commands."""
from pathlib import Path
import csv
import json
import math
import os
import struct
import sys
import tempfile
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lite3_lab_4_1.experiment import Experiment


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '87', 'Use isolated domain 87 only'
    root = Path(tempfile.mkdtemp(prefix='health_policy_synthetic_',
                                 dir=Path.home()/'catkin_ws_lite3dog/results'))
    rclpy.init(args=['--ros-args', '-p', f'result_dir:={root}'])
    node = Experiment()
    peer = Node('health_policy_synthetic_inputs')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    pubs = {topic:peer.create_publisher(kind, topic, qos_profile_sensor_data)
            for topic, kind in [('/course/points_raw', PointCloud2), ('/course/imu', Imu),
                                ('/course/odom', Odometry), ('/course/lio_points', PointCloud2)]}
    clients = {name:peer.create_client(Trigger, '/lab_4_1/'+name)
               for name in ('start_trial', 'finish_trial')}
    report = {'synthetic_inputs': True, 'ros_domain': 87, 'robot_motion_commands_sent': False,
              'directory':str(root)}

    def spin(duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.002)

    def call(name):
        assert clients[name].wait_for_service(timeout_sec=3)
        future = clients[name].call_async(Trigger.Request())
        deadline = time.monotonic()+3
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert future.done(), name+' timeout'
        return future.result()

    def cloud(stamp, frame, invalid=False):
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        msg.height, msg.width = 1, 0 if invalid else 1
        msg.fields = [PointField(name=n, offset=i*4, datatype=7, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))]
        msg.point_step, msg.row_step = 12, 0 if invalid else 12
        msg.data = b'' if invalid else struct.pack('<fff', 1., 0., 0.)
        msg.is_dense = True
        return msg

    def publish(norm=9.81, x=0., invalid_cloud=False, invalid_map=False):
        stamp = peer.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'imu_link'
        imu.linear_acceleration.z = float(norm)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.orientation.w = 1.
        pubs['/course/points_raw'].publish(cloud(stamp, 'lidar_link', invalid_cloud))
        pubs['/course/imu'].publish(imu)
        pubs['/course/odom'].publish(odom)
        pubs['/course/lio_points'].publish(cloud(stamp, 'odom', invalid_map))
        spin(.006)

    def stationary(norm=9.81, count=150):
        for _ in range(count):
            publish(norm)

    def loop(count=180):
        for index in range(count):
            publish(5.8 if index % 2 == 0 else 18.2,
                    x=2.*math.sin(math.pi*index/(count-1)))

    def reset_return():
        node.set_parameters([Parameter('returned_to_start', value=False)])

    def close():
        node.set_parameters([Parameter('returned_to_start', value=True)])
        answer = call('finish_trial')
        assert answer.success
        return json.loads(answer.message)

    try:
        spin(.5)
        assert not call('start_trial').success
        stationary(1.)
        assert not call('start_trial').success
        stationary()
        assert call('start_trial').success
        loop()
        node.tick()
        health = json.loads((root/'health.json').read_text())
        assert health['ready'] and health['health_phase'] == 'recording'
        assert not health['imu_stationary_ready'] and health['imu_norm_std_mps2'] > .8
        assert not node.monitor.health(node.now_s(), require_stationary=True)['ready']
        stationary()
        first = close()
        assert first['success'] and first['health_at_start']['ready']
        assert not first['data_faults'] and not node.monitor.motion_active
        report['dynamic_trial_passed'] = True
        report['stationary_admission_and_finish_retained'] = True

        reset_return()
        assert call('start_trial').success
        loop()
        publish(float('nan'))
        assert 'imu:invalid_values' in node.active['data_faults']
        stationary()
        assert not node.monitor.health(node.now_s())['ready']
        second = close()
        assert not second['success'] and 'imu:invalid_values' in second['data_faults']
        report['invalid_imu_not_erased_by_recovery'] = True

        reset_return()
        stationary()
        assert call('start_trial').success
        publish(invalid_cloud=True, invalid_map=True)
        stationary()
        assert node.monitor.health(node.now_s())['ready']
        third = close()
        assert not third['success']
        assert {'cloud:invalid_values', 'map:invalid_values'} <= set(third['data_faults'])
        report['transient_empty_cloud_faults_retained'] = True
        node.tick()
        with (root/'health.csv').open(newline='') as handle:
            rows = list(csv.DictReader(handle))
        assert any(r['health_phase'] == 'recording' and r['imu_stationary_ready'] == 'False'
                   and r['ready'] == 'True' for r in rows)
        report['phase_diagnostics_written'] = True
        report['passed'] = True
        (root/'ros_motion_health_check.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    finally:
        executor.remove_node(node)
        executor.remove_node(peer)
        node.destroy_node()
        peer.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

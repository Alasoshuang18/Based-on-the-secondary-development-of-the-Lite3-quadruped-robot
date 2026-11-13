#!/usr/bin/env python3
"""Expose native Fortress XYZ points in the existing robot's optical TF frame.

Only the header frame name changes: no points, timing, axes or scan-pattern
fields are fabricated. Gazebo publishes these coordinates in the sensor frame.
"""
import rclpy
import signal
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def main():
    rclpy.init()
    node = Node('mid360_pointcloud_frame')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    publisher = node.create_publisher(PointCloud2, '/lite3_sim/mid360/points', qos)

    def receive(message):
        message.header.frame_id = 'mid360_optical_frame'
        publisher.publish(message)

    node.create_subscription(PointCloud2, '/lite3_sim/mid360/points_raw', receive, qos)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 launch may relay SIGINT after the process group already received
        # it; do not interrupt ROS object destruction a second time.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

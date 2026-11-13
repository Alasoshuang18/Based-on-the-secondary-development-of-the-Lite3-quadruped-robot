"""2-D SLAM using a Gazebo truth odometry prior; not an odometry benchmark."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        ExecuteProcess(cmd=['/usr/bin/python3', str(root/'sim_odometry.py')], output='screen'),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='navigation_cloud_to_scan', parameters=[{
                 'use_sim_time': True, 'target_frame': 'base_footprint',
                 'transform_tolerance': 0.15, 'min_height': 0.20, 'max_height': 1.20,
                 'angle_min': -3.141592653589793, 'angle_max': 3.141592653589793,
                 'angle_increment': 0.03490658503988659, 'scan_time': 0.2,
                 'range_min': 0.1, 'range_max': 15.0, 'use_inf': True,
                 'concurrency_level': 1}],
             remappings=[('cloud_in', '/lite3_sim/mid360/points'),
                         ('scan', '/lite3_sim/scan')], output='screen'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', parameters=[str(root/'config/slam.yaml')],
             output='screen'),
    ])

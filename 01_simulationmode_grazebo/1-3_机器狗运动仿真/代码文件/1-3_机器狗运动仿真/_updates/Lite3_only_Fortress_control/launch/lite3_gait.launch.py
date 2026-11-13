from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('simulationmode_grazebo'))
    return LaunchDescription([
        DeclareLaunchArgument('pin_body', default_value='false', choices=['true', 'false']),
        Node(package='simulationmode_grazebo', executable='lite3_controller',
             parameters=[str(share/'6-3_robot_sport/config/lite3_gait.yaml'),
                         {'pin_body': ParameterValue(LaunchConfiguration('pin_body'), value_type=bool),
                          'use_sim_time': True}],
             output='screen'),
    ])

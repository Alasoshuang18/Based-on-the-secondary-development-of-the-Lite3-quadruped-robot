"""Bridge actual contact sensors on the two existing navigation obstacles."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='lite3_obstacle_contacts_bridge', parameters=[{'use_sim_time': True}],
             arguments=['/lite3_sim/obstacle_contacts/'+side+
                        '@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts'
                        for side in ('front', 'left')], output='screen'),
    ])

"""Reuse the measured 4-2 sensor/SLAM chain without unused legacy consumers."""
from pathlib import Path

LAB42=Path.home()/'catkin_ws_lite3dog/src/04_slam/4-2_mapping_and_relocalization'


def prune_legacy_consumers(actions, context, result_dir):
    from launch.utilities import perform_substitutions,normalize_to_list_of_substitutions
    from launch_ros.actions import Node
    output=[]
    registry_count=0
    for action in actions:
        if isinstance(action,Node):
            executable=perform_substitutions(context,normalize_to_list_of_substitutions(action.node_executable))
            if executable in ('velocity_adapter','continuous_adapter'):
                continue
            if executable=='experiment':
                registry_count+=1
                action=Node(package='lite3_lab_5_2',executable='session_registry',
                            parameters=[{'result_dir':result_dir,'use_sim_time':False}],output='screen')
        output.append(action)
    if registry_count!=1:
        raise RuntimeError('Expected exactly one 4-2 registry/experiment; inspect shared bringup before starting')
    return output


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument,OpaqueFunction
    from launch.substitutions import LaunchConfiguration
    from ament_index_python.packages import get_package_share_directory
    from lite3_course_common.bringup import make_launch,_build
    shared=Path(get_package_share_directory('lite3_course_common'))
    original=make_launch('lite3_lab_4_2',str(LAB42/'launch/experiment.launch.py'),stack='mapping')
    declarations=[action for action in original.entities if isinstance(action,DeclareLaunchArgument)]
    def build(context):
        def value(name):
            return LaunchConfiguration(name).perform(context)
        if value('mode')!='real' or value('native_execute')!='false' or value('enable_real_motion')!='false':
            raise ValueError('Lean mapping is real sensor acquisition with legacy motion outputs disabled')
        actions=_build(context,'lite3_lab_4_2',LAB42,'mapping',shared)
        return prune_legacy_consumers(actions,context,value('result_dir'))
    return LaunchDescription(declarations+[OpaqueFunction(function=build)])

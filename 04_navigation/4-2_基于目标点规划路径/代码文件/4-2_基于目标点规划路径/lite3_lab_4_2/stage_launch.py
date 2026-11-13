"""Mapping-only stage launch; all motion remains in the unchanged official bridge."""
import json
import os
from pathlib import Path
import tempfile
import yaml
from .checkpoint import finite_pose, load_checkpoint, digest

WS = Path.home() / 'catkin_ws_lite3dog'
LAB42 = WS / 'src/04_slam/4-2_mapping_and_relocalization'


def configuration():
    return dict(length=float(os.environ['ROBOT_LENGTH']), width=float(os.environ['ROBOT_WIDTH']),
                padding=float(os.environ['FOOTPRINT_PADDING']),
                lidar_xyz=finite_pose(yaml.safe_load(os.environ['LIDAR_XYZ'])),
                lidar_rpy=finite_pose(yaml.safe_load(os.environ['LIDAR_RPY'])),
                body_height=float(os.environ['BODY_HEIGHT']),
                lidar_config_sha256=digest(os.environ['LIDAR_CONFIG']),
                motion='official_Lite3_ROS_MotionSender', max_forward_mps=.20, max_yaw_radps=.20)


def generate_stage_launch(stage):
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    def build(context):
        from launch.actions import GroupAction, IncludeLaunchDescription
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        from launch.substitutions import LaunchConfiguration
        from launch_ros.actions import SetParameter
        from ament_index_python.packages import get_package_share_directory
        def value(key):
            return LaunchConfiguration(key).perform(context)
        config = configuration()
        parent, start = value('checkpoint'), value('start_pose')
        if stage == 2 and parent:
            raise ValueError('5-2 starts a new map; use 5-3 to resume')
        if stage > 2 and not parent:
            raise ValueError('A previous-stage checkpoint is required')
        scoped = []
        if parent:
            load_checkpoint(parent, stage, config)
            pose = finite_pose(yaml.safe_load(start))
            if value('pose_confirmed') != 'true':
                raise ValueError('Verify the actual starting pose in the parent map, then set pose_confirmed:=true')
            prefix = str(Path(parent).expanduser().resolve().parent / 'slam')
            scoped += [SetParameter(name='map_file_name', value=prefix),
                       SetParameter(name='map_start_pose', value=pose),
                       SetParameter(name='map_start_at_dock', value=False)]
        results = WS / 'results'
        results.mkdir(parents=True, exist_ok=True)
        requested = value('result_dir')
        if requested:
            run = Path(requested).expanduser().resolve()
            run.mkdir(parents=True, exist_ok=True)
            if any(run.iterdir()):
                raise FileExistsError('Use a new empty result directory: ' + str(run))
        else:
            run = Path(tempfile.mkdtemp(prefix=f'5_{stage}_exploration_', dir=results))
        manifest = dict(stage=stage, configuration=config,
                        parent_checkpoint=str(Path(parent).expanduser().resolve()) if parent else None,
                        start_pose_map=pose if parent else None,
                        localization='slam_toolbox_mapping', map_to_odom_owner='slam_toolbox')
        (run / 'session.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
        (results / f'5_{stage}_current_run.txt').write_text(str(run)+'\n', encoding='utf-8')
        print(f'5-{stage} session: {run}', flush=True)
        args = dict(mode='real', start_runtime='true', start_hardware=value('start_hardware'),
                    lab42_latest_odometry='true', native_execute='false', enable_real_motion='false',
                    autostart='false', rviz='false', result_dir=str(run),
                    map=str(run/'maps/robot_dog_auto_map.yaml'),
                    lidar_config=os.environ['LIDAR_CONFIG'], lidar_xyz=os.environ['LIDAR_XYZ'],
                    lidar_rpy=os.environ['LIDAR_RPY'], initial_body_height=os.environ['BODY_HEIGHT'],
                    robot_length_m=os.environ['ROBOT_LENGTH'], robot_width_m=os.environ['ROBOT_WIDTH'],
                    footprint_padding_m=os.environ['FOOTPRINT_PADDING'])
        scoped.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('lite3_lab_5_2'))/'launch/lean_mapping.launch.py')), launch_arguments=args.items()))
        return [GroupAction(actions=scoped)]
    return LaunchDescription([
        DeclareLaunchArgument('result_dir', default_value=''),
        DeclareLaunchArgument('checkpoint', default_value=''),
        DeclareLaunchArgument('start_pose', default_value='[]'),
        DeclareLaunchArgument('pose_confirmed', default_value='false'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        OpaqueFunction(function=build)])

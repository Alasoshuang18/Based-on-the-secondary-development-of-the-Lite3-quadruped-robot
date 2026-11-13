"""所有实验共享的启动约定；实验算法仍放在各实验自己的Python节点中。"""
from datetime import datetime
from pathlib import Path
import json
import math
import os
import yaml

def make_launch(package, launch_file, stack='sensors'):
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('lite3_course_common'))
    lab = Path(launch_file).resolve().parents[1]
    scene = 'dynamic' if package == 'lite3_lab_5_4' else 'room'
    defaults = dict(mode='real', autostart='false', rviz='true', gui='false',
                    world=str(share/'worlds'/(scene+'.sdf')),
                    map=str(Path.home()/'catkin_ws_lite3dog/maps/robot_dog_auto_map.yaml'),
                    params_file=str(lab/'config/params.yaml'),
                    result_dir=str(Path.home()/'catkin_ws_lite3dog/results'/package/datetime.now().strftime('%Y%m%d_%H%M%S')),
                    start_runtime='true', start_hardware='true', motion_backend='native', bag_source='real', initial_body_height='.35',
                    gait_autostart='true', lio_diagnostics='false', real_livox_serialized='true',
                    lab42_latest_odometry='false', lab42_idle_recovery='false', native_execute='false', native_sha256='', enable_real_motion='false', publish_sensor_tf='true',
                    lidar_config=str(share/'config/MID360_config.json'),
                    robot_length_m='1.2', robot_width_m='.8', footprint_padding_m='.15', scan_range_max='15.',
                    lidar_xyz='[0.22,0.0,0.175]', lidar_rpy='[0.0,0.0,0.0]',
                    imu_lidar_xyz='[-0.011,-0.02329,0.04412]', imu_lidar_rpy='[0.0,0.0,0.0]')
    if package != 'lite3_lab_3_2': defaults['bag_path'] = ''
    if stack in ('navigation','exploration') or package == 'lite3_lab_3_4':
        defaults['processor_params_file'] = str(Path(get_package_share_directory('lite3_lab_3_3'))/'config/params.yaml')
    actions = [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
    actions.append(OpaqueFunction(function=lambda context: _build(context, package, lab, stack, share)))
    return LaunchDescription(actions)

def _build(context, package, lab, stack, share):
    from launch.substitutions import LaunchConfiguration
    from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import Node
    from ament_index_python.packages import get_package_share_directory
    def value(name): return LaunchConfiguration(name).perform(context)
    def boolean(name):
        text = value(name).lower()
        if text not in ('true', 'false'): raise ValueError(name+'必须为true/false')
        return text == 'true'
    mode = value('mode')
    if mode not in ('sim','real','bag'): raise ValueError('mode必须为sim/real/bag')
    use_sim = mode != 'real'
    result = Path(value('result_dir')).expanduser()
    result.mkdir(parents=True, exist_ok=True)
    params = str(Path(value('params_file')).expanduser())
    map_path = str(Path(value('map')).expanduser())
    has_lio = stack in ('lio','mapping','localization','navigation','exploration')
    mapping = stack in ('mapping','exploration')
    navigation = stack in ('navigation','exploration')
    localization = stack in ('localization','navigation')
    camera = package == 'lite3_lab_3_6'
    source_mode = value('bag_source') if mode == 'bag' else mode
    if source_mode not in ('sim','real'): raise ValueError('bag_source必须为sim/real')
    actions = []
    import numpy as np
    from .geometry import rotation_matrix, rpy_quaternion, quaternion_from_matrix
    def vector(name):
        item = np.asarray(yaml.safe_load(value(name)),dtype=float)
        if item.shape != (3,) or not np.isfinite(item).all(): raise ValueError(name+'必须为3个有限数')
        return item
    lidar_xyz, lidar_rpy = vector('lidar_xyz'), vector('lidar_rpy')
    lidar_rotation = rotation_matrix(rpy_quaternion(lidar_rpy))
    if source_mode == 'real':
        imu_lidar_xyz = vector('imu_lidar_xyz')
        imu_lidar_rotation = rotation_matrix(rpy_quaternion(vector('imu_lidar_rpy')))
        base_imu_rotation = lidar_rotation@imu_lidar_rotation.T
        base_imu_xyz = lidar_xyz-base_imu_rotation@imu_lidar_xyz
    else:
        base_imu_rotation, base_imu_xyz = np.eye(3), np.zeros(3)
        imu_lidar_xyz, imu_lidar_rotation = lidar_xyz, lidar_rotation
    length, width, padding = (float(value(key)) for key in ('robot_length_m','robot_width_m','footprint_padding_m'))
    footprint, envelope_radius = measured_envelope(length, width, padding)
    scan_range = float(value('scan_range_max'))
    if not math.isfinite(scan_range) or scan_range <= .2: raise ValueError('scan_range_max必须大于0.2m且有限')
    experiment_extra = {}
    if package in ('lite3_lab_3_4','lite3_lab_3_5'):
        experiment_extra['allow_real_motion'] = boolean('enable_real_motion')
    if package == 'lite3_lab_4_4' or package.startswith('lite3_lab_5_'):
        experiment_extra['enable_real_motion'] = boolean('enable_real_motion')
    if package == 'lite3_lab_4_4': experiment_extra['footprint_radius_m'] = envelope_radius
    if package.startswith('lite3_lab_5_') or package == 'lite3_lab_3_5':
        experiment_extra.update(robot_length_m=length, robot_width_m=width, footprint_padding_m=padding)
    if not boolean('start_runtime'):
        # 已有公共栈时只启动本课，不能重复TF、里程计、导航或速度仲裁。
        actions.append(Node(package=package, executable='experiment', parameters=[params, {
            'mode':mode,'use_sim_time':use_sim,'autostart':boolean('autostart'),
            'result_dir':str(result),'map_file':map_path,'map_path':map_path, **experiment_extra}], output='screen'))
        if boolean('rviz'):
            rviz = lab/'rviz/experiment.rviz'
            if not rviz.is_file(): rviz = share/'rviz/course.rviz'
            actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d',str(rviz)],
                                parameters=[{'use_sim_time':use_sim}]))
        return actions
    from .runtime_guard import acquire_runtime
    acquire_runtime(mode, result)
    if mode == 'bag' and package != 'lite3_lab_3_2':
        bag = Path(value('bag_path')).expanduser()
        if not (bag/'metadata.yaml').is_file(): raise FileNotFoundError('mode bag需要bag_path目录')
        topics = ['/course/points_raw','/course/imu']
        if source_mode == 'real': topics.append('/livox/lidar')
        if not has_lio: topics += ['/tf','/tf_static']
        actions.append(ExecuteProcess(cmd=['ros2','bag','play',str(bag),'--clock','50',
                '--qos-profile-overrides-path',str(share/'config/bag_lio_qos.yaml'),
                '--read-ahead-queue-size','100','--disable-keyboard-controls','--topics',*topics], output='screen'))
    if boolean('start_runtime') and mode == 'sim':
        from .model import spawn_actions
        world = Path(value('world')).expanduser()
        if not world.is_file(): world = share/'worlds'/(value('world')+'.sdf')
        if not world.is_file(): raise FileNotFoundError(world)
        command = ['ign','gazebo','-r','-s','-v','2',str(world)]
        # server-only仍需要渲染雷达，使用现有DISPLAY或EGL，绝非用伪造点云替代。
        actions.append(ExecuteProcess(cmd=command, output='screen'))
        if boolean('gui'):
            actions.append(ExecuteProcess(cmd=['ign','gazebo','-g'], output='screen'))
        actions += spawn_actions(camera=camera, motion_backend=value('motion_backend'),
                                 lidar_xyz=lidar_xyz, lidar_rpy=lidar_rpy,
                                 require_lio=has_lio,
                                 autostart_gait=boolean('gait_autostart') and (has_lio or package in ('lite3_lab_3_4','lite3_lab_3_5')))
        bridges = ['/course/dynamic_obstacle/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                   '/model/moving_box/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V']
        remaps = [('/model/moving_box/pose','/course/world_poses')]
        if camera:
            for topic, typ in [('image','Image'), ('depth_image','Image'), ('camera_info','CameraInfo')]:
                source = '/course/sim_camera/'+topic
                bridges.append(source+'@sensor_msgs/msg/'+typ+'[ignition.msgs.'+('Image' if typ=='Image' else 'CameraInfo'))
                remaps.append((source,'/course/camera_raw/'+topic))
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                            name='course_extra_bridge', arguments=bridges, remappings=remaps,
                            parameters=[{'use_sim_time':True}]))
    elif boolean('start_hardware') and mode == 'real':
        # 单一Livox实例以CustomMsg提供真实逐点时间，再由adapter转标准点云。
        if not camera:
            reference = Path(value('lidar_config')).expanduser()
            if not reference.is_file(): raise FileNotFoundError(reference)
            actions.append(Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
                parameters=[{'xfer_format':1, 'multi_topic':0, 'data_src':0, 'publish_freq':10.,
                             'output_data_type':0, 'frame_id':'lidar_link', 'user_config_path':str(reference),
                             'cmdline_input_bd_code':'livox0000000001', 'lvx_file_path':''}]))
        if camera:
            actions.append(Node(package='realsense2_camera', executable='realsense2_camera_node',
                 namespace='camera', name='camera', parameters=[{'enable_color':True,'enable_depth':True,
                  'enable_infra1':False,'enable_infra2':False,
                  'depth_module.depth_profile':'640x480x30','rgb_camera.color_profile':'640x480x30',
                  'align_depth.enable':True,'pointcloud.enable':False,'enable_sync':False}],
                 remappings=[('/camera/camera/color/image_raw','/course/camera/color/image_raw'),
                  ('/camera/camera/aligned_depth_to_color/image_raw','/course/camera/aligned_depth_to_color/image_raw'),
                  ('/camera/camera/aligned_depth_to_color/camera_info','/course/camera/aligned_depth_to_color/camera_info')]))
    # bag模式不另开硬件/clock。LIO适配只订阅后端输出，不重复发布录包原始数据。
    if (mode != 'bag' or has_lio) and not (mode == 'real' and camera):
        actions.append(Node(package='lite3_course_common', executable='sensor_adapter',
                            parameters=[{'mode':mode,'use_lio':has_lio,'use_sim_time':use_sim,
                                         'initial_body_height':float(value('initial_body_height'))}]))
        latest_odometry = (package == 'lite3_lab_4_2' and mode == 'real'
                           and boolean('lab42_latest_odometry'))
        adapter_entry = lab/'fastlio_latency_fix/sensor_adapter.py'
        if latest_odometry and not adapter_entry.is_file():
            raise FileNotFoundError(adapter_entry)
        actions[-1] = Node(package=None if latest_odometry else 'lite3_course_common',
            executable='/usr/bin/python3' if latest_odometry else 'sensor_adapter',
            arguments=['-B',str(adapter_entry)] if latest_odometry else [], parameters=[{
            'mode':mode,'use_lio':has_lio,'use_sim_time':use_sim,
            'real_livox_serialized':boolean('real_livox_serialized'),
            'initial_body_height':float(value('initial_body_height')),
            'base_imu_xyz':base_imu_xyz.tolist(),
            'base_imu_quat':quaternion_from_matrix(base_imu_rotation).tolist()}])
    if not (mode == 'real' and camera): actions.append(Node(package='lite3_course_common', executable='velocity_adapter',
                        parameters=[{'mode':mode,'use_sim_time':use_sim,
                                     'require_obstacles':navigation or package in ('lite3_lab_3_4','lite3_lab_3_5'),
                                     'require_lio':has_lio,
                                     'status_every_ticks':2 if package=='lite3_lab_4_2' and mode=='real' else 10,
                                     'enable_real_motion':boolean('enable_real_motion'),
                                     'robot_length_m':length,'robot_width_m':width,
                                     'require_gait_ready':mode=='sim' and value('motion_backend')=='gait'}]))
    if mode == 'real' and not camera:
        # 仅建立连续后端；execute开关与显式arm缺一不可，启动课程不会主动让真机行走。
        digest = value('native_sha256')
        if boolean('native_execute') and (len(digest)!=64 or any(c not in '0123456789abcdef' for c in digest)):
            raise ValueError('native_execute=true需要native_sha256为已编译runner的64位小写SHA256')
        idle_recovery = package == 'lite3_lab_4_2' and boolean('lab42_idle_recovery')
        runner_path = str(Path(__file__).resolve().parents[2] /
                          'lite3_course_native/native/native_runner_lab42') if idle_recovery else ''
        actions.append(Node(package='lite3_course_native', executable='continuous_adapter',
            parameters=[{'use_sim_time':False, 'execute':boolean('native_execute'),
                         'input_topic':'/course/cmd_vel_checked',
                         'expected_sha256':value('native_sha256'), 'native_path':runner_path,
                         'lab42_idle_recovery':idle_recovery}],output='screen'))
    if mode != 'bag' or has_lio:
        statics = [('base_link','TORSO',(0,0,0)), ('mid360_optical_frame','lidar_link',(0,0,0)),
                   ('TORSO','imu_link',(0,0,0)), ('map','world',(0,0,0))] if mode=='sim' else [
                   ('base_link','lidar_link',lidar_xyz), ('base_link','imu_link',base_imu_xyz)]
        # 实机外参是教学初值，必须按实际测量替换；真机连续运动出口默认未启用。
        if not has_lio: statics.append(('map','base_footprint',(0,0,0)))
        urdf_mounts = (package == 'lite3_lab_4_2' and mode == 'real'
                       and boolean('lab42_latest_odometry') and boolean('publish_sensor_tf'))
        if urdf_mounts:
            import importlib.util
            source = lab/'fastlio_latency_fix/sensor_mounts.py'
            spec = importlib.util.spec_from_file_location('lab42_measured_sensor_mounts',source)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            description = module.measured_sensor_urdf(lidar_xyz,lidar_rpy,base_imu_xyz,base_imu_rotation)
            (result/'sensor_mounts.urdf').write_text(description,encoding='utf-8')
            actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                name='course_sensor_mounts', parameters=[{'use_sim_time':False,
                    'robot_description':description}], output='screen'))
        for index, (parent, child, xyz) in enumerate(statics):
            if urdf_mounts and child in ('lidar_link','imu_link'):
                continue
            if mode != 'sim' and child in ('lidar_link','imu_link') and not boolean('publish_sensor_tf'):
                continue
            quat = [0.,0.,0.,1.]
            if mode != 'sim' and child == 'lidar_link': quat = rpy_quaternion(lidar_rpy)
            if mode != 'sim' and child == 'imu_link': quat = quaternion_from_matrix(base_imu_rotation)
            actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                name='course_static_'+str(index), arguments=['--x',str(xyz[0]),'--y',str(xyz[1]),'--z',str(xyz[2]),
                  '--qx',str(quat[0]),'--qy',str(quat[1]),'--qz',str(quat[2]),'--qw',str(quat[3]),
                  '--frame-id',parent,'--child-frame-id',child],
                parameters=[{'use_sim_time':use_sim}]))
    if has_lio:
        config = _lio_config(result, source_mode, use_sim, imu_lidar_xyz, imu_lidar_rotation,
                             diagnostics=boolean('lio_diagnostics'),
                             imu_topic=_lio_imu_topic(package, source_mode))
        actions.append(Node(package='lite3_course_common', executable='lio_starter',
                            parameters=[{'use_sim_time':use_sim,'lio_config':str(config)}],output='screen'))
        actions.append(Node(package='lite3_course_common', executable='map_archive',
                            parameters=[{'use_sim_time':use_sim,'map_path':map_path}]))
    if mapping or localization or navigation:
        scan_tf_env = {}
        if package == 'lite3_lab_4_2' and mode == 'real':
            tf2_library = lab/'tf2_time_fix/install/lib/libtf2.so'
            if not tf2_library.is_file():
                raise FileNotFoundError('4-2 TF2修复库缺失：'+str(tf2_library))
            scan_tf_env = {'LD_LIBRARY_PATH':str(tf2_library.parent)+os.pathsep+os.environ.get('LD_LIBRARY_PATH','')}
        actions.append(Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
           name='course_scan', additional_env=scan_tf_env, remappings=[('cloud_in','/course/points_raw'),('scan','/scan')],
           parameters=[{'use_sim_time':use_sim,'target_frame':'base_footprint','transform_tolerance':.2,
                        'min_height':.2,'max_height':(.65 if package=='lite3_lab_4_2' and mode=='real' and boolean('lab42_latest_odometry') else 1.5),
                        'range_min':.2,'range_max':scan_range,
                        'angle_min':-3.14159,'angle_max':3.14159,'angle_increment':.0174533,
                        'scan_time':(.1 if package=='lite3_lab_4_2' and mode=='real' else .2),'use_inf':True,
                        **({'queue_size':8} if package=='lite3_lab_4_2' else {'concurrency_level':1})}]))
    if mapping:
        slam = dict(use_sim_time=use_sim, odom_frame='odom', map_frame='map', base_frame='base_footprint',
                    scan_topic='/scan', mode='mapping', resolution=.05, max_laser_range=scan_range,
                    minimum_travel_distance=.03, minimum_travel_heading=.03, map_update_interval=1.,
                    transform_publish_period=.05, minimum_time_interval=.15, throttle_scans=1,
                    solver_plugin='solver_plugins::CeresSolver', ceres_linear_solver='SPARSE_NORMAL_CHOLESKY',
                    ceres_preconditioner='SCHUR_JACOBI', ceres_trust_strategy='LEVENBERG_MARQUARDT',
                    ceres_dogleg_type='TRADITIONAL_DOGLEG', ceres_loss_function='None',
                    use_scan_matching=True, do_loop_closing=True)
        if package == 'lite3_lab_4_2' and mode == 'real':
            slam['min_laser_range'] = .2  # Matches /scan.range_min; no guard threshold change.
        actions.append(Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                            name='slam_toolbox', additional_env=scan_tf_env, parameters=[slam], output='screen'))
    if localization or navigation:
        nav_params = _nav_config(result, use_sim, map_path, length, width, padding, scan_range)
        nav_share = Path(get_package_share_directory('nav2_bringup'))
        if localization:
            if not Path(map_path).is_file(): raise FileNotFoundError('请先完成4-2/4-4保存地图：'+map_path)
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav_share/'launch/localization_launch.py')),
                launch_arguments={'map':map_path,'params_file':str(nav_params),'use_sim_time':str(use_sim).lower(),
                                  'autostart':'true','use_composition':'False'}.items()))
        if navigation:
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav_share/'launch/navigation_launch.py')),
                launch_arguments={'params_file':str(nav_params),'use_sim_time':str(use_sim).lower(),
                                  'autostart':'true','use_composition':'False'}.items()))
    if navigation or package == 'lite3_lab_3_4':
        from ament_index_python.packages import get_package_share_directory as locate
        processor = Path(value('processor_params_file')).expanduser()
        if not processor.is_file(): raise FileNotFoundError(processor)
        actions.append(Node(package='lite3_lab_3_3', executable='experiment', parameters=[str(processor),
            {'mode':mode,'use_sim_time':use_sim,'autostart':True,'result_dir':str(result/'perception')}]))
    actions.append(Node(package=package, executable='experiment', parameters=[params, {
        'mode':mode,'use_sim_time':use_sim,'autostart':boolean('autostart'),
        'result_dir':str(result),'map_file':map_path,'map_path':map_path, **experiment_extra}], output='screen'))
    if boolean('rviz'):
        rviz = lab/'rviz/experiment.rviz'
        if not rviz.is_file(): rviz = share/'rviz/course.rviz'
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d',str(rviz)],
                            parameters=[{'use_sim_time':use_sim}], output='screen'))
    return actions

def _lio_imu_topic(package, mode):
    # FAST-LIO normalizes the MID-360 acceleration units during IMU initialization.
    # Keep 4-2 inference independent of the Python point-cloud adapter's callback load.
    return '/livox/imu' if package == 'lite3_lab_4_2' and mode == 'real' else '/course/imu'


def _lio_config(result, mode, use_sim, imu_lidar_xyz, imu_lidar_rotation, diagnostics=False,
                imu_topic='/course/imu'):
    data = {'use_sim_time':use_sim, 'feature_extract_enable':False,'point_filter_num':3,'max_iteration':3,
            # 上游MOV_THRESHOLD=1.5；cube/2必须大于1.5*det_range，否则静止也反复修剪地图。
            'filter_size_surf':.5,'filter_size_map':.5,'cube_side_length':120.,'runtime_pos_log_enable':False,
            'common.lid_topic':'/livox/lidar' if mode=='real' else '/course/points_raw',
            'common.imu_topic':imu_topic,'common.time_sync_en':False,'common.time_offset_lidar_to_imu':0.,
            # 官方ROS2分支0走default_handler，适用于无逐点offset的瞬时扫描；不伪造时间。
            'preprocess.lidar_type':1 if mode=='real' else 0,'preprocess.scan_line':4 if mode=='real' else 24,
            'preprocess.blind':.3,'preprocess.timestamp_unit':0,'preprocess.scan_rate':5,
            'mapping.acc_cov':.1,'mapping.gyr_cov':.1,'mapping.b_acc_cov':.0001,'mapping.b_gyr_cov':.0001,
            'mapping.fov_degree':360.,'mapping.det_range':30.,'mapping.extrinsic_est_en':False,
            'mapping.extrinsic_T':imu_lidar_xyz.tolist(), 'mapping.extrinsic_R':imu_lidar_rotation.flatten().tolist(),
            'publish.path_en':True,'publish.effect_map_en':False,'publish.map_en':False,
            'publish.scan_publish_en':True,'publish.dense_publish_en':False,'publish.scan_bodyframe_pub_en':True,
            'pcd_save.pcd_save_en':False,'pcd_save.interval':-1}
    # 只增加实际预测/更新日志和未降采样输出；估计输入、逐点时间和算法参数保持原值。
    if diagnostics:
        data['runtime_pos_log_enable'] = True
        data['publish.dense_publish_en'] = True
    path = result/'lio_runtime.yaml'
    path.write_text(yaml.safe_dump({'/**':{'ros__parameters':data}}), encoding='utf-8')
    return path

def measured_envelope(length, width, padding):
    """以实测足腿完整矩形包络生成Nav2足迹及探索保守外接半径。"""
    if not all(math.isfinite(v) for v in (length,width,padding)) or length <= 0 or width <= 0 or padding < 0:
        raise ValueError('机身长宽必须为正数，padding必须非负，且均为有限数')
    x,y = length/2, width/2
    return json.dumps([[x,y],[x,-y],[-x,-y],[-x,y]]), math.ceil(math.hypot(x+padding,y+padding)*100)/100

def _nav_config(result, use_sim, map_path, robot_length_m=1.2, robot_width_m=.8, footprint_padding_m=.15, scan_range_max=15.):
    from ament_index_python.packages import get_package_share_directory
    source = Path(get_package_share_directory('nav2_bringup'))/'params/nav2_params.yaml'
    data = yaml.safe_load(source.read_text())
    footprint, _ = measured_envelope(robot_length_m, robot_width_m, footprint_padding_m)
    def walk(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if key == 'use_sim_time': value[key] = use_sim
                elif key in ('robot_base_frame','base_frame_id'): value[key] = 'base_footprint'
                elif key == 'odom_topic': value[key] = '/course/odom'
                else: walk(item)
        elif isinstance(value, list):
            for item in value: walk(item)
    walk(data)
    amcl = data['amcl']['ros__parameters']
    amcl.update(scan_topic='/scan', min_particles=300, max_particles=1000,
                laser_max_range=scan_range_max, update_min_d=.02, update_min_a=.02, set_initial_pose=False)
    data['map_server']['ros__parameters']['yaml_filename'] = map_path
    controller = data['controller_server']['ros__parameters']
    controller.update(controller_frequency=10., odom_topic='/course/odom')
    controller['progress_checker'].update(required_movement_radius=.03, movement_time_allowance=60.)
    controller['general_goal_checker'].update(xy_goal_tolerance=.15, yaw_goal_tolerance=.25)
    follow = controller['FollowPath']
    # BaseObstacle只检查中心格；矩形机身/四足包络必须同时检查完整足迹边界。
    critics = list(follow.get('critics', []))
    for critic in ('BaseObstacle','ObstacleFootprint'):
        if critic not in critics: critics.append(critic)
    follow['critics'] = critics
    follow['ObstacleFootprint.scale'] = .02
    follow.update(min_vel_x=0., max_vel_x=.02, max_vel_y=0., max_vel_theta=.02,
                  max_speed_xy=.02, min_speed_xy=0., min_speed_theta=0.,
                  acc_lim_x=.05, acc_lim_y=0., acc_lim_theta=.10, decel_lim_x=-.05,
                  decel_lim_y=0., decel_lim_theta=-.10, sim_time=2., vx_samples=10, vy_samples=1, vtheta_samples=10)
    for name in ('local_costmap','global_costmap'):
        cost = data[name][name]['ros__parameters']
        cost.update(robot_base_frame='base_footprint', footprint=footprint,
                    # 0.10m目标净空加一个0.05m栅格裕量；实际间隙仍须逐次测量。
                    footprint_padding=footprint_padding_m, resolution=.05)
        cost.pop('robot_radius', None)
        if name == 'local_costmap':
            # 5-3审计需要实际完整栅格，不能只发送增量后以旧完整图冒充当前图。
            cost.update(always_send_full_costmap=True, publish_frequency=2.)
        cost['inflation_layer'].update(inflation_radius=.9,cost_scaling_factor=3.)
        for layer in ('voxel_layer','obstacle_layer'):
            if layer in cost:
                for sensor in ('scan',):
                    if sensor in cost[layer]: cost[layer][sensor]['topic'] = '/scan'
    smoother=data['velocity_smoother']['ros__parameters']
    smoother.update(max_velocity=[.02,0.,.02],min_velocity=[0.,0.,-.02],
                    max_accel=[.05,0.,.10],max_decel=[-.05,0.,-.10])
    # navigation_launch最终由velocity_smoother向/cmd_vel输出，课程单出口也订阅它。
    path = result/'nav2_runtime.yaml'
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    return path

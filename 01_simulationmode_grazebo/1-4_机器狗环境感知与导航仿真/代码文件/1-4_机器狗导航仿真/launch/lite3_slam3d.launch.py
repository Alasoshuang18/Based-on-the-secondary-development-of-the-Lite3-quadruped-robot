"""Full 6-DoF lidar ICP odometry and RTAB-Map graph SLAM, without Gazebo poses."""
from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    root = Path(__file__).resolve().parents[1]
    database = Path(LaunchConfiguration('database').perform(context)).expanduser().resolve()
    if not database.is_relative_to(root):
        raise RuntimeError('Database must stay within 6-4_robot_navigation')
    database.parent.mkdir(parents=True, exist_ok=True)
    common = {
        'use_sim_time': True, 'frame_id': 'TORSO', 'qos': 2,
        'wait_for_transform': .3,
        'Reg/Force3DoF': 'false', 'Icp/Strategy': '1',
        'Icp/PointToPlane': 'true', 'Icp/Iterations': '10',
        'Icp/VoxelSize': '0.06', 'Icp/Epsilon': '0.001',
        'Icp/PointToPlaneK': '10', 'Icp/PointToPlaneRadius': '0',
        'Icp/MaxCorrespondenceDistance': '0.4', 'Icp/OutlierRatio': '0.7',
    }
    return [
        Node(package='rtabmap_odom', executable='icp_odometry',
             namespace='lite3_sim/slam3d', name='icp_odometry', output='screen',
             parameters=[common, {
                 'odom_frame_id': 'odom3d', 'publish_tf': True,
                 'scan_cloud_is_2d': False, 'deskewing': False,
                 'scan_range_min': .3, 'scan_range_max': 12.,
                 'scan_cloud_max_points': 0,
                 'expected_update_rate': 6.,
                 'Odom/ResetCountdown': '0',
                 'Odom/ScanKeyFrameThr': '0.6',
                 'OdomF2M/ScanMaxSize': '6000',
                 'OdomF2M/ScanSubtractRadius': '0.06',
                 'OdomF2M/BundleAdjustment': 'false',
                 'Icp/CorrespondenceRatio': '0.1',
             }], remappings=[('scan_cloud', '/lite3_sim/mid360/points'),
                             ('odom', '/lite3_sim/slam3d/odom')]),
        Node(package='rtabmap_slam', executable='rtabmap',
             namespace='lite3_sim/slam3d', name='rtabmap', output='screen',
             parameters=[common, {
                 'database_path': str(database), 'map_frame_id': 'map3d',
                 'subscribe_depth': False, 'subscribe_rgb': False,
                 'subscribe_scan': False, 'subscribe_scan_cloud': True,
                 'subscribe_odom_info': True, 'approx_sync': False,
                 'topic_queue_size': 3, 'sync_queue_size': 3,
                 'Reg/Strategy': '1', 'Optimizer/Slam2D': 'false',
                 'RGBD/ProximityBySpace': 'true',
                 'RGBD/ProximityMaxGraphDepth': '0',
                 'RGBD/ProximityPathMaxNeighbors': '1',
                 'RGBD/AngularUpdate': '0.03', 'RGBD/LinearUpdate': '0.03',
                 'RGBD/CreateOccupancyGrid': 'true',
                 'Mem/NotLinkedNodesKept': 'false', 'Mem/STMSize': '10',
                 'Rtabmap/DetectionRate': '2', 'Rtabmap/MemoryThr': '200',
                 'Rtabmap/TimeThr': '150',
                 'Icp/CorrespondenceRatio': '0.2',
                 'Grid/Sensor': '0', 'Grid/3D': 'true',
                 'Grid/CellSize': '0.08', 'Grid/RangeMax': '12',
                 'Grid/MinGroundHeight': '-0.5', 'Grid/MaxGroundHeight': '-0.2',
                 'Grid/MaxGroundAngle': '25', 'Grid/NormalK': '10',
                 'Grid/MinClusterSize': '3', 'Grid/MaxObstacleHeight': '2',
                 'Grid/RayTracing': 'false', 'GridGlobal/MaxNodes': '200',
             }], remappings=[('scan_cloud', '/lite3_sim/mid360/points'),
                             ('odom', '/lite3_sim/slam3d/odom')]),
    ]


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument('database', default_value=str(
            root/'maps'/('slam3d_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.db'))),
        OpaqueFunction(function=setup),
    ])

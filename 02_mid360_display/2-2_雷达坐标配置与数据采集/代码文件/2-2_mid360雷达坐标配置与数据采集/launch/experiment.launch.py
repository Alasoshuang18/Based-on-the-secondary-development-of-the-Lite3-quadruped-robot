"""mode=bag时启动唯一回放clock；排除bag中已经录过的/clock消息。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from lite3_course_common.bringup import make_launch


def playback(context):
    if LaunchConfiguration('mode').perform(context) != 'bag':
        return []
    if LaunchConfiguration('play_bag').perform(context).lower() != 'true':
        return []
    path = Path(LaunchConfiguration('bag_path').perform(context)).expanduser()
    if not (path / 'metadata.yaml').is_file():
        raise RuntimeError('mode:=bag需要bag_path:=含metadata.yaml的录包目录；外部回放可设play_bag:=false')
    rate = float(LaunchConfiguration('playback_rate').perform(context))
    if not .05 <= rate <= 4.:
        raise RuntimeError('playback_rate必须在0.05~4.0之间')
    # --topics白名单不包含/clock，保证只有rosbag生成的时钟生效。
    from lite3_course_common.runtime_guard import ensure_no_clock
    ensure_no_clock()
    qos_file = str(Path(get_package_share_directory('lite3_lab_3_2')) / 'config' / 'bag_qos.yaml')
    topics = ['/course/points_raw', '/course/imu', '/tf', '/tf_static']
    if LaunchConfiguration('bag_source').perform(context) == 'real':
        topics.insert(0, '/livox/lidar')  # 真实录包的逐点时间，供FAST_LIO复算。
    return [ExecuteProcess(cmd=['ros2', 'bag', 'play', str(path), '--clock', '50', '--rate', str(rate),
                    '--read-ahead-queue-size', '100', '--disable-keyboard-controls',
                    '--qos-profile-overrides-path', qos_file, '--topics', *topics], output='screen')]


def generate_launch_description():
    description = make_launch('lite3_lab_3_2', __file__, stack='sensors')
    description.add_action(DeclareLaunchArgument('bag_path', default_value='', description='rosbag2目录'))
    description.add_action(DeclareLaunchArgument('play_bag', default_value='true'))
    description.add_action(DeclareLaunchArgument('playback_rate', default_value='1.0'))
    description.add_action(OpaqueFunction(function=playback))
    return description

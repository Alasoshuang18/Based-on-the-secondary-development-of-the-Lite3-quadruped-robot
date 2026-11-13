"""Spawn a control-enabled Lite3 with IMU and simulation clock bridges."""
from pathlib import Path, PurePosixPath
import math
import re
import xml.etree.ElementTree as ET
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from navigation_attachments import attach, ROOT
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_spawn(pin_body=False, enable_lidar=True, with_payload=True, payload_mass=1.5071784,
               lidar_hz=5., horizontal=180, vertical=12):
    """Build one robot; an optional world weld isolates leg calibration.

    This weld is an SDF extension, so the ROS URDF root remains TORSO. The
    model stays dynamic and its twelve controlled leg joints remain movable.
    A pinned-body test cannot qualify a free-body walking controller.
    """
    if not isinstance(pin_body, bool):
        raise ValueError('pin_body must be a boolean')
    share = Path(get_package_share_directory('simulationmode_grazebo'))
    root = ET.fromstring((share / '6-2_robot_modeling/urdf/Lite3.urdf').read_text())
    yaml = share / '6-3_robot_sport/config/lite3_controllers.yaml'
    if not yaml.is_file():
        raise RuntimeError('Missing installed lite3_controllers.yaml')
    # Keep the original package read-only. At a slow real-time factor a 30 Hz
    # simulated state publisher leaves long wall-time gaps during GUI/DDS work.
    # Publish measured controller state at the existing 100 Hz control rate;
    # joint limits, controller rate and all gait watchdogs remain unchanged.
    controller_text = yaml.read_text()
    pattern = r'(?m)^(\s*state_publish_rate:\s*)(30(?:\.0)?|100(?:\.0)?)(\s*(?:#.*)?)$'
    if len(list(re.finditer(pattern, controller_text))) != 1:
        raise RuntimeError('Unexpected original controller state_publish_rate; inspect before spawning')
    controller_text = re.sub(pattern, lambda m: m[1]+'100.0'+m[3], controller_text)
    runtime = ROOT/'.runtime'
    runtime.mkdir(exist_ok=True)
    yaml = runtime/'navigation_controllers.yaml'
    yaml.write_text(controller_text)
    if root.find('ros2_control') is not None:
        raise RuntimeError('Base URDF already contains ros2_control; inspect before adding another.')
    for child in list(root):
        if child.tag == 'mujoco':
            root.remove(child)
    prefix = 'package://simulationmode_grazebo/'
    for mesh in root.findall('.//mesh'):
        name = mesh.attrib['filename']
        if not name.startswith(prefix):
            raise RuntimeError('Unexpected mesh URI: ' + name)
        relative_name = name[len(prefix):]
        relative_path = PurePosixPath(relative_name)
        if (not relative_name or relative_path.is_absolute() or '..' in relative_path.parts
                or '\\' in relative_name or ':' in relative_name):
            raise RuntimeError('Mesh URI must be a relative resource in this package: ' + name)
        # --symlink-install resources may point into src.  Validate the URI
        # above, then allow the legitimate installed link to resolve there.
        path = share.joinpath(*relative_path.parts)
        if not path.is_file():
            raise RuntimeError('Missing mesh: ' + str(path))
        mesh.set('filename', path.resolve().as_uri())
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    control = ET.SubElement(root, 'ros2_control', name='Lite3SimSystem', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'gz_ros2_control/GazeboSimSystem'
    # Conservative bent-leg initial pose; not a validated balance controller.
    for leg in ('FL', 'FR', 'HL', 'HR'):
        for suffix, initial in (('HipX', 0.0), ('HipY', -0.65), ('Knee', 1.3)):
            name = f'{leg}_{suffix}_joint'
            if name not in joints or joints[name].attrib.get('type') != 'revolute':
                raise RuntimeError('Missing expected revolute joint: ' + name)
            limit = joints[name].find('limit')
            if not float(limit.attrib['lower']) <= initial <= float(limit.attrib['upper']):
                raise RuntimeError('Initial position outside joint limit: ' + name)
            joint = ET.SubElement(control, 'joint', name=name)
            command = ET.SubElement(joint, 'command_interface', name='position')
            ET.SubElement(command, 'param', name='min').text = limit.attrib['lower']
            ET.SubElement(command, 'param', name='max').text = limit.attrib['upper']
            state = ET.SubElement(joint, 'state_interface', name='position')
            ET.SubElement(state, 'param', name='initial_value').text = str(initial)
            ET.SubElement(joint, 'state_interface', name='velocity')
    # Match the Gazebo sensor name so gz_ros2_control can register its states.
    # The gait still consumes the independently bridged sensor_msgs/Imu topic.
    control_imu = ET.SubElement(control, 'sensor', name='lite3_imu')
    for interface in ('orientation.x', 'orientation.y', 'orientation.z', 'orientation.w',
                      'angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z',
                      'linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z'):
        ET.SubElement(control_imu, 'state_interface', name=interface)
    gazebo = ET.SubElement(root, 'gazebo')
    if pin_body:
        # Non-reference <gazebo> blobs are copied into the generated SDF model.
        # DART supports a fixed joint from the implicit world to the root link.
        # Do not mark the model static: that also immobilizes its leg joints.
        pin = ET.SubElement(gazebo, 'joint',
                            name='lite3_calibration_world_fixed', type='fixed')
        ET.SubElement(pin, 'parent').text = 'world'
        ET.SubElement(pin, 'child').text = 'TORSO'
    plugin_library = Path(get_package_prefix('gz_ros2_control')) / 'lib/libgz_ros2_control-system.so'
    if not plugin_library.is_file():
        raise RuntimeError('Missing Gazebo control plugin library: ' + str(plugin_library))
    plugin = ET.SubElement(gazebo, 'plugin', filename=str(plugin_library),
                           name='gz_ros2_control::GazeboSimROS2ControlPlugin')
    ET.SubElement(plugin, 'robot_param').text = 'robot_description'
    # This Humble plugin prepends its ROS namespace when locating the parameter node.
    ET.SubElement(plugin, 'robot_param_node').text = 'robot_state_publisher'
    ET.SubElement(plugin, 'parameters').text = str(yaml)
    ros = ET.SubElement(plugin, 'ros')
    ET.SubElement(ros, 'namespace').text = '/lite3_sim'
    # Fortress PosePublisher puts simulation stamps on individual Pose_V poses.
    # A single model pose is enough for measured foot FK; avoid the large world
    # pose stream and PoseArray's missing top-level stamp on Humble.
    poses = ET.SubElement(gazebo, 'plugin',
                          filename='ignition-gazebo-pose-publisher-system',
                          name='ignition::gazebo::systems::PosePublisher')
    for tag, value in (('publish_model_pose', 'true'),
                       ('publish_nested_model_pose', 'true'),
                       ('publish_link_pose', 'false'),
                       ('publish_visual_pose', 'false'),
                       ('publish_collision_pose', 'false'),
                       ('publish_sensor_pose', 'false'),
                       ('use_pose_vector_msg', 'true'),
                       ('static_publisher', 'false'),
                       ('update_frequency', '50')):
        ET.SubElement(poses, tag).text = value
    sensor_tag = ET.SubElement(root, 'gazebo', reference='TORSO')
    sensor = ET.SubElement(sensor_tag, 'sensor', name='lite3_imu', type='imu')
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '100'
    ET.SubElement(sensor, 'topic').text = '/lite3_sim/imu'
    ET.SubElement(sensor, 'pose').text = '0 0 0 0 0 0'
    ET.SubElement(sensor, 'imu')
    # Visuals only: physical properties and collision geometries are unchanged.
    colors = {'FL': '0.9 0.1 0.1 1', 'FR': '0.1 0.8 0.1 1',
              'HL': '0.1 0.2 0.9 1', 'HR': '0.95 0.7 0.05 1'}
    for leg, rgba in colors.items():
        link = root.find("link[@name='" + leg + "_FOOT']")
        if link is None:
            raise RuntimeError('Missing FOOT visual attachment: ' + leg)
        visual = ET.SubElement(link, 'visual', name=leg + '_foot_debug_visual')
        geometry = ET.SubElement(visual, 'geometry')
        ET.SubElement(geometry, 'sphere', radius='0.022')
        material = ET.SubElement(visual, 'material', name=leg + '_foot_debug_material')
        ET.SubElement(material, 'color', rgba=rgba)
    for link in root.findall('link'):
        if link.attrib['name'].endswith('_FOOT'):
            continue
        for visual in link.findall('visual'):
            material = visual.find('material')
            if material is None:
                material = ET.SubElement(visual, 'material', name=link.attrib['name'] + '_debug_material')
            material.set('name', link.attrib['name'] + '_debug_material')
            color = material.find('color')
            if color is None:
                color = ET.SubElement(material, 'color')
            color.set('rgba', '0.35 0.4 0.45 1')
    attach(root, payload_mass, with_payload, enable_lidar, lidar_hz, horizontal, vertical)
    description = ET.tostring(root, encoding='unicode')
    generated = ROOT/'.runtime'
    generated.mkdir(exist_ok=True)
    (generated/'lite3_navigation.urdf').write_text(description, encoding='utf-8')
    pose_topic = '/model/lite3_control/pose'
    # At the fixed initial q=(0,-0.65,1.3), FK gives the same z on all feet.
    # Pin at exact flat-ground standing height rather than fixing the original
    # 0.40 m drop-in pose, which would leave all four feet hanging in the air.
    spawn_z = (0.20 + 0.21012) * math.cos(0.65) + 0.022 if pin_body else 0.40
    actions = [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='lite3_feedback_bridge',
             parameters=[{'use_sim_time': True}],
             arguments=['/lite3_sim/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                        '/lite3_sim/ground_contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
                        pose_topic + '@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
             remappings=[(pose_topic, '/lite3_sim/model_pose')],
             output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='lite3_sim', name='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}],
             remappings=[('joint_states', '/lite3_sim/joint_state_broadcaster/joint_states')],
             output='screen'),
        Node(package='ros_gz_sim', executable='create', namespace='lite3_sim',
             arguments=['-world', 'lite3_world', '-name', 'lite3_control',
                        '-param', 'robot_description', '-z', str(spawn_z)],
             parameters=[{'robot_description': description, 'use_sim_time': True}], output='screen'),
    ]
    if enable_lidar:
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                            name='mid360_points_bridge',
                            arguments=['/lite3_sim/mid360/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                                       '--ros-args','-r','/lite3_sim/mid360/scan/points:=/lite3_sim/mid360/points_raw'],
                            parameters=[{'use_sim_time': True}], output='screen'))
        actions.append(Node(executable='/usr/bin/python3', arguments=[str(ROOT/'pointcloud_frame.py')],
                            parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def _spawn_from_context(context):
    choice = LaunchConfiguration('pin_body').perform(context).lower()
    if choice not in ('true', 'false'):
        raise RuntimeError('pin_body must be true or false')
    def boolean(name):
        value = LaunchConfiguration(name).perform(context).lower()
        if value not in ('true','false'):
            raise RuntimeError(name+' must be true or false')
        return value == 'true'
    return make_spawn(pin_body=(choice == 'true'), enable_lidar=boolean('enable_lidar'),
                      with_payload=boolean('with_payload'),
                      payload_mass=float(LaunchConfiguration('payload_mass').perform(context)),
                      lidar_hz=float(LaunchConfiguration('lidar_hz').perform(context)),
                      horizontal=int(LaunchConfiguration('horizontal_samples').perform(context)),
                      vertical=int(LaunchConfiguration('vertical_samples').perform(context)))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('with_payload', default_value='true'),
        DeclareLaunchArgument('payload_mass', default_value='1.5071784'),
        DeclareLaunchArgument('lidar_hz', default_value='5.0'),
        DeclareLaunchArgument('horizontal_samples', default_value='180'),
        DeclareLaunchArgument('vertical_samples', default_value='12'),
        DeclareLaunchArgument(
            'pin_body', default_value='false', choices=['true', 'false'],
            description='Fix TORSO to world for leg calibration only; false for free walking.'),
        OpaqueFunction(function=_spawn_from_context),
    ])

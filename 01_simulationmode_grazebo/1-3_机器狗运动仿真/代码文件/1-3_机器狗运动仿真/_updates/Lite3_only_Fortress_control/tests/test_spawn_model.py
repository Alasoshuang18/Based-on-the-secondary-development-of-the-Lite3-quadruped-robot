"""Check the generated spawn description without ROS or Gazebo installed.

This verifies the XML and launch wiring only; it does not exercise Gazebo's
physics, sensor plugin, or ros_gz_bridge at runtime.
"""

import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET


LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "lite3_spawn.launch.py"
LEGS = ("FL", "FR", "HL", "HR")
EXPECTED_IMU_INTERFACES = (
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
)


def fixture_urdf():
    """Small model with exactly the links and movable joints used by launch."""
    robot = ET.Element("robot", name="Lite3")
    ET.SubElement(robot, "link", name="TORSO")
    limits = {"HipX": (-0.523, 0.523), "HipY": (-2.67, 0.314),
              "Knee": (0.524, 2.792)}
    for leg in LEGS:
        ET.SubElement(robot, "link", name=f"{leg}_FOOT")
        for suffix, (lower, upper) in limits.items():
            joint = ET.SubElement(robot, "joint", name=f"{leg}_{suffix}_joint",
                                  type="revolute")
            ET.SubElement(joint, "limit", lower=str(lower), upper=str(upper),
                          effort="24", velocity="26.2")
    return ET.tostring(robot, encoding="unicode")


class StubNode:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class StubLaunchDescription:
    def __init__(self, actions):
        self.actions = actions


class StubAction:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class StubLaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


class SpawnModelTests(unittest.TestCase):
    def generate(self, pin_body=False):
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            share = base / "share" / "simulationmode_grazebo"
            urdf = share / "6-2_robot_modeling" / "urdf" / "Lite3.urdf"
            urdf.parent.mkdir(parents=True)
            urdf.write_text(fixture_urdf(), encoding="utf-8")
            controllers = share / "6-3_robot_sport" / "config" / "lite3_controllers.yaml"
            controllers.parent.mkdir(parents=True)
            controllers.write_text("{}\n", encoding="utf-8")
            gz_prefix = base / "gz_ros2_control"
            plugin = gz_prefix / "lib" / "libgz_ros2_control-system.so"
            plugin.parent.mkdir(parents=True)
            plugin.touch()

            ament = types.ModuleType("ament_index_python")
            packages = types.ModuleType("ament_index_python.packages")

            def get_share(package):
                self.assertEqual(package, "simulationmode_grazebo")
                return str(share)

            def get_prefix(package):
                self.assertEqual(package, "gz_ros2_control")
                return str(gz_prefix)

            packages.get_package_share_directory = get_share
            packages.get_package_prefix = get_prefix
            launch = types.ModuleType("launch")
            launch.LaunchDescription = StubLaunchDescription
            launch_actions = types.ModuleType("launch.actions")
            launch_actions.DeclareLaunchArgument = StubAction
            launch_actions.OpaqueFunction = StubAction
            substitutions = types.ModuleType("launch.substitutions")
            substitutions.LaunchConfiguration = StubLaunchConfiguration
            launch_ros = types.ModuleType("launch_ros")
            actions = types.ModuleType("launch_ros.actions")
            actions.Node = StubNode
            stubs = {
                "ament_index_python": ament,
                "ament_index_python.packages": packages,
                "launch": launch,
                "launch.actions": launch_actions,
                "launch.substitutions": substitutions,
                "launch_ros": launch_ros,
                "launch_ros.actions": actions,
            }
            spec = importlib.util.spec_from_file_location("spawn_model_under_test", LAUNCH_FILE)
            self.assertIsNotNone(spec)
            self.assertIsNotNone(spec.loader)
            module = importlib.util.module_from_spec(spec)
            with patch.dict(sys.modules, stubs):
                spec.loader.exec_module(module)
                actions = module.make_spawn(pin_body=pin_body)
        return module, actions

    def test_generated_imu_control_sensor_and_pose_bridge(self):
        _, actions = self.generate()
        self.assertEqual(len(actions), 3)
        nodes = {action.kwargs["package"]: action.kwargs
                 for action in actions}
        self.assertEqual(set(nodes), {"ros_gz_bridge", "robot_state_publisher",
                                      "ros_gz_sim"})
        robot_description = nodes["robot_state_publisher"]["parameters"][0]["robot_description"]
        self.assertEqual(
            nodes["ros_gz_sim"]["parameters"][0]["robot_description"],
            robot_description,
        )
        root = ET.fromstring(robot_description)

        control = root.find("./ros2_control")
        self.assertIsNotNone(control)
        self.assertEqual(len(control.findall("joint")), 12)
        self.assertEqual(len(root.findall("joint")), 12)
        self.assertIsNone(root.find("./gazebo/joint[@name='lite3_calibration_world_fixed']"))
        create_arguments = nodes["ros_gz_sim"]["arguments"]
        self.assertAlmostEqual(float(create_arguments[create_arguments.index('-z')+1]), 0.40)
        control_sensors = control.findall("sensor")
        self.assertEqual(len(control_sensors), 1)
        self.assertEqual(control_sensors[0].get("name"), "lite3_imu")
        actual_interfaces = [item.get("name") for item in
                             control_sensors[0].findall("state_interface")]
        self.assertEqual(len(actual_interfaces), 10)
        self.assertEqual(set(actual_interfaces), set(EXPECTED_IMU_INTERFACES))

        gazebo_imu = root.findall("./gazebo/sensor[@name='lite3_imu']")
        self.assertEqual(len(gazebo_imu), 1)
        self.assertEqual(gazebo_imu[0].get("type"), "imu")
        self.assertEqual(gazebo_imu[0].findtext("topic"), "/lite3_sim/imu")
        self.assertEqual(len(root.findall(
            "./gazebo[@reference='TORSO']/sensor[@name='lite3_imu']")), 1)
        self.assertEqual(len(root.findall(".//gazebo/sensor[@type='imu']")), 1)

        bridge = nodes["ros_gz_bridge"]
        pose_topic = "/model/lite3_control/pose"
        self.assertIn(pose_topic + "@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                      bridge["arguments"])
        self.assertIn((pose_topic, "/lite3_sim/model_pose"), bridge["remappings"])
        self.assertNotIn("/world/lite3_world/model/lite3_control/pose",
                         " ".join(bridge["arguments"]))

    def test_pinned_body_preserves_leg_control_and_ground_standing_height(self):
        _, actions = self.generate(pin_body=True)
        nodes = {action.kwargs["package"]: action.kwargs for action in actions}
        parameters = nodes['robot_state_publisher']['parameters'][0]
        root = ET.fromstring(parameters['robot_description'])
        pin = root.find("./gazebo/joint[@name='lite3_calibration_world_fixed']")
        self.assertIsNotNone(pin)
        self.assertEqual(pin.get('type'), 'fixed')
        self.assertEqual(pin.findtext('parent'), 'world')
        self.assertEqual(pin.findtext('child'), 'TORSO')
        self.assertEqual(len(root.findall('./gazebo/joint')), 1)
        self.assertEqual(len(root.findall('./ros2_control/joint')), 12)
        self.assertEqual(len(root.findall('./joint')), 12)
        self.assertEqual(root.findall('./link')[0].get('name'), 'TORSO')
        self.assertIsNone(root.find("./link[@name='world']"))
        self.assertFalse(any(item.text.strip().lower() == 'true'
                             for item in root.findall('.//static') if item.text))
        self.assertEqual(len(root.findall("./gazebo[@reference='TORSO']/sensor[@name='lite3_imu']")), 1)
        create_arguments = nodes['ros_gz_sim']['arguments']
        height = float(create_arguments[create_arguments.index('-z')+1])
        expected = (0.20+0.21012)*math.cos(0.65)+0.022
        self.assertAlmostEqual(height, expected, places=12)
        self.assertLess(height, 0.36)
        self.assertEqual(create_arguments[create_arguments.index('-name')+1], 'lite3_control')

    def test_launch_pin_argument_defaults_to_free_body(self):
        module, _ = self.generate()
        description = module.generate_launch_description()
        self.assertIsInstance(description, StubLaunchDescription)
        self.assertEqual(len(description.actions), 2)
        declaration, factory = description.actions
        self.assertEqual(declaration.args, ('pin_body',))
        self.assertEqual(declaration.kwargs['default_value'], 'false')
        self.assertEqual(declaration.kwargs['choices'], ['true', 'false'])
        self.assertEqual(factory.kwargs['function'], module._spawn_from_context)

    def test_invalid_pin_argument_cannot_silently_change_physics_mode(self):
        module, _ = self.generate()
        with self.assertRaisesRegex(RuntimeError, 'pin_body must be true or false'):
            module._spawn_from_context({'pin_body': 'yes'})
        with self.assertRaisesRegex(ValueError, 'pin_body must be a boolean'):
            module.make_spawn(pin_body='false')


if __name__ == "__main__":
    unittest.main()

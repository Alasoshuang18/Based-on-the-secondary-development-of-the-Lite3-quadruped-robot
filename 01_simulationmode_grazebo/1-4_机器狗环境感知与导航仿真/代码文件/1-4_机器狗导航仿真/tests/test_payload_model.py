"""Offline COM checks for the navigation attachments, without ROS or a robot mesh.

These analytical fixtures exercise the existing 6-3 mass parser against the
actual 6-4 attachment builder.  They do not qualify contact dynamics or walking.
"""
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
CONTROL_SRC = next(path for path in (
    ROOT.parent / '6-3_robot_sport/Lite3_only_Fortress_control/src',
    ROOT.parent / '6-3_robot_sport/src',
) if (path / 'lite3_measured_core.py').is_file())
sys.path.insert(0, str(CONTROL_SRC))
from lite3_gait_core import JOINTS, STAND
from lite3_measured_core import URDFCenterOfMass

spec = importlib.util.spec_from_file_location(
    'payload_test_attachments', ROOT / 'launch/navigation_attachments.py')
attachments = importlib.util.module_from_spec(spec)
spec.loader.exec_module(attachments)


def link(root, name, mass, xyz='0 0 0'):
    element = ET.SubElement(root, 'link', name=name)
    inertial = ET.SubElement(element, 'inertial')
    ET.SubElement(inertial, 'origin', xyz=xyz, rpy='0 0 0')
    ET.SubElement(inertial, 'mass', value=str(mass))
    return element


def fixture():
    root = ET.Element('robot', name='analytical_mass_fixture')
    link(root, 'TORSO', 10)
    link(root, 'leg_fixture', 2, '.1 0 -.2')
    joint = ET.SubElement(root, 'joint', name=JOINTS[0], type='revolute')
    ET.SubElement(joint, 'parent', link='TORSO')
    ET.SubElement(joint, 'child', link='leg_fixture')
    ET.SubElement(joint, 'origin', xyz='.2 0 0', rpy='0 0 0')
    ET.SubElement(joint, 'axis', xyz='0 0 1')
    return root


class PayloadMassTests(unittest.TestCase):
    def mass_model(self, root):
        # The production parser accepts a path.  Keep fixtures in a temporary
        # directory; never touch a generated robot or the 6-3 source tree.
        with tempfile.TemporaryDirectory(prefix='lite3_com_test_') as directory:
            path = Path(directory) / 'fixture.urdf'
            ET.ElementTree(root).write(path, encoding='utf-8')
            return URDFCenterOfMass(path)

    def assert_vector_close(self, actual, expected):
        for value, wanted in zip(actual, expected):
            self.assertAlmostEqual(value, wanted, places=11)

    def test_base_articulated_mass_has_analytical_com(self):
        model = self.mass_model(fixture())
        self.assertAlmostEqual(model.total_mass, 12)
        self.assert_vector_close(model.body_com(STAND), (.6/12, 0, -.4/12))

    def test_navigation_payload_and_lidar_are_both_mass_weighted(self):
        root = attachments.attach(fixture())
        model = self.mass_model(root)
        total = 12 + 1.5071784 + .265
        expected = ((.6 - .04*1.5071784 + .22*.265)/total,
                    0, (-.4 + .101*1.5071784 + .145*.265)/total)
        self.assertAlmostEqual(model.total_mass, total)
        self.assert_vector_close(model.body_com(STAND), expected)

    def test_sensor_disabled_does_not_remove_physical_lidar_mass(self):
        enabled = self.mass_model(attachments.attach(fixture(), enable_lidar=True))
        disabled = self.mass_model(attachments.attach(fixture(), enable_lidar=False))
        self.assertEqual(enabled.total_mass, disabled.total_mass)
        self.assert_vector_close(enabled.body_com(STAND), disabled.body_com(STAND))

    def test_payload_disabled_removes_only_box_mass(self):
        model = self.mass_model(attachments.attach(fixture(), with_payload=False))
        total = 12 + .265
        self.assertAlmostEqual(model.total_mass, total)
        self.assert_vector_close(model.body_com(STAND),
                                 ((.6+.22*.265)/total, 0, (-.4+.145*.265)/total))

    def test_changed_payload_mass_changes_com_using_same_mount(self):
        light = self.mass_model(attachments.attach(fixture(), payload_mass=.6))
        heavy = self.mass_model(attachments.attach(fixture(), payload_mass=3.0))
        self.assertAlmostEqual(heavy.total_mass-light.total_mass, 2.4)
        for model, mass in ((light, .6), (heavy, 3.0)):
            total = 12 + mass + .265
            self.assert_vector_close(model.body_com(STAND),
                ((.6-.04*mass+.22*.265)/total, 0, (-.4+.101*mass+.145*.265)/total))
        self.assertGreater(heavy.body_com(STAND)[2], light.body_com(STAND)[2])

    def test_articulation_changes_leg_com_but_fixed_mounts_stay_rigid(self):
        model = self.mass_model(attachments.attach(fixture()))
        turned = list(STAND)
        turned[0] = math.pi/2
        total = model.total_mass
        before, after = model.body_com(STAND), model.body_com(turned)
        # The leg's mass centre moves (.3,0,-.2) -> (.2,.1,-.2).
        # Both attachment contributions remain fixed in TORSO.
        self.assert_vector_close(tuple(b-a for a, b in zip(before, after)),
                                 (-.2/total, .2/total, 0))

    def test_fixed_mount_rotation_applies_to_local_inertial_origin(self):
        root = ET.Element('robot', name='rotated_fixed_mass_fixture')
        link(root, 'TORSO', 10)
        link(root, 'rotated_payload', 2, '.05 0 0')
        joint = ET.SubElement(root, 'joint', name='fixture_mount', type='fixed')
        ET.SubElement(joint, 'parent', link='TORSO')
        ET.SubElement(joint, 'child', link='rotated_payload')
        ET.SubElement(joint, 'origin', xyz='-.04 0 .101', rpy=f'0 0 {math.pi/2}')
        model = self.mass_model(root)
        self.assert_vector_close(model.body_com(STAND), (-.08/12, .1/12, .202/12))


if __name__ == '__main__':
    unittest.main()

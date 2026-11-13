import importlib.util
import math
from pathlib import Path
import struct
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('attachments',ROOT/'launch/navigation_attachments.py')
attachments = importlib.util.module_from_spec(spec)
spec.loader.exec_module(attachments)


class NavigationTests(unittest.TestCase):
    def model(self, **kwargs):
        root = ET.Element('robot',name='Lite3')
        ET.SubElement(root,'link',name='TORSO')
        return attachments.attach(root,**kwargs)

    def test_original_box_is_centered_in_metres(self):
        data = (ROOT/'meshes/box.stl').read_bytes()
        n = struct.unpack_from('<I',data,80)[0]
        self.assertEqual(n,12)
        points=[]
        for i in range(n):
            row=struct.unpack_from('<12fH',data,84+50*i)
            points.extend((row[3:6],row[6:9],row[9:12]))
        for axis,size in enumerate((.206,.1407,.052)):
            self.assertAlmostEqual(min(p[axis] for p in points),-size/2,places=7)
            self.assertAlmostEqual(max(p[axis] for p in points),size/2,places=7)

    def test_mounts_are_clear_of_torso_and_each_other(self):
        root=self.model()
        self.assertEqual(root.find("joint[@name='mid360_mount']/parent").get('link'),'TORSO')
        self.assertGreater(.101-.052/2,.07)
        self.assertGreater(.22-.065/2,-.04+.206/2)
        self.assertGreater(.145+.03,.101+.052/2)
        self.assertEqual(root.find("joint[@name='navigation_box_mount']").get('type'),'fixed')

    def test_fortress_sensor_and_low_load_profile(self):
        root=self.model()
        sensor=root.find(".//sensor[@name='mid360_sensor']")
        self.assertEqual(sensor.get('type'),'gpu_lidar')
        self.assertEqual(sensor.findtext('update_rate'),'5.0')
        self.assertEqual(sensor.findtext('ray/scan/horizontal/samples'),'180')
        self.assertAlmostEqual(float(sensor.findtext('ray/scan/vertical/min_angle')),math.radians(-7))
        self.assertFalse(any('livox' in p.get('filename','') for p in root.findall('.//plugin')))

    def test_optional_sensor_can_be_disabled(self):
        root=self.model(enable_lidar=False,with_payload=False)
        self.assertIsNone(root.find('.//sensor'))
        self.assertIsNone(root.find("link[@name='navigation_box']"))

    def test_invalid_profiles_rejected(self):
        for args in ({'payload_mass':0},{'payload_mass':float('nan')},{'lidar_hz':100}, {'horizontal':10000}):
            with self.assertRaises(ValueError): self.model(**args)


if __name__ == '__main__': unittest.main()

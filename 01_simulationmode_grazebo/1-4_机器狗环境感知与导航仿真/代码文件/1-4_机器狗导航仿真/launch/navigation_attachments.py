"""Fortress-native MID-360 FOV approximation and genuine CAD payload mesh.

No Classic plugin, hardware driver, CSV scan-pattern or Livox CustomMsg is used.
All new resources and generated descriptions stay in the navigation directory.
"""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def inertial(link, mass, dimensions):
    node = ET.SubElement(link, 'inertial')
    ET.SubElement(node, 'origin', xyz='0 0 0', rpy='0 0 0')
    ET.SubElement(node, 'mass', value=str(mass))
    x, y, z = dimensions
    ET.SubElement(node, 'inertia', ixx=str(mass*(y*y+z*z)/12), ixy='0', ixz='0',
                  iyy=str(mass*(x*x+z*z)/12), iyz='0', izz=str(mass*(x*x+y*y)/12))


def fixed(root, name, child, xyz):
    joint = ET.SubElement(root, 'joint', name=name, type='fixed')
    ET.SubElement(joint, 'parent', link='TORSO')
    ET.SubElement(joint, 'child', link=child)
    ET.SubElement(joint, 'origin', xyz=xyz, rpy='0 0 0')
    gazebo = ET.SubElement(root, 'gazebo', reference=name)
    ET.SubElement(gazebo, 'preserveFixedJoint').text = 'true'


def add_payload(root, mass):
    meta = json.loads((ROOT/'cad/box_export.json').read_text(encoding='utf-8-sig'))
    dimensions = meta['dimensions_m']
    link = ET.SubElement(root, 'link', name='navigation_box')
    inertial(link, mass, dimensions)
    visual = ET.SubElement(link, 'visual', name='cad_box_visual')
    ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh',
                  filename=(ROOT/'meshes/box.stl').as_uri(), scale='1 1 1')
    color = ET.SubElement(visual, 'material', name='navigation_box_blue')
    ET.SubElement(color, 'color', rgba='0.10 0.30 0.65 1')
    collision = ET.SubElement(link, 'collision', name='cad_box_collision')
    ET.SubElement(ET.SubElement(collision, 'geometry'), 'box',
                  size=' '.join(map(str, dimensions)))
    # Torso collision top z=.07; bottom of box z=.075: 5 mm clearance.
    fixed(root, 'navigation_box_mount', 'navigation_box', '-0.04 0 0.101')


def add_lidar(root, enabled, hz, horizontal, vertical):
    link = ET.SubElement(root, 'link', name='mid360_link')
    inertial(link, .265, (.065, .065, .060))
    for kind in ('visual', 'collision'):
        node = ET.SubElement(link, kind, name='mid360_housing_'+kind)
        ET.SubElement(ET.SubElement(node, 'geometry'), 'box', size='.065 .065 .060')
        if kind == 'visual':
            material = ET.SubElement(node, 'material', name='mid360_housing_black')
            ET.SubElement(material, 'color', rgba='.08 .08 .08 1')
    fixed(root, 'mid360_mount', 'mid360_link', '0.22 0 0.145')
    # Optical frame is on top of the housing, above the payload and torso.
    optical = ET.SubElement(root, 'link', name='mid360_optical_frame')
    joint = ET.SubElement(root, 'joint', name='mid360_optical_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='mid360_link')
    ET.SubElement(joint, 'child', link='mid360_optical_frame')
    ET.SubElement(joint, 'origin', xyz='0 0 .030', rpy='0 0 0')
    if not enabled:
        return
    gazebo = ET.SubElement(root, 'gazebo', reference='mid360_link')
    sensor = ET.SubElement(gazebo, 'sensor', name='mid360_sensor', type='gpu_lidar')
    for tag, text in [('pose','0 0 .030 0 0 0'), ('always_on','true'),
                      ('visualize','false'), ('update_rate',str(hz)),
                      ('topic','/lite3_sim/mid360/scan')]:
        ET.SubElement(sensor, tag).text = text
    lidar = ET.SubElement(sensor, 'ray')
    scan = ET.SubElement(lidar, 'scan')
    for tag, count, lo, hi in [('horizontal',horizontal,-math.pi,math.pi),
                                ('vertical',vertical,math.radians(-7),math.radians(52))]:
        axis = ET.SubElement(scan, tag)
        for name, value in [('samples',count),('resolution',1),('min_angle',lo),('max_angle',hi)]:
            ET.SubElement(axis, name).text = str(value)
    ranges = ET.SubElement(lidar, 'range')
    for name, value in [('min',.1),('max',40.),('resolution',.01)]:
        ET.SubElement(ranges, name).text = str(value)


def attach(root, payload_mass=1.5071784, with_payload=True, enable_lidar=True,
           lidar_hz=5., horizontal=180, vertical=12):
    if root.find("link[@name='TORSO']") is None:
        raise ValueError('Expected existing Lite3 TORSO link')
    if not math.isfinite(payload_mass) or not 0 < payload_mass <= 5:
        raise ValueError('payload_mass must be in (0,5] kg; verify actual mass')
    if not 1 <= lidar_hz <= 10 or not 32 <= horizontal <= 720 or not 2 <= vertical <= 32:
        raise ValueError('Lidar settings outside the bounded navigation profile')
    if with_payload:
        add_payload(root, payload_mass)
    add_lidar(root, enable_lidar, lidar_hz, horizontal, vertical)
    return root

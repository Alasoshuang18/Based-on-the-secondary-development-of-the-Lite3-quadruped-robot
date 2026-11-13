#!/usr/bin/python3
"""Copy the existing scene and add obstacle contact sensors, without moving geometry."""
from pathlib import Path
import argparse
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--real-time-factor", type=float, default=.5)
    args = parser.parse_args()
    if not .1 <= args.real_time_factor <= 1.:
        parser.error("Use a bounded 0.1 to 1.0 real-time target")
    root = Path(__file__).resolve().parent
    original = root/'worlds/lite3_navigation.sdf'
    tree = ET.parse(original)
    world = tree.getroot().find('world')
    world.find('physics/real_time_factor').text = str(args.real_time_factor)
    for side in ('front', 'left'):
        link = world.find(f"model[@name='lidar_target_{side}']/link")
        if link is None:
            raise ValueError('Existing scene lacks expected obstacle: '+side)
        collision = link.find('collision').get('name')
        sensor = ET.SubElement(link, 'sensor', name='avoidance_contact_'+side, type='contact')
        ET.SubElement(sensor, 'always_on').text = 'true'
        ET.SubElement(sensor, 'update_rate').text = '100'
        topic = '/lite3_sim/obstacle_contacts/'+side
        ET.SubElement(sensor, 'topic').text = topic
        contact = ET.SubElement(sensor, 'contact')
        ET.SubElement(contact, 'collision').text = collision
        ET.SubElement(contact, 'topic').text = topic
    target = root/'worlds/lite3_navigation_guarded.sdf'
    ET.indent(tree, space='  ')
    tree.write(target, encoding='utf-8', xml_declaration=True)
    print(target)


if __name__ == '__main__':
    main()

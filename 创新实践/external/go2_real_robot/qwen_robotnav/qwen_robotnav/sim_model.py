"""Scoped corrections for a generated simulation model, never shared source files."""

import xml.etree.ElementTree as ET


def horizontal_lidar_description(description, sensor_name='velodyne'):
    """A single vertical sample must use zero elevation for a 2-D LaserScan."""
    root=ET.fromstring(description)
    for sensor in root.findall('.//sensor'):
        if sensor.get('name')!=sensor_name:
            continue
        vertical=sensor.find('lidar/scan/vertical')
        if vertical is None:
            vertical=sensor.find('ray/scan/vertical')
        if vertical is None or vertical.findtext('samples','').strip()!='1':
            continue
        for name in ('min_angle','max_angle'):
            element=vertical.find(name)
            if element is None:element=ET.SubElement(vertical,name)
            element.text='0.0'
    return ET.tostring(root,encoding='unicode')

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


def rgbd_optical_description(description):
    """Label RGB-D pixel rays with the optical frame, not the body-axis link."""
    root = ET.fromstring(description)
    sensor = root.find(".//sensor[@name='realsense_d435']")
    if sensor is None:
        raise ValueError('Expected simulator RGB-D sensor is missing')
    frame = sensor.find('camera/optical_frame_id')
    if frame is None:
        raise ValueError('Expected RGB-D optical frame field is missing')
    frame.text = 'camera_d435_optical'
    if root.find("./link[@name='camera_d435_optical']") is None:
        ET.SubElement(root, 'link', name='camera_d435_optical')
        joint = ET.SubElement(root, 'joint', name='camera_d435_optical_joint', type='fixed')
        ET.SubElement(joint, 'parent', link='camera_d435')
        ET.SubElement(joint, 'child', link='camera_d435_optical')
        ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='-1.5707963267948966 0 -1.5707963267948966')
    return ET.tostring(root, encoding='unicode')

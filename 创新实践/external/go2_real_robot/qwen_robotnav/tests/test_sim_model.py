import xml.etree.ElementTree as ET
from qwen_robotnav.sim_model import horizontal_lidar_description


def test_only_corrects_selected_single_line_sensor_elevation():
    xml='''<robot><gazebo><sensor name="velodyne"><lidar><scan><horizontal><samples>360</samples></horizontal><vertical><samples>1</samples><min_angle>-0.2618</min_angle><max_angle>0.2618</max_angle></vertical></scan></lidar></sensor><sensor name="other"><lidar><scan><vertical><samples>1</samples><min_angle>-0.2</min_angle></vertical></scan></lidar></sensor></gazebo></robot>'''
    root=ET.fromstring(horizontal_lidar_description(xml))
    sensor=root.find('.//sensor[@name="velodyne"]')
    assert sensor.findtext('lidar/scan/vertical/min_angle')=='0.0'
    assert sensor.findtext('lidar/scan/vertical/max_angle')=='0.0'
    assert sensor.findtext('lidar/scan/horizontal/samples')=='360'
    assert root.find('.//sensor[@name="other"]').findtext('lidar/scan/vertical/min_angle')=='-0.2'


def test_multibeam_sensor_is_preserved():
    xml='<robot><sensor name="velodyne"><lidar><scan><vertical><samples>16</samples><min_angle>-0.2</min_angle><max_angle>0.2</max_angle></vertical></scan></lidar></sensor></robot>'
    root=ET.fromstring(horizontal_lidar_description(xml))
    assert root.findtext('.//vertical/min_angle')=='-0.2'


def test_rgbd_optical_frame_rotates_pixel_forward_into_robot_forward():
    import math
    import numpy as np
    from qwen_robotnav.sim_model import rgbd_optical_description
    xml='<robot><link name="camera_d435"/><gazebo><sensor name="realsense_d435"><camera><optical_frame_id>camera_d435</optical_frame_id></camera></sensor></gazebo></robot>'
    root=ET.fromstring(rgbd_optical_description(xml))
    assert root.findtext('.//optical_frame_id')=='camera_d435_optical'
    rpy=[float(v) for v in root.find('./joint/origin').get('rpy').split()]
    roll,pitch,yaw=rpy
    rx=np.array([[1,0,0],[0,math.cos(roll),-math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
    rz=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    assert np.allclose(rz@rx@np.array([0,0,1]),[1,0,0])
    assert np.allclose(rz@rx@np.array([1,0,0]),[0,-1,0])

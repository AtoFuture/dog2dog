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

"""Check the actual Nav2 saver/reloader preserves unknown cells in trinary maps."""
import signal
import shutil
import subprocess
import time
import uuid

import pytest

rclpy=pytest.importorskip('rclpy')
pytest.importorskip('nav_msgs')
from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
from g3_integration_eval.coverage import calculate_coverage, load_map


def test_nav2_trinary_export_preserves_unknown_cells(tmp_path):
    try:get_package_prefix('nav2_map_server')
    except PackageNotFoundError:pytest.skip('Nav2 map saver is not installed')
    if not shutil.which('ros2'):pytest.skip('ROS CLI is not on PATH')
    rclpy.init(args=[])
    node=rclpy.create_node('map_export_test_'+uuid.uuid4().hex[:8])
    topic='/map_export_test_'+uuid.uuid4().hex
    pub=node.create_publisher(OccupancyGrid,topic,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    msg=OccupancyGrid();msg.header.frame_id='map';msg.info.width=2;msg.info.height=2
    msg.info.resolution=.05;msg.info.origin.orientation.w=1.;msg.data=[-1,0,100,-1]
    pub.publish(msg)
    prefix=tmp_path/'saved'
    child=subprocess.Popen(['ros2','run','nav2_map_server','map_saver_cli','-f',str(prefix),'-t',topic,
        '--ros-args','-p','map_subscribe_transient_local:=true','-p','free_thresh_default:=0.196'],
        stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)
    try:
        deadline=time.monotonic()+10
        while child.poll() is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        assert child.poll()==0
        saved=load_map(prefix.with_suffix('.yaml'))
        assert saved.free_thresh==pytest.approx(.196)
        unknown=sum(saved.free_thresh <= 1-v/255 <= saved.occupied_thresh for v in saved.data)
        assert unknown==2
        assert calculate_coverage(saved.yaml_path,saved.yaml_path).reference_free_cells==1
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        node.destroy_node();rclpy.shutdown()

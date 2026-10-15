"""Undistorted cameras can legally send an empty distortion vector."""
from types import SimpleNamespace
from sensor_msgs.msg import CameraInfo
from vision_detector.detector_node import VisionDetector


def test_camera_info_without_distortion_coefficients_does_not_crash():
    node=VisionDetector.__new__(VisionDetector)
    node._intrinsics=None
    node.get_logger=lambda:SimpleNamespace(info=lambda *a:None,warn=lambda *a:None)
    msg=CameraInfo();msg.k=[554.,0.,320.,0.,554.,240.,0.,0.,1.]
    assert len(msg.d)==0
    node._on_camera_info(msg)
    assert node._intrinsics.fx==554.

"""Positive ROS contract check: real YOLO RGB inference, synthetic depth and TF.

This is NOT a real RGB-D accuracy or warehouse-person detection experiment.
Only bundled Ultralytics assets and existing local weights are used.
"""
import argparse
import json
import math
import os
from pathlib import Path
import time
import subprocess
import signal
import sys

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from vision_interfaces.msg import Detection3D

from g3_integration_eval.detection_evidence_node import DetectionEvidenceNode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--expected-domain',type=int,required=True)
    args = parser.parse_args()
    if args.expected_domain in (0,42,43) or int(os.environ.get('ROS_DOMAIN_ID','0'))!=args.expected_domain:
        parser.error('Choose a matching isolated ROS domain')
    root=args.output_dir.resolve()
    if (root/'detections/evidence.sqlite3').exists():
        parser.error('Choose a new output directory to avoid reusing old evidence')
    root.mkdir(parents=True,exist_ok=True)
    project=Path(__file__).resolve().parents[4]
    import ultralytics
    photo=Path(ultralytics.__file__).parent/'assets/bus.jpg'
    image=cv2.imread(str(photo))
    if image is None:raise RuntimeError('Bundled YOLO validation photograph unavailable')
    image=cv2.resize(image,(640,480))
    ros_args=['--ros-args','-p','weights:='+str(project/'baseline/g2_vision_explore/yolo26n.pt'),
        '-p','evidence_output_dir:='+str(root/'detections'),'-p','tf_fallback_to_latest:=false',
        '-p','detect_hz:=3.0','-p','depth_topic:=/camera/aligned_depth_to_color/image_raw']
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    node=Node('detection_replay_fixture');collector=DetectionEvidenceNode()
    child_env=dict(os.environ,OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',OPENBLAS_NUM_THREADS='2')
    log=(root/'detector.log').open('w')
    detector=subprocess.Popen([sys.executable,'-m','vision_detector.detector_node',*ros_args],
        stdout=log,stderr=subprocess.STDOUT,env=child_env,start_new_session=True)
    broadcaster=StaticTransformBroadcaster(node)
    tf=TransformStamped();tf.header.stamp=node.get_clock().now().to_msg()
    tf.header.frame_id='map';tf.child_frame_id='replay_camera_optical'
    tf.transform.translation.z=.3
    # Rz(-pi/2) Rx(-pi/2): optical forward maps to map +X.
    tf.transform.rotation.x=-.5;tf.transform.rotation.y=.5
    tf.transform.rotation.z=-.5;tf.transform.rotation.w=.5
    broadcaster.sendTransform(tf)
    color_pub=node.create_publisher(Image,'/camera/color/image_raw',qos_profile_sensor_data)
    depth_pub=node.create_publisher(Image,'/camera/aligned_depth_to_color/image_raw',qos_profile_sensor_data)
    info_pub=node.create_publisher(CameraInfo,'/camera/color/camera_info',qos_profile_sensor_data)
    observations=[];evidence=[];frames=[0]
    node.create_subscription(Detection3D,'/detections_3d',lambda m:observations.append(dict(
        class_name=m.class_name,track_id=m.id,position_m=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z])),50)
    node.create_subscription(String,'/mission/detection_evidence',lambda m:evidence.append(json.loads(m.data)),50)
    def publish():
        stamp=node.get_clock().now().to_msg()
        info=CameraInfo();info.header.stamp=stamp;info.header.frame_id='replay_camera_optical'
        info.width=640;info.height=480;info.k=[554.25469,0.,320.5,0.,554.25469,240.5,0.,0.,1.]
        info_pub.publish(info)
        color=Image();color.width=640;color.height=480;color.encoding='bgr8';color.step=640*3;color.data=image.tobytes()
        depth=Image();depth.width=640;depth.height=480;depth.encoding='32FC1';depth.step=640*4
        depth.data=np.full((480,640),2.,dtype=np.float32).tobytes()
        for msg in (color,depth):msg.header=info.header
        color_pub.publish(color);depth_pub.publish(depth);frames[0]+=1
    timer=node.create_timer(.3,publish)
    executor=SingleThreadedExecutor()
    for item in (node,collector):executor.add_node(item)
    started=time.monotonic();success=False;error=None
    try:
        end=started+60
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.1)
            if detector.poll() is not None:
                raise RuntimeError('Detector exited; inspect detector.log')
            if evidence and evidence[-1]['ready_evidence_ids'] and observations:
                success=any(o['class_name'] in ('person','bus') for o in observations) and all(
                    all(math.isfinite(v) for v in o['position_m']) and abs(o['position_m'][0]-2.)<1e-4 for o in observations)
                break
    except Exception as exc:
        import traceback
        traceback.print_exc()
        error=type(exc).__name__+": "+str(exc)
    finally:
        timer.cancel()
        executor.shutdown(timeout_sec=5)
        os.killpg(detector.pid,signal.SIGINT) if detector.poll() is None else None
        try:detector.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(detector.pid,signal.SIGTERM);detector.wait(timeout=5)
        log.close()
        stats=None
        for item in (collector,node):item.destroy_node()
        rclpy.shutdown()
    report=dict(success=success,error=error,validation_type='RGB photo + synthetic depth/TF ROS contract check',
        image_asset=str(photo),depth_source='synthetic constant 2m plane',
        tf_source='synthetic static optical-to-map transform',real_rgbd_accuracy_validated=False,
        live_warehouse_target_validated=False,frames_published=frames[0],detector_stats=stats,
        observations=observations,evidence=evidence[-1] if evidence else None,elapsed_seconds=time.monotonic()-started)
    (root/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    return 0 if success else 1


if __name__=='__main__':raise SystemExit(main())

"""Collect G2 Detection3D + snapshot messages into a reviewable local record."""
import json
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_interfaces.msg import Detection3D, DetectionSnapshot
from g3_integration_eval.detection_evidence import DetectionEvidenceStore
from g3_integration_eval.person_appearance import describe


def stamp_ns(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


class DetectionEvidenceNode(Node):
    def __init__(self):
        super().__init__('detection_evidence')
        self.declare_parameter('evidence_output_dir', '')
        directory = self.get_parameter('evidence_output_dir').value
        if not directory:
            raise ValueError('Set evidence_output_dir to this mission run directory')
        self.store = DetectionEvidenceStore(directory)
        self.publisher = self.create_publisher(String, '/mission/detection_evidence', 10)
        self.create_subscription(Detection3D, '/detections_3d', self._detection, 50)
        self.create_subscription(DetectionSnapshot, '/detections_snapshot', self._snapshot, 50)
        self._publish()

    def _publish(self):
        self.publisher.publish(String(data=json.dumps(self.store.export(), ensure_ascii=False)))

    def _detection(self, msg):
        p = msg.pose.pose.position
        value = dict(track_id=msg.id, stamp_ns=stamp_ns(msg.stamp), source=msg.source,
                     class_name=msg.class_name, confidence=msg.confidence,
                     position_m=[p.x,p.y,p.z], has_snapshot=msg.has_snapshot,
                     onset_stamp_ns=stamp_ns(msg.onset_stamp))
        for name in ('torso_tilt_deg','torso_height_m'):
            value[name] = float(getattr(msg,name)) if math.isfinite(getattr(msg,name)) else None
        try:
            self.store.add_detection(value)
            self._publish()
        except ValueError as exc:
            self.get_logger().warning(f'Detection rejected: {exc}')

    def _snapshot(self, msg):
        try:
            data = bytes(msg.image.data)
            if len(data)>4*1024*1024:raise ValueError('JPEG too large')
            image=cv2.imdecode(np.frombuffer(data,dtype=np.uint8),cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('JPEG cannot be decoded')
            self.store.add_snapshot(msg.id, stamp_ns(msg.stamp), data, appearance=describe(image))
            self._publish()
        except (ValueError,cv2.error) as exc:
            self.get_logger().warning(f'Snapshot rejected: {exc}')

    def destroy_node(self):
        self.store.export()
        self.store.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DetectionEvidenceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()

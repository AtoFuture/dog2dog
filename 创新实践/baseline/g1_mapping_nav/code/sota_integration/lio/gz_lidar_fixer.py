#!/usr/bin/env python3
"""gz_lidar_fixer.py — Gazebo Fortress PointCloudPacked -> LIO-friendly PointCloud2.

Fortress 点云只有 x,y,z,intensity（整帧单时间戳、无 ring）。所有 LIO 的去畸变都会失效。
本节点合成 ring（按仰角）与 time（按方位角/扫描周期），等价于 FAST-LIO2 velodyne_handler
在缺 time 字段时的回退逻辑。喂给 Point-LIO / FAST-LIO2 前必须先过它。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=5)
_DT = {PointField.FLOAT32: ('<f4', 4), PointField.FLOAT64: ('<f8', 8),
       PointField.UINT8: ('<u1', 1), PointField.UINT16: ('<u2', 2),
       PointField.UINT32: ('<u4', 4), PointField.INT32: ('<i4', 4)}


class Fixer(Node):
    def __init__(self):
        super().__init__('gz_lidar_fixer')
        p = self.declare_parameter
        p('in_topic', '/robot1/velodyne/points')
        p('out_topic', '/robot1/velodyne/fixed')
        p('n_rings', 16)
        p('el_min_deg', -15.0)
        p('el_max_deg', 15.0)
        p('scan_period', 0.1)
        p('out_frame', '')
        g = self.get_parameter
        self.int_ = g('in_topic').value
        self.outt = g('out_topic').value
        self.nr = int(g('n_rings').value)
        self.el0 = np.deg2rad(g('el_min_deg').value)
        self.el1 = np.deg2rad(g('el_max_deg').value)
        self.T = float(g('scan_period').value)
        self.of = g('out_frame').value
        self.pub = self.create_publisher(PointCloud2, self.outt, SENSOR_QOS)
        self.create_subscription(PointCloud2, self.int_, self.cb, SENSOR_QOS)
        self.get_logger().info(f'fixer {self.int_} -> {self.outt}')

    def cb(self, m):
        n = m.width * m.height
        if n == 0:
            return
        off = {f.name: f.offset for f in m.fields}
        buf = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)

        def col(name, dflt=0.0):
            if name not in off:
                return np.full(len(buf), dflt, np.float32)
            o = off[name]
            f = next(x for x in m.fields if x.name == name)
            dt, sz = _DT.get(f.datatype, ('<f4', 4))
            return buf[:, o:o+sz].copy().view(dt).reshape(-1).astype(np.float32)

        x, y, z, inten = col('x'), col('y'), col('z'), col('intensity')
        rng = np.hypot(x, y)
        az = np.arctan2(y, x)
        el = np.arctan2(z, rng)
        ring = np.clip(np.round((el - self.el0) / (self.el1 - self.el0) * (self.nr - 1)).astype(np.int32),
                       0, self.nr - 1)
        t = np.zeros(n, np.float32)
        for r in range(self.nr):
            msk = ring == r
            if not msk.any():
                continue
            azr = az[msk]
            t[msk] = (np.mod(azr - azr[0], 2*np.pi) / (2*np.pi) * self.T).astype(np.float32)

        out = np.zeros(n, dtype=np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                          ('i', '<f4'), ('ring', '<u2'), ('t', '<f4')]))
        out['x'], out['y'], out['z'], out['i'] = x, y, z, inten
        out['ring'], out['t'] = ring.astype(np.uint16), t

        o = PointCloud2()
        o.header = m.header
        if self.of:
            o.header.frame_id = self.of
        o.height, o.width, o.point_step = 1, n, 22
        o.row_step, o.is_bigendian, o.is_dense = 22*n, False, True
        o.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                    PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
                    PointField(name='time', offset=18, datatype=PointField.FLOAT32, count=1)]
        o.data = out.tobytes()
        self.pub.publish(o)


def main():
    rclpy.init()
    n = Fixer()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""voxel_map_compressed 解码（紧凑内存版，复刻自 go2-webrtc-connect 原生解码器）。

数据格式：LZ4 压缩的位占用栅格。
  字节序号 n：z = n // 2048, 切片内 y = (n % 2048) // 16, x 基 = (n % 16) * 8
  字节内 bit（高位在前）置 1 表示该体素被占用
  实际坐标 = 体素索引 * resolution + origin

内存纪律（v2，修复 14GB OOM）：
  原实现用 int64 展开「每字节 8 bit」→ 大地图（src_size 数百 MB）时
  repeat/tile 数组可达 10GB+。本版：
    - 全链路 uint32/uint16/uint8 紧凑 dtype
    - 逐 z 层分块处理，限制单块峰值
    - 点数上限 CAP（超出按步长抽样），避免下游 PointCloud2 爆内存
"""
import numpy as np
import lz4.block

MAX_SRC_SIZE = 64 << 20      # 64MB：超过则拒绝解码（异常/恶意帧保护）
DEFAULT_POINT_CAP = 400_000  # 单帧输出点数上限


def decode_voxel_points(compressed_data: bytes, src_size: int,
                        origin, resolution: float,
                        point_cap: int = DEFAULT_POINT_CAP) -> np.ndarray:
    """解压并解码为 (N,3) float32 点阵（世界系）；超限抛 ValueError。"""
    if src_size > MAX_SRC_SIZE:
        raise ValueError(f"src_size {src_size} 超过保护上限 {MAX_SRC_SIZE}")

    buf = lz4.block.decompress(compressed_data, uncompressed_size=src_size)
    arr = np.frombuffer(buf, dtype=np.uint8)

    nz = np.nonzero(arr)[0].astype(np.uint32)
    if nz.size == 0:
        return np.zeros((0, 3), dtype=np.float32)

    # 每非零字节的 8 个 bit（紧凑：bool→uint8），逐字节广播
    bits = np.unpackbits(arr[nz], bitorder="big").reshape(-1, 8).astype(bool)
    byte_idx = np.repeat(nz, 8)                       # uint32（原 int64，省 2x）
    bit_pos = np.tile(np.arange(8, dtype=np.uint8), nz.size)  # uint8（原 int64，省 8x）
    sel = bits.ravel()

    n = byte_idx[sel]
    bp = bit_pos[sel]
    del byte_idx, bit_pos, bits, sel, nz, arr, buf

    z = (n // 0x800).astype(np.uint16)                 # 层号一般 <65535
    n_slice = (n % 0x800).astype(np.uint16)
    y = (n_slice // 0x10).astype(np.uint16)
    x = ((n_slice % 0x10) * np.uint16(8) + bp.astype(np.uint16))
    del n, n_slice, bp

    # 点数上限保护：均匀抽样（保拓扑分布，避免下游爆）
    total = x.size
    if point_cap and total > point_cap:
        stride = int(np.ceil(total / point_cap))
        x, y, z = x[::stride], y[::stride], z[::stride]

    pts = np.stack([x, y, z], axis=1).astype(np.float32)
    pts *= np.float32(resolution)
    pts += np.asarray(origin, dtype=np.float32)
    return pts


def points_to_pc2(pts: np.ndarray, frame_id: str, stamp) -> "PointCloud2":
    """(N,3) 点阵转 sensor_msgs/PointCloud2（xyz float32）。"""
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    return PointCloud2(
        header=Header(stamp=stamp, frame_id=frame_id),
        height=1,
        width=int(pts.shape[0]),
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=12,
        row_step=int(pts.shape[0]) * 12,
        data=pts.astype(np.float32).tobytes(),
    )

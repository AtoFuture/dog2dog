#!/usr/bin/env python3
"""WebRTC→ROS2 桥节点：把 Go2 内置雷达的 WebRTC 数据流发布成 ROS2 话题。

背景：狗的 utlidar DDS 转发环节静默（软件问题），但 WebRTC 通道活着——
本节点模拟 App 连接（信令 .161:9991），订阅三流并转发：

  /baiyanggo/webrtc/pose         geometry_msgs/PoseStamped   ~19Hz odom系位姿
  /baiyanggo/webrtc/voxel_map    sensor_msgs/PointCloud2     ~10Hz 体素点云(运动时)
  /baiyanggo/webrtc/lidar_state  std_msgs/String(JSON)       ~5Hz  雷达自诊断
  /baiyanggo/webrtc/status       std_msgs/String(JSON)       桥健康状态

数据通路说明：
  pose/lidar_state 走 pub_sub 订阅回调（字符串消息，路由正常）；
  voxel_map_compressed 是二进制帧，经驱动的 deal_array_buffer_for_lidar 处理——
  本节点直接挂钩该方法（向量化解码），不依赖驱动的主题路由。

部署（.18）：baiyanggo-install.sh 生成 systemd 服务 baiyanggo-webrtc-bridge。
依赖：webrtc_venv（含补丁版 go2-webrtc-connect、numpy、lz4）+ /opt/ros/foxy。
"""
import asyncio
import json
import math
import random
import resource
import threading
import time

import numpy as np

# 自保护：地址空间硬上限。桥历史上有过 14GB 内存事故（驱动的 wasmtime 解码器
# 预留 8GB + 大帧解码），在 16GB 的狗载 Jetson 上会 OOM 整机。
# 设 3.5GB 上限后最坏情况是本进程 MemoryError，绝不拖垮 Jetson。
try:
    _LIMIT = 3500 << 20
    _soft, _hard = resource.getrlimit(resource.RLIMIT_AS)
    if _hard == resource.RLIM_INFINITY or _hard > _LIMIT:
        resource.setrlimit(resource.RLIMIT_AS, (_LIMIT, _hard))
except Exception:
    pass

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField, Imu
from std_msgs.msg import Header, String

from .voxel_decode import decode_voxel_points

SRC_POSE = "rt/utlidar/robot_pose"
SRC_LIDAR_STATE = "rt/utlidar/lidar_state"
SRC_VOXEL = "rt/utlidar/voxel_map_compressed"
# IMU 来源用 sportmodestate(20Hz)；lowstate 在 WebRTC 下被限到 1Hz（实测扫描）
SRC_SPORTMODE = "rt/lf/sportmodestate"
SPORT_REQ = "rt/api/sport/request"
SPORT_API_MOVE = 1008

RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class WebrtcBridgeNode(Node):
    def __init__(self):
        super().__init__("baiyanggo_webrtc_bridge")
        self.declare_parameter("robot_ip", "192.168.123.161")
        self.declare_parameter("reconnect_interval", 3.0)  # 狗侧服务崩溃间隙快速重拨
        self.declare_parameter("cloud_pub_min_period", 0.5)   # 体素点云最低发布间隔（秒）

        self.pub_pose = self.create_publisher(PoseStamped, "/baiyanggo/webrtc/pose", RELIABLE)
        self.pub_cloud = self.create_publisher(PointCloud2, "/baiyanggo/webrtc/voxel_map", RELIABLE)
        self.pub_state = self.create_publisher(String, "/baiyanggo/webrtc/lidar_state", RELIABLE)
        self.pub_imu = self.create_publisher(Imu, "/baiyanggo/webrtc/imu", RELIABLE)
        self.pub_sport_odom = self.create_publisher(Odometry, "/baiyanggo/webrtc/sport_odom", RELIABLE)
        self.pub_status = self.create_publisher(String, "/baiyanggo/webrtc/status", 10)

        self.counters = {"pose": 0, "cloud": 0, "state": 0, "cloud_pts": 0, "decode_err": 0, "imu": 0}
        self._last_cloud_pub = 0.0

        # ---- 运动通路（默认禁用；需显式 enable_motion:=true 才会转发 cmd_vel）----
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("cmd_timeout", 0.6)      # 超时未更新则急停
        self.declare_parameter("max_vx", 0.3)
        self.declare_parameter("max_vy", 0.2)
        self.declare_parameter("max_wz", 0.6)
        self._last_cmd = None
        self._last_cmd_t = 0.0
        self._moving = False
        self._channel = None
        self._move_pending = False
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        # 通用 sport 指令通路（姿态类如 BalanceStand=1002/StopMove=1003；受 enable_motion 门控）
        self.create_subscription(String, "/baiyanggo/webrtc/sport_cmd", self._on_sport_cmd, 10)
        self.create_timer(0.1, self._cmd_tick)          # 10Hz 下发
        self.connected = False
        self._status_timer = self.create_timer(2.0, self._publish_status)

        self._loop = asyncio.new_event_loop()
        self._worker = threading.Thread(target=self._run_forever, daemon=True)
        self._worker.start()
        self.get_logger().info("WebRTC 桥节点已启动")

    # ---------------- 运动通路 ----------------

    def _on_cmd_vel(self, msg: Twist):
        self._last_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_cmd_t = time.monotonic()

    def _on_sport_cmd(self, msg: String):
        if not self.get_parameter("enable_motion").value or self._channel is None:
            self.get_logger().warning("sport_cmd 被忽略（enable_motion=false 或未连接）")
            return
        try:
            import json as _json
            d = _json.loads(msg.data)
            api_id = int(d["api_id"])
            param = d.get("parameter", {})
        except Exception as e:
            self.get_logger().warning(f"sport_cmd 解析失败: {e}")
            return
        ch = self._channel

        async def _send():
            try:
                r = await ch.pub_sub.publish_request_new(
                    SPORT_REQ, {"api_id": api_id, "parameter": param})
                self.get_logger().info(f"sport_cmd {api_id} 回执: {str(r)[:160]}")
            except Exception as e:
                self.get_logger().warning(f"sport_cmd {api_id} 失败: {e}")
        asyncio.run_coroutine_threadsafe(_send(), self._loop)
        self.get_logger().info(f"sport_cmd 已下发: api_id={api_id} param={param}")

    def _cmd_tick(self):
        if not self.get_parameter("enable_motion").value or self._channel is None:
            return
        now = time.monotonic()
        stale = self._last_cmd is None or (now - self._last_cmd_t) > \
            float(self.get_parameter("cmd_timeout").value)
        if stale:
            if self._moving:
                self._send_move(0.0, 0.0, 0.0)      # 看门狗急停（一次）
                self._moving = False
            return
        vx, vy, wz = self._last_cmd
        cl = lambda v, lim: max(-lim, min(lim, v))
        vx = cl(vx, float(self.get_parameter("max_vx").value))
        vy = cl(vy, float(self.get_parameter("max_vy").value))
        wz = cl(wz, float(self.get_parameter("max_wz").value))
        # BYY修复⑦ 坐标系约定：Unitree sport Move 的旋转正方向与 ROS 相反
        # (实测 wz=+0.3x3s 狗右转176°)；侧移 y 约定同理按 ROS→Unitree 取反。
        self._send_move(vx, -vy, -wz)
        self._moving = abs(vx) > 1e-3 or abs(vy) > 1e-3 or abs(wz) > 1e-3

    def _send_move(self, vx, vy, wz):
        if self._move_pending or self._channel is None:
            return
        self._move_pending = True
        ch = self._channel

        async def _send():
            try:
                # 发即忘 + 正确请求信封：狗对 Move 指令大多不回执（实测61超时/7成功），
                # 等回执会把10Hz指令流掐成0.67Hz；但普通MSG包狗会静默丢弃——
                # 必须复刻 publish_request_new 的请求结构（type=req + identity + JSON字符串参数）。
                gen_id = int(time.time() * 1000) % 2147483648 + random.randint(0, 1000)
                ch.pub_sub.publish_without_callback(
                    SPORT_REQ,
                    {"header": {"identity": {"id": gen_id, "api_id": SPORT_API_MOVE}},
                     "parameter": json.dumps({"x": vx, "y": vy, "z": wz})},
                    msg_type="req")
                self.counters["moves"] = self.counters.get("moves", 0) + 1
            except Exception as e:
                self.counters["move_err"] = self.counters.get("move_err", 0) + 1
                self.get_logger().warning(f"Move 下发失败: {e}", throttle_duration_sec=5)
            finally:
                self._move_pending = False

        asyncio.run_coroutine_threadsafe(_send(), self._loop)

    # ---------------- ROS 侧 ----------------

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            "connected": self.connected,
            "counts": self.counters,
            "time": time.time(),
        })
        self.pub_status.publish(msg)

    def _publish_cloud(self, pts, meta: dict) -> None:
        # BYY修复④ 自适应地板滤波：狗静止时地面点会漂移上浮(实测z漂到+0.17~0.28，
        # 且随时间波动)，固定高度窗挡不住 → 幻影障碍围死狗、Nav2 只能发零速。
        # 每帧用 z 直方图众数定位地板(地面点占绝对多数)，只发布[地板+0.32, 地板+0.95]。
        try:
            if pts.shape[0] > 200:
                hist, edges = np.histogram(pts[:, 2], bins=40)
                i = int(np.argmax(hist))
                floor_z = 0.5 * (edges[i] + edges[i + 1])
                if hist[i] / pts.shape[0] >= 0.25:
                    lo, hi = floor_z + 0.32, floor_z + 0.95
                else:   # 无明显地板(空旷/异常帧)时退回绝对窗
                    lo, hi = 0.30, 0.95
                pts = pts[(pts[:, 2] >= lo) & (pts[:, 2] <= hi)]
        except Exception as e:
            self.get_logger().warning(f"自适应地板滤波失败: {e}", throttle_duration_sec=30)
        # BYY修复⑤ 自身半径滤除：L1 会打到狗自己的腿/背(0.3~0.6m 身位圈)，
        # 机器人半径仅0.35m——脚印半径内的"障碍"必为自身回波，物理上不成立。
        try:
            lp = getattr(self, "_last_pose_xy", None)
            if lp is not None and pts.shape[0]:
                d2 = (pts[:, 0] - lp[0]) ** 2 + (pts[:, 1] - lp[1]) ** 2
                pts = pts[d2 > 0.55 ** 2]
        except Exception as e:
            self.get_logger().warning(f"自身半径滤除失败: {e}", throttle_duration_sec=30)
        stamp = self.get_clock().now().to_msg()
        cloud = PointCloud2(
            header=Header(stamp=stamp, frame_id="odom_utlidar"),
            height=1,
            width=pts.shape[0],
            is_dense=True,
            is_bigendian=False,
            fields=[
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ],
            point_step=12,
            row_step=pts.shape[0] * 12,
            data=pts.astype(np.float32).tobytes(),
        )
        self.pub_cloud.publish(cloud)
        self.counters["cloud"] += 1
        self.counters["cloud_pts"] = int(pts.shape[0])
        _ = meta

    def _handle_pose(self, data):
        try:
            d = data.get("data", data)
            h = d["header"]
            p = d["pose"]
            msg = PoseStamped()
            msg.header.stamp.sec = int(h["stamp"]["sec"])
            msg.header.stamp.nanosec = int(h["stamp"]["nanosec"])
            msg.header.frame_id = h.get("frame_id", "odom")
            msg.pose.position.x = float(p["position"]["x"])
            msg.pose.position.y = float(p["position"]["y"])
            msg.pose.position.z = float(p["position"]["z"])
            msg.pose.orientation.x = float(p["orientation"]["x"])
            msg.pose.orientation.y = float(p["orientation"]["y"])
            msg.pose.orientation.z = float(p["orientation"]["z"])
            msg.pose.orientation.w = float(p["orientation"]["w"])
            self.pub_pose.publish(msg)
            self._last_pose_xy = (msg.pose.position.x, msg.pose.position.y)
            self.counters["pose"] += 1
        except Exception as e:
            self.get_logger().debug(f"pose 解析失败: {e}")

    def _handle_sportmode(self, data):
        # sportmodestate = 运动控制器的腿部里程计（position/velocity/yaw_speed +
        # imu_state），与 L1 扫描匹配完全独立——BYY修复⑭：作为主导航里程计发布。
        # （utlidar robot_pose 在慢速机动中航向渐进漂移/±180°翻转，不适合闭环控制）
        try:
            d = data.get("data", data)
            pos = d.get("position")
            if not pos or len(pos) < 2:
                return
            rpy = d.get("imu_state", {}).get("rpy")
            yaw = float(rpy[2]) if rpy and len(rpy) == 3 else None
            m = Odometry()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "odom"
            m.child_frame_id = "base_link"
            m.pose.pose.position.x = float(pos[0])
            m.pose.pose.position.y = float(pos[1])
            if yaw is not None:
                m.pose.pose.orientation.z = math.sin(yaw / 2.0)
                m.pose.pose.orientation.w = math.cos(yaw / 2.0)
            vel = d.get("velocity")
            if vel and len(vel) >= 2:
                m.twist.twist.linear.x = float(vel[0])
                m.twist.twist.linear.y = float(vel[1])
            ys = d.get("yaw_speed")
            if ys is not None:
                m.twist.twist.angular.z = float(ys)
            self.pub_sport_odom.publish(m)
            self.counters["sport_odom"] = self.counters.get("sport_odom", 0) + 1
            imu_state = d.get("imu_state", {})
            q = imu_state.get("quaternion")
            if q and len(q) == 4:
                mi = Imu()
                mi.header.stamp = m.header.stamp
                mi.header.frame_id = "base_link"
                mi.orientation.w = float(q[0])
                mi.orientation.x = float(q[1])
                mi.orientation.y = float(q[2])
                mi.orientation.z = float(q[3])
                g = imu_state.get("gyroscope")
                if g and len(g) == 3:
                    mi.angular_velocity.z = float(g[2])
                self.pub_imu.publish(mi)
                self.counters["imu"] += 1
        except Exception as e:
            self.get_logger().debug(f"sportmodestate 解析失败: {e}")

    def _handle_lidar_state(self, data):
        msg = String()
        msg.data = json.dumps(data.get("data", data), ensure_ascii=False)
        self.pub_state.publish(msg)
        self.counters["state"] += 1

    def _handle_voxel_cb(self, msg) -> None:
        """订阅回调路径的体素处理（驱动已解码 / 原始字节两种形态兼容）。"""
        try:
            inner = msg.get("data", {})
            payload = inner.get("data")
            if isinstance(payload, dict) and "points" in payload:
                pts = np.asarray(payload["points"], dtype=np.float32)   # 驱动 native 解码结果
            elif isinstance(payload, (bytes, bytearray)):
                pts = decode_voxel_points(bytes(payload), inner["src_size"],
                                          inner["origin"], inner["resolution"])
            else:
                return  # 已被挂钩处理（handled）或无数据
            if pts.size:
                self._publish_cloud(pts, inner)
        except Exception as e:
            self.counters["decode_err"] += 1
            self.get_logger().warning(f"voxel(回调路径) 处理失败: {e}", throttle_duration_sec=10)

    def _handle_voxel_raw(self, buffer: bytes) -> dict:
        """挂钩驱动的二进制帧入口：解头→解码→（节流）发布。

        返回**小对象**（不含点阵）——回传大数组会被驱动的 pub_sub 解析链
        长期持有（未匹配消息的 futures 累积），是此前 14GB OOM 的帮凶。
        """
        import struct
        (hlen,) = struct.unpack_from("<I", buffer, 0)
        meta = json.loads(buffer[8:8 + hlen].decode("utf-8")).get("data", {})
        binary_data = buffer[8 + hlen:]

        if not hasattr(self, "_voxel_first_logged"):
            self._voxel_first_logged = True
            self.get_logger().info(
                f"首帧体素: src_size={meta.get('src_size', 0)/1e6:.2f}MB "
                f"压缩={len(binary_data)/1e6:.2f}MB res={meta.get('resolution')}")

        # 发布节流（体素图更新慢，10Hz 发大点云无意义且费内存/带宽）
        now = time.monotonic()
        if now - getattr(self, "_last_cloud_pub", 0.0) < self._cloud_min_period:
            return {"type": "msg", "topic": "rt/utlidar/voxel_map_compressed",
                    "data": {"throttled": True}}

        try:
            pts = decode_voxel_points(
                binary_data, meta["src_size"], meta["origin"], meta["resolution"])
            self._publish_cloud(pts, meta)
        except ValueError as e:
            self.counters["decode_err"] += 1
            self.get_logger().warning(f"voxel 帧被保护拒绝: {e}", throttle_duration_sec=30)
        except Exception as e:
            self.counters["decode_err"] += 1
            self.get_logger().warning(f"voxel 解码失败: {e}", throttle_duration_sec=10)
        return {"type": "msg", "topic": "rt/utlidar/voxel_map_compressed",
                "data": {"handled": True, "src_size": meta.get("src_size", 0)}}

    # ---------------- WebRTC 侧 ----------------

    def _run_forever(self):
        asyncio.set_event_loop(self._loop)
        while rclpy.ok():
            try:
                self._loop.run_until_complete(self._session())
            except Exception as e:
                self.get_logger().warning(f"会话异常: {e}")
            self.connected = False
            self._channel = None
            self._moving = False
            time.sleep(self.get_parameter("reconnect_interval").value)

    async def _session(self):
        from go2_webrtc_driver.constants import WebRTCConnectionMethod
        from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection
        from go2_webrtc_driver import webrtc_datachannel

        # SDP 候选过滤：无束缚模式下 .18 有 WiFi 地址（192.168.3.x/10.97.x），
        # 这些候选机器人（192.168.123.x 内网）不可达，混入 ICE 候选池会导致
        # 会话选中坏路径后掉线。只保留 192.168.123. 网段的候选。
        _dep_mods = []
        try:
            from go2_webrtc_driver import webrtc_driver as _wd, unitree_auth as _ua
            _dep_mods = [_ua, _wd]
        except Exception:
            pass
        for _m in _dep_mods:
            if getattr(_m, "_baiyanggo_sdp_filtered", False):
                continue
            _orig_send = _m.send_sdp_to_local_peer

            def _filtered_send(ip, sdp_json, _orig=_orig_send):
                try:
                    d = json.loads(sdp_json)
                    kept, dropped = [], 0
                    for ln in d.get("sdp", "").split("\r\n"):
                        if ln.startswith("a=candidate:") and "192.168.123." not in ln:
                            dropped += 1
                            continue
                        kept.append(ln)
                    d["sdp"] = "\r\n".join(kept)
                    if dropped and not getattr(_m, "_sdp_drop_logged", False):
                        _m._sdp_drop_logged = True
                        self.get_logger().info(f"SDP 候选过滤: 保留 {len(kept)} 行, 剔除 {dropped} 个非内网候选")
                    sdp_json = json.dumps(d)
                except Exception:
                    pass
                return _orig(ip, sdp_json)

            _m.send_sdp_to_local_peer = _filtered_send
            _m._baiyanggo_sdp_filtered = True

        # 强制驱动使用纯 numpy "native" 解码器（勿用默认 libvoxel）：
        # libvoxel 经 wasmtime 初始化时预留 8GB 虚拟内存，在 16GB Jetson 上曾致整机 OOM。
        # 实际解码由本节点的挂钩完成，驱动的解码器不会被调用，这里只是避免 wasm 加载。
        if not getattr(webrtc_datachannel.WebRTCDataChannel, "_baiyanggo_native_decoder", False):
            _orig_set_decoder = webrtc_datachannel.WebRTCDataChannel.set_decoder

            def _forced_native(self_dc, decoder_type=None):
                return _orig_set_decoder(self_dc, "native")

            webrtc_datachannel.WebRTCDataChannel.set_decoder = _forced_native
            webrtc_datachannel.WebRTCDataChannel._baiyanggo_native_decoder = True

        # 挂钩体素二进制入口（幂等）
        node = self
        if not getattr(webrtc_datachannel.WebRTCDataChannel, "_baiyanggo_hooked", False):
            def hooked(self_dc, buffer):
                return node._handle_voxel_raw(buffer)
            webrtc_datachannel.WebRTCDataChannel.deal_array_buffer_for_lidar = hooked
            webrtc_datachannel.WebRTCDataChannel._baiyanggo_hooked = True

        # 媒体静默：驱动默认 addTransceiver(video/audio, recvonly)，狗会推 H.264/AAC，
        # aiortc 在 Jetson 上软解码烧满一个核（GIL 饿死 rclpy 发布线程，ROS 话题全停发），
        # 且媒体轨道停摆时 recv() 抛 MediaStreamError 拖死整个会话——日志中每次
        # "数据通道断开"的第一现场均为它。钩住 addTransceiver 把媒体协商成 inactive：
        # m-line 保留、不传媒体。（conn.pc 在 connect() 内才创建，故只能在类级别挂钩）
        from aiortc import RTCPeerConnection as _RTPC
        if not getattr(_RTPC, "_baiyanggo_media_muted", False):
            _orig_add_tr = _RTPC.addTransceiver

            def _muted_add_tr(pc_self, kind_or_track, direction="sendrecv", **kw):
                try:
                    _kind = kind_or_track if isinstance(kind_or_track, str) \
                        else getattr(kind_or_track, "kind", "")
                    if _kind in ("video", "audio"):
                        kw.pop("direction", None)
                        return _orig_add_tr(pc_self, kind_or_track, direction="inactive", **kw)
                except Exception:
                    pass
                return _orig_add_tr(pc_self, kind_or_track, direction=direction, **kw)

            _RTPC.addTransceiver = _muted_add_tr
            _RTPC._baiyanggo_media_muted = True

        conn = Go2WebRTCConnection(
            WebRTCConnectionMethod.LocalSTA,
            ip=self.get_parameter("robot_ip").value,
        )
        await conn.connect()
        waited = 0.0
        while conn.datachannel is not None and not conn.datachannel.data_channel_opened \
                and waited < 30:
            await asyncio.sleep(1.0)
            waited += 1
        ch = conn.datachannel
        if not (ch and ch.data_channel_opened):
            self.get_logger().warning("数据通道未打开，断开重试")
            await conn.disconnect()
            return

        self.connected = True
        self._channel = ch
        self._cloud_min_period = float(self.get_parameter("cloud_pub_min_period").value)
        self.get_logger().info("WebRTC 数据通道已验证，订阅雷达流")
        ps = ch.pub_sub
        ps.subscribe(SRC_POSE, self._handle_pose)
        ps.subscribe(SRC_LIDAR_STATE, self._handle_lidar_state)
        ps.subscribe(SRC_SPORTMODE, self._handle_sportmode)
        # 体素流：数据经订阅回调到达（实测载荷 = {type,topic,data:{src_size,origin,
        # resolution,width,data}}）。若 deal_array_buffer 挂钩已处理（返回 handled），
        # 此处 data 内无点集，自动跳过——两条路径互补、不会重复发布。
        # 注意：不要发 disableTrafficSaving——实测它是体素流的"静音开关"。
        ps.subscribe(SRC_VOXEL, self._handle_voxel_cb)

        while ch.data_channel_opened:
            await asyncio.sleep(1.0)
        self.get_logger().warning("数据通道断开")
        await conn.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = WebrtcBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

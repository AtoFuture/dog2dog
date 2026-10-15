#!/usr/bin/env bash
# BaiyangGo WebRTC 桥部署脚本（在扩展坞 Jetson .18 上执行，unitree 用户）
# 功能：建/补 webrtc_venv（含驱动补丁）→ 安装桥包 → 注册 systemd 常驻服务
set -euo pipefail

VENV="$HOME/webrtc_venv"
BRIDGE_SRC="$(cd "$(dirname "$0")" && pwd)"
PYPI_MIRROR="https://mirrors.aliyun.com/pypi/simple/"

log() { echo "[baiyanggo-install] $*"; }

# ---------- 1. venv 与依赖 ----------
if [ ! -d "$VENV" ]; then
    log "创建 venv ..."
    python3 -m venv "$VENV"
fi
"$VENV/bin/pip" install -q -i "$PYPI_MIRROR" go2-webrtc-connect numpy lz4 || {
    log "pip 安装失败（检查 .18 的互联网出口：默认经 A6000 NAT）"; exit 1; }

# ---------- 2. 驱动补丁（幂等） ----------
"$VENV/bin/python" - <<'PYEOF'
import os
site = os.path.expanduser("~/webrtc_venv/lib/python3.8/site-packages/go2_webrtc_driver")

# 2a) py3.8 兼容：PEP604 联合类型 → Optional
repl = {
    "util.py": [("str | None", "Optional[str]"),
                ("RSA.RsaKey | None", "Optional[RSA.RsaKey]"),
                (") -> dict | None:", ") -> Optional[dict]:")],
    "webrtc_audiohub.py": [("logging.Logger | None", "Optional[logging.Logger]")],
}
for fn, pairs in repl.items():
    p = os.path.join(site, fn)
    if not os.path.exists(p):
        continue
    s = open(p).read()
    orig = s
    for a, b in pairs:
        s = s.replace(a, b)
    if s != orig and "from typing import Optional" not in s:
        # 插到首个 import 之前
        idx = s.index("import ")
        s = s[:idx] + "from typing import Optional\n" + s[idx:]
    open(p, "w").write(s)

# 2b) 超时 sys.exit(1) → 可重试
p = os.path.join(site, "webrtc_datachannel.py")
s = open(p).read()
old = '            print("Data channel did not open in time")\n            sys.exit(1)'
new = '            print("Data channel did not open in time (patched: no exit)")'
if old in s:
    open(p, "w").write(s.replace(old, new))
print("驱动补丁完成")
PYEOF

# ---------- 3. systemd 服务 ----------
mkdir -p ~/.config/systemd/user
cat > ~/.config/systemd/user/baiyanggo-webrtc-bridge.service <<EOF
[Unit]
Description=BaiyangGo WebRTC-to-ROS2 bridge (utlidar streams)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
Environment=PYTHONPATH=${BRIDGE_SRC}
# 必须与狗载其余节点同 RMW（CycloneDDS），否则话题互不可见
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
Environment=CYCLONEDDS_URI=file:///home/unitree/cyclonedds_ws/cyclonedds.xml
ExecStart=/bin/bash -c 'source /opt/ros/foxy/setup.bash && source /home/unitree/cyclonedds_ws/install/setup.bash && exec ${VENV}/bin/python -u -m baiyanggo_webrtc_bridge.bridge_node'
Restart=always
RestartSec=15
# 内存硬上限：桥跑飞时由 cgroup 杀死自己，绝不 OOM 整台 Jetson（曾发生 14GB OOM）
MemoryMax=4G
MemorySwapMax=0

[Install]
WantedBy=default.target
EOF
systemctl --user daemon-reload
systemctl --user enable baiyanggo-webrtc-bridge.service
log "服务已注册：systemctl --user {start|status|stop} baiyanggo-webrtc-bridge"
log "话题：/baiyanggo/webrtc/{pose,voxel_map,lidar_state,status}"
log "验证：source /opt/ros/foxy/setup.bash && ros2 topic hz /baiyanggo/webrtc/pose"

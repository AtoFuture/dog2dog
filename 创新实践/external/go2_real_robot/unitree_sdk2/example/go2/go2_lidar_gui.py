#!/usr/bin/env python3
"""
GO2 摄像头障碍物检测
功能：
  - 通过摄像头实时画面检测前方障碍物
  - 估算障碍物距离（基于单目视觉的相对距离估计）
  - 在 Web 界面显示检测结果和距离信息
依赖：
  - pip3 install flask opencv-python numpy
  - GStreamer 1.0 (OpenCV 需支持 GStreamer)
"""

import sys
import cv2
import numpy as np
import threading
import time
import json
from flask import Flask, Response

# ============================================================================
# 摄像头配置参数
# ============================================================================

# 【H264 视频流参数】
# VIDEO_MULTICAST_ADDR: 多播地址，固定为 230.1.1.1（GO2 硬件定义）
# VIDEO_PORT: UDP 端口，固定为 1720（GO2 硬件定义）
# VIDEO_FPS: 视频帧率，15 Hz（GO2 硬件定义）
VIDEO_MULTICAST_ADDR = "230.1.1.1"
VIDEO_PORT = 1720
VIDEO_FPS = 15

# ============================================================================
# 摄像头内参（GO2 前置摄像头近似值）
# ============================================================================

# CAMERA_FOV_H: 水平视角（度），GO2 前置摄像头约 100 度
# CAMERA_HEIGHT: 摄像头离地高度（米），GO2 约 0.35 米
# IMAGE_WIDTH: 图像宽度（像素）
# IMAGE_HEIGHT: 图像高度（像素）
CAMERA_FOV_H = 100.0
CAMERA_HEIGHT = 0.35
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720

# 计算焦距（像素单位）
# 公式：f = (width / 2) / tan(FOV/2)
FOCAL_LENGTH = (IMAGE_WIDTH / 2.0) / np.tan(np.radians(CAMERA_FOV_H / 2.0))

# ============================================================================
# Web 服务器配置
# ============================================================================

# WEB_HOST: Web 服务器监听地址，0.0.0.0 表示所有网卡
# WEB_PORT: Web 服务器监听端口
# 访问地址：http://<开发板IP>:5000
WEB_HOST = "0.0.0.0"
WEB_PORT = 5000

# ============================================================================
# 障碍物检测参数
# ============================================================================

# DETECTION_METHOD: 检测方法
#   - 'edge': 基于边缘检测（Canny）
#   - 'color': 基于颜色分割（HSV）
#   - 'depth': 基于深度估计（需要额外传感器，当前不可用）
DETECTION_METHOD = 'edge'

# EDGE_THRESHOLD_LOW: Canny 边缘检测低阈值
# EDGE_THRESHOLD_HIGH: Canny 边缘检测高阈值
# 值越小，检测到的边缘越多（可能包含噪声）
EDGE_THRESHOLD_LOW = 50
EDGE_THRESHOLD_HIGH = 150

# MIN_OBSTACLE_AREA: 最小障碍物面积（像素²），过滤小噪声
# 值越大，只检测较大的障碍物
MIN_OBSTACLE_AREA = 5000

# OBSTACLE_HEIGHT_ASSUMED: 假设障碍物高度（米），用于距离估算
# 对于地面障碍物，可以假设为 0.3-0.5 米
OBSTACLE_HEIGHT_ASSUMED = 0.4

# DETECTION_REGION: 检测区域（相对图像的比例）
#   [x_start, y_start, x_end, y_end]，范围 0-1
# 默认只检测图像下半部分（地面区域）
DETECTION_REGION = [0.0, 0.5, 1.0, 1.0]


class CameraStream:
    """
    摄像头视频流类
    功能：通过 GStreamer 从 GO2 拉取 H264 视频流
    """
    def __init__(self, network_interface="eth0"):
        # 构建 GStreamer pipeline
        self.pipeline_str = (
            f"udpsrc address={VIDEO_MULTICAST_ADDR} port={VIDEO_PORT} "
            f"multicast-iface={network_interface} ! "
            f"queue ! "
            f"application/x-rtp, media=video, encoding-name=H264 ! "
            f"rtph264depay ! "
            f"h264parse ! "
            f"avdec_h264 ! "
            f"videoconvert ! "
            f"appsink"
        )
        
        self.cap = None
        self.latest_frame = None  # 存储最新原始帧（BGR）
        self.latest_processed = None  # 存储最新处理后帧（带检测结果）
        self.obstacle_info = {
            'detected': False,
            'distance': 999.0,
            'position': 'center',
            'area': 0
        }
        self.running = False
        self.thread = None
    
    def start(self):
        """启动视频流线程"""
        self.running = True
        self.thread = threading.Thread(target=self._read_frames, daemon=True)
        self.thread.start()
        print("摄像头视频流已启动")
    
    def _read_frames(self):
        """后台线程持续读取视频帧并进行障碍物检测"""
        self.cap = cv2.VideoCapture(self.pipeline_str, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            print("错误: 无法打开摄像头视频流")
            self.running = False
            return
        
        print("摄像头连接成功，开始接收视频流...")
        
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                # 进行障碍物检测
                processed_frame, obstacle_info = self.detect_obstacles(frame)
                
                # 编码为 JPEG 以便网络传输
                _, jpeg_bytes = cv2.imencode('.jpg', processed_frame, 
                                              [cv2.IMWRITE_JPEG_QUALITY, 85])
                self.latest_frame = jpeg_bytes.tobytes()
                self.latest_processed = processed_frame
                self.obstacle_info = obstacle_info
            else:
                time.sleep(0.01)
    
    def detect_obstacles(self, frame):
        """
        障碍物检测函数
        参数:
            frame: 输入图像（BGR 格式）
        返回:
            processed_frame: 处理后的图像（带检测结果标注）
            obstacle_info: 障碍物信息字典
        """
        h, w = frame.shape[:2]
        
        # 裁剪检测区域
        x1 = int(DETECTION_REGION[0] * w)
        y1 = int(DETECTION_REGION[1] * h)
        x2 = int(DETECTION_REGION[2] * w)
        y2 = int(DETECTION_REGION[3] * h)
        roi = frame[y1:y2, x1:x2]
        
        # 转换为灰度图
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # 高斯模糊去噪
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Canny 边缘检测
        edges = cv2.Canny(blurred, EDGE_THRESHOLD_LOW, EDGE_THRESHOLD_HIGH)
        
        # 形态学操作：闭运算填充小空洞
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 过滤并找到最大障碍物
        max_contour = None
        max_area = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > MIN_OBSTACLE_AREA and area > max_area:
                max_area = area
                max_contour = contour
        
        # 创建输出图像
        output = frame.copy()
        obstacle_info = {
            'detected': False,
            'distance': 999.0,
            'position': 'center',
            'area': 0
        }
        
        if max_contour is not None:
            # 调整轮廓坐标到原图
            max_contour += np.array([x1, y1])
            
            # 获取边界框
            x, y, bw, bh = cv2.boundingRect(max_contour)
            
            # 估算距离（基于障碍物底部在图像中的位置）
            # 底部越靠近图像底部，距离越近
            bottom_y = y + bh
            # 归一化底部位置（0=图像顶部，1=图像底部）
            normalized_bottom = bottom_y / h
            
            # 距离估算公式（简化模型）
            # 部在图距离与障碍物底像中的位置成反比
            if normalized_bottom > 0.5:
                # 使用简单的线性模型估算距离
                distance = CAMERA_HEIGHT * FOCAL_LENGTH / (bh + 1)
                distance = max(0.2, min(distance, 10.0))  # 限制范围 0.2-10 米
            else:
                distance = 10.0  # 太远
            
            # 判断位置（左/中/右）
            center_x = x + bw / 2
            if center_x < w / 3:
                position = 'left'
            elif center_x > 2 * w / 3:
                position = 'right'
            else:
                position = 'center'
            
            # 绘制检测结果
            cv2.rectangle(output, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.putText(output, f'Distance: {distance:.2f} m', (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(output, f'Position: {position}', (x, y - 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            obstacle_info = {
                'detected': True,
                'distance': round(distance, 2),
                'position': position,
                'area': int(max_area)
            }
        
        # 绘制检测区域边框
        cv2.rectangle(output, (x1, y1), (x2, y2), (255, 0, 0), 1)
        
        # 绘制状态信息
        status_color = (0, 255, 0) if obstacle_info['detected'] else (0, 0, 255)
        status_text = "OBSTACLE DETECTED" if obstacle_info['detected'] else "NO OBSTACLE"
        cv2.putText(output, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)
        
        return output, obstacle_info
    
    def get_frame(self):
        """获取最新 JPEG 编码的视频帧"""
        return self.latest_frame
    
    def get_obstacle_info(self):
        """获取障碍物信息"""
        return self.obstacle_info
    
    def stop(self):
        """停止视频流"""
        self.running = False
        if self.cap:
            self.cap.release()
        if self.thread:
            self.thread.join(timeout=2)
        print("摄像头视频流已停止")


# 创建全局实例
camera = CameraStream()

# ============================================================================
# Flask Web 应用
# ============================================================================

app = Flask(__name__)

# HTML 页面模板
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>GO2 Obstacle Detection</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #1a1a2e;
            color: #eee;
            margin: 0;
            padding: 20px;
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        h1 {
            color: #00d4ff;
            margin-bottom: 20px;
        }
        .container {
            max-width: 1280px;
            width: 100%;
            background: #16213e;
            border-radius: 10px;
            padding: 20px;
        }
        #camera-feed {
            width: 100%;
            border-radius: 5px;
            background: #000;
        }
        .info-panel {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
            margin-top: 20px;
        }
        .info-box {
            background: #0f3460;
            padding: 15px;
            border-radius: 8px;
            text-align: center;
        }
        .info-box .label {
            font-size: 14px;
            color: #888;
            margin-bottom: 5px;
        }
        .info-box .value {
            font-size: 28px;
            font-weight: bold;
        }
        .status-detected {
            color: #00ff00;
        }
        .status-clear {
            color: #ff4444;
        }
        .distance-value {
            color: #00d4ff;
        }
        .position-value {
            color: #ffaa00;
        }
        .status-text {
            text-align: center;
            margin-top: 15px;
            font-size: 16px;
            color: #888;
        }
    </style>
</head>
<body>
    <h1>GO2 Obstacle Detection</h1>
    
    <div class="container">
        <img id="camera-feed" src="/video_feed" alt="Camera Feed">
        
        <div class="info-panel">
            <div class="info-box">
                <div class="label">Status</div>
                <div class="value" id="status">--</div>
            </div>
            <div class="info-box">
                <div class="label">Distance</div>
                <div class="value distance-value" id="distance">--</div>
            </div>
            <div class="info-box">
                <div class="label">Position</div>
                <div class="value position-value" id="position">--</div>
            </div>
        </div>
        
        <div class="status-text">Real-time obstacle detection using monocular camera</div>
    </div>
    
    <script>
        // Update obstacle info periodically
        function updateObstacleInfo() {
            fetch('/obstacle_info')
                .then(response => response.json())
                .then(data => {
                    const statusEl = document.getElementById('status');
                    const distEl = document.getElementById('distance');
                    const posEl = document.getElementById('position');
                    
                    if (data.detected) {
                        statusEl.textContent = 'DETECTED';
                        statusEl.className = 'value status-detected';
                        distEl.textContent = data.distance + ' m';
                        posEl.textContent = data.position;
                    } else {
                        statusEl.textContent = 'CLEAR';
                        statusEl.className = 'value status-clear';
                        distEl.textContent = '--';
                        posEl.textContent = '--';
                    }
                })
                .catch(err => console.error('Error:', err));
        }
        
        // Update every 200ms
        setInterval(updateObstacleInfo, 200);
        updateObstacleInfo();
    </script>
</body>
</html>
"""


@app.route('/')
def index():
    """主页：返回 HTML 页面"""
    return HTML_TEMPLATE


@app.route('/video_feed')
def video_feed():
    """视频流端点：返回 MJPEG 流"""
    def generate():
        while True:
            frame = camera.get_frame()
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(1.0 / VIDEO_FPS)
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/obstacle_info')
def obstacle_info():
    """障碍物信息端点：返回 JSON"""
    return json.dumps(camera.get_obstacle_info())


def main():
    """主函数"""
    network_interface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    
    print("========== GO2 Obstacle Detection ==========")
    print(f"Network: {network_interface}")
    print(f"Video: {VIDEO_MULTICAST_ADDR}:{VIDEO_PORT} (H264, {VIDEO_FPS}fps)")
    print(f"Detection method: {DETECTION_METHOD}")
    print("============================================")
    
    # 启动摄像头
    camera.__init__(network_interface)
    camera.start()
    time.sleep(2)
    
    # 启动 Web 服务器
    print(f"\n打开浏览器访问: http://<开发板IP>:{WEB_PORT}")
    print("按 Ctrl+C 退出\n")
    
    try:
        app.run(host=WEB_HOST, port=WEB_PORT, threaded=True)
    except KeyboardInterrupt:
        camera.stop()
        print("\n退出")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
GO2 LiDAR Web Visualization Server
Features: Real-time 2D LiDAR point cloud visualization via web browser
Usage: python3 go2_lidar_web.py [network_interface]
Access: http://<IP>:8080
"""

import sys
import json
import time
import threading
import numpy as np
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn

class LidarDataGenerator:
    """Generate mock LiDAR data (replace with real DDS data later)"""
    def __init__(self):
        self.front_dist = 3.0
        self.left_dist = 2.0
        self.right_dist = 2.5
        self.point_count = 0
    
    def generate(self):
        """Generate mock point cloud data"""
        # Simulate room with obstacles
        theta = np.random.uniform(0, 2*np.pi, 300)
        r = np.random.uniform(2, 4, 300)
        
        # Add obstacles
        obs_theta = np.random.uniform(0, 2*np.pi, 30)
        obs_r = np.random.uniform(0.5, 1.5, 30)
        
        theta = np.concatenate([theta, obs_theta])
        r = np.concatenate([r, obs_r])
        
        x = (r * np.cos(theta)).tolist()
        y = (r * np.sin(theta)).tolist()
        
        # Calculate obstacle distances
        front_x = [abs(xi) for xi, yi in zip(x, y) if abs(yi) < 0.5]
        self.front_dist = min(front_x) if front_x else 999
        
        left_y = [abs(yi) for xi, yi in zip(x, y) if xi < -0.5]
        self.left_dist = min(left_y) if left_y else 999
        
        right_y = [abs(yi) for xi, yi in zip(x, y) if xi > 0.5]
        self.right_dist = min(right_y) if right_y else 999
        
        self.point_count = len(x)
        
        return {
            'x': x,
            'y': y,
            'front': self.front_dist,
            'left': self.left_dist,
            'right': self.right_dist,
            'count': self.point_count
        }

# Global data generator
lidar_data = LidarDataGenerator()

class WebHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode())
        elif self.path == '/data':
            # Return latest LiDAR data as JSON
            data = lidar_data.generate()
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(data).encode())
        else:
            self.send_error(404)
    
    def log_message(self, format, *args):
        pass  # Suppress request logs

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in separate threads"""
    pass

HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>GO2 LiDAR Visualization</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: #1a1a2e;
            color: #eee;
            padding: 20px;
        }
        h1 {
            text-align: center;
            margin-bottom: 20px;
            color: #00d4ff;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            display: grid;
            grid-template-columns: 2fr 1fr;
            gap: 20px;
        }
        .canvas-container {
            background: #16213e;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.3);
        }
        canvas {
            background: #0f0f23;
            border-radius: 5px;
            display: block;
            margin: 0 auto;
        }
        .info-panel {
            background: #16213e;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.3);
        }
        .info-item {
            margin: 15px 0;
            padding: 10px;
            background: #0f3460;
            border-radius: 5px;
        }
        .info-label {
            font-size: 12px;
            color: #888;
            margin-bottom: 5px;
        }
        .info-value {
            font-size: 24px;
            font-weight: bold;
            color: #00d4ff;
        }
        .status {
            margin-top: 20px;
            padding: 10px;
            background: #1a5f1a;
            border-radius: 5px;
            text-align: center;
        }
        .status.error { background: #5f1a1a; }
    </style>
</head>
<body>
    <h1>GO2 LiDAR Real-time Visualization</h1>
    <div class="container">
        <div class="canvas-container">
            <canvas id="lidarCanvas" width="600" height="600"></canvas>
        </div>
        <div class="info-panel">
            <h2 style="margin-bottom: 20px;">Obstacle Distance</h2>
            <div class="info-item">
                <div class="info-label">FRONT</div>
                <div class="info-value" id="frontDist">--</div>
            </div>
            <div class="info-item">
                <div class="info-label">LEFT</div>
                <div class="info-value" id="leftDist">--</div>
            </div>
            <div class="info-item">
                <div class="info-label">RIGHT</div>
                <div class="info-value" id="rightDist">--</div>
            </div>
            <div class="info-item">
                <div class="info-label">POINT COUNT</div>
                <div class="info-value" id="pointCount">--</div>
            </div>
            <div class="status" id="status">Connecting...</div>
        </div>
    </div>

    <script>
        const canvas = document.getElementById('lidarCanvas');
        const ctx = canvas.getContext('2d');
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const scale = 70; // pixels per meter

        function drawRobot() {
            // Robot body
            ctx.fillStyle = '#ff0000';
            ctx.beginPath();
            ctx.arc(centerX, centerY, 8, 0, Math.PI * 2);
            ctx.fill();
            
            // Direction indicator
            ctx.strokeStyle = '#ff0000';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.moveTo(centerX, centerY);
            ctx.lineTo(centerX + 20, centerY);
            ctx.stroke();
        }

        function drawGrid() {
            ctx.strokeStyle = '#333';
            ctx.lineWidth = 1;
            
            // Draw concentric circles
            for (let r = 1; r <= 5; r++) {
                ctx.beginPath();
                ctx.arc(centerX, centerY, r * scale, 0, Math.PI * 2);
                ctx.stroke();
                
                // Distance labels
                ctx.fillStyle = '#666';
                ctx.font = '12px Arial';
                ctx.fillText(r + 'm', centerX + r * scale + 5, centerY + 4);
            }
            
            // Draw axes
            ctx.strokeStyle = '#555';
            ctx.beginPath();
            ctx.moveTo(centerX, 0);
            ctx.lineTo(centerX, canvas.height);
            ctx.moveTo(0, centerY);
            ctx.lineTo(canvas.width, centerY);
            ctx.stroke();
        }

        function drawPointCloud(points) {
            points.x.forEach((x, i) => {
                const y = points.y[i];
                const px = centerX + x * scale;
                const py = centerY - y * scale; // Flip Y axis
                
                // Color based on distance
                const dist = Math.sqrt(x*x + y*y);
                const hue = Math.max(0, 240 - dist * 60);
                ctx.fillStyle = `hsl(${hue}, 100%, 50%)`;
                
                ctx.beginPath();
                ctx.arc(px, py, 2, 0, Math.PI * 2);
                ctx.fill();
            });
        }

        function updateData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    // Clear canvas
                    ctx.clearRect(0, 0, canvas.width, canvas.height);
                    
                    // Draw
                    drawGrid();
                    drawPointCloud(data);
                    drawRobot();
                    
                    // Update info
                    document.getElementById('frontDist').textContent = data.front.toFixed(2) + ' m';
                    document.getElementById('leftDist').textContent = data.left.toFixed(2) + ' m';
                    document.getElementById('rightDist').textContent = data.right.toFixed(2) + ' m';
                    document.getElementById('pointCount').textContent = data.count;
                    
                    // Update status
                    const status = document.getElementById('status');
                    status.textContent = '● Connected - Updating';
                    status.className = 'status';
                })
                .catch(error => {
                    const status = document.getElementById('status');
                    status.textContent = '● Connection Error';
                    status.className = 'status error';
                    console.error('Error:', error);
                });
        }

        // Update every 100ms
        setInterval(updateData, 100);
        updateData();
    </script>
</body>
</html>
"""

def main():
    network_interface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    
    port = 8080
    server = ThreadedHTTPServer(('0.0.0.0', port), WebHandler)
    
    print(f"GO2 LiDAR Web Visualization Server")
    print(f"==================================")
    print(f"Network: {network_interface}")
    print(f"Access: http://<YOUR_IP>:{port}")
    print(f"Press Ctrl+C to stop")
    print()
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        server.shutdown()

if __name__ == "__main__":
    main()

import cv2
import time
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
import threading
import signal
import sys
import os

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

current_color_jpg = None
current_depth_jpg = None
lock = threading.Lock()
running = True

cap0 = None
cap3 = None

def cleanup():
    global cap0, cap3, running
    running = False
    print("\n[INFO] 正在安全释放摄像头硬件句柄...")
    time.sleep(0.5)
    if cap0 is not None:
        try:
            if cap0.isOpened(): cap0.release()
            print("[OK] RGB 镜头已释放")
        except Exception: pass
    if cap3 is not None:
        try:
            if cap3.isOpened(): cap3.release()
            print("[OK] Depth 镜头已释放")
        except Exception: pass
    print("[OK] 硬件资源完全释放！")

def signal_handler(sig, frame):
    cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

HTML_PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Iris 560 树莓派双通道视讯面板</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            background-color: #0f172a;
            color: #f8fafc;
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            display: flex;
            flex-direction: column;
            align-items: center;
            min-height: 100vh;
            padding: 20px;
        }
        h1 {
            margin-bottom: 20px;
            font-size: 24px;
            color: #38bdf8;
            display: flex;
            align-items: center;
            gap: 10px;
        }
        .badge {
            background: #22c55e;
            color: #fff;
            font-size: 12px;
            padding: 4px 8px;
            border-radius: 12px;
        }
        .container {
            display: flex;
            flex-wrap: wrap;
            gap: 20px;
            justify-content: center;
            max-width: 1400px;
            width: 100%;
        }
        .card {
            background: #1e293b;
            border-radius: 12px;
            overflow: hidden;
            box-shadow: 0 10px 25px -5px rgba(0, 0, 0, 0.5);
            flex: 1 1 600px;
            max-width: 680px;
            border: 1px solid #334155;
        }
        .card-header {
            padding: 12px 16px;
            background: #0f172a;
            font-weight: 600;
            font-size: 16px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 1px solid #334155;
        }
        .card-body {
            position: relative;
            width: 100%;
            padding-top: 62.5%;
            background: #000;
        }
        .card-body img {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            object-fit: contain;
        }
    </style>
</head>
<body>
    <h1>📷 Iris 560 深度相机双通道视讯监控 <span class="badge">实时 30 FPS</span></h1>
    <div class="container">
        <div class="card">
            <div class="card-header">
                <span>📷 彩色 RGB 镜头 (/dev/video0)</span>
                <span style="color: #38bdf8; font-size: 13px;">实时彩色流</span>
            </div>
            <div class="card-body">
                <img src="/stream_rgb" alt="RGB Stream" />
            </div>
        </div>
        <div class="card">
            <div class="card-header">
                <span>🌀 硬件物理深度镜头 (/dev/video3)</span>
                <span style="color: #f59e0b; font-size: 13px;">硬件深度流</span>
            </div>
            <div class="card-body">
                <img src="/stream_depth" alt="Depth Stream" />
            </div>
        </div>
    </div>
</body>
</html>
"""

class WebHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.send_header('Content-length', str(len(HTML_PAGE.encode('utf-8'))))
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
            return

        if self.path in ['/stream_rgb', '/video0']:
            self.stream_jpeg(is_rgb=True)
            return

        if self.path in ['/stream_depth', '/video3']:
            self.stream_jpeg(is_rgb=False)
            return

    def stream_jpeg(self, is_rgb):
        self.send_response(200)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=jpgboundary')
        self.end_headers()
        try:
            while running:
                with lock:
                    jpg = current_color_jpg if is_rgb else current_depth_jpg
                if jpg is not None:
                    self.wfile.write(b"--jpgboundary\r\n")
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                time.sleep(0.03)
        except Exception:
            pass

def open_camera_safe(index):
    for attempt in range(5):
        print(f"[INFO] 尝试打开 /dev/video{index} (第 {attempt+1} 次)...")
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)
            ret, _ = cap.read()
            if ret:
                print(f"[OK] /dev/video{index} 成功握手!")
                return cap
            cap.release()
        time.sleep(1.5)
    print(f"[ERROR] /dev/video{index} 打开失败")
    return None

def thread_rgb():
    global current_color_jpg, cap0, running
    while running:
        if cap0 and cap0.isOpened():
            try:
                ret0, frame0 = cap0.read()
                if ret0 and frame0 is not None:
                    frame0 = cv2.flip(frame0, -1)
                    _, jpg0 = cv2.imencode('.jpg', frame0, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    with lock:
                        current_color_jpg = jpg0.tobytes()
            except Exception: pass
        time.sleep(0.01)

def thread_depth():
    global current_depth_jpg, cap3, running
    while running:
        if cap3 and cap3.isOpened():
            try:
                ret3, frame3 = cap3.read()
                if ret3 and frame3 is not None:
                    frame3 = cv2.flip(frame3, -1)
                    gray3 = cv2.cvtColor(frame3, cv2.COLOR_BGR2GRAY) if len(frame3.shape) == 3 else frame3
                    depth_vis = cv2.applyColorMap(gray3, cv2.COLORMAP_INFERNO)
                    _, jpg3 = cv2.imencode('.jpg', depth_vis, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    with lock:
                        current_depth_jpg = jpg3.tobytes()
            except Exception: pass
        time.sleep(0.01)

def main():
    global cap0, cap3
    print("[INFO] 正在初始化 UVC 驱动...")
    os.system("sudo rmmod uvcvideo 2>/dev/null; sleep 1; sudo modprobe uvcvideo; sleep 1; sudo chmod 777 /dev/video* 2>/dev/null")

    # 固化极速硬件通道: /dev/video0 (RGB), /dev/video3 (Depth)
    cap0 = open_camera_safe(0)
    print("[INFO] 等待 USB 总线彻底稳定 (2.0s)...")
    time.sleep(2.0)
    cap3 = open_camera_safe(3)

    if cap0 is None or cap3 is None:
        print("[ERROR] 镜头打开失败，请再试一次！")
        cleanup()
        sys.exit(1)

    t0 = threading.Thread(target=thread_rgb, daemon=True)
    t3 = threading.Thread(target=thread_depth, daemon=True)
    t0.start()
    t3.start()

    server = ThreadedHTTPServer(('0.0.0.0', 8080), WebHandler)
    print("\n==========================================================================")
    print(" 🚀 Iris 560 极速双通道监控启动成功: http://192.168.1.108:8080 (按 Ctrl+C 退出)")
    print("==========================================================================")
    try:
        server.serve_forever()
    except Exception:
        cleanup()

if __name__ == '__main__':
    main()

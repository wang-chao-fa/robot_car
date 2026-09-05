#!/usr/bin/env python3
import cv2, threading, time, signal, numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler

color_jpg = b''
depth_jpg = b''
frame_lock = threading.Lock()
running = True

def capture_loop():
    global color_jpg, depth_jpg, running
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    # 不强制 FOURCC，使用相机默认格式（避免半绿问题）

    if not cap.isOpened():
        print("[ERROR] 无法打开 /dev/video0"); running = False; return

    # 丢弃前 10 帧（硬件预热）
    for _ in range(10): cap.read()
    print(f"[INFO] 相机已打开: {int(cap.get(3))}x{int(cap.get(4))}")

    while running:
        ret, frame = cap.read()
        if not ret or frame is None or frame.size == 0:
            time.sleep(0.02); continue
        if np.std(frame) < 5:   # 过滤无效帧
            continue

        frame = cv2.flip(frame, -1)   # 180° 翻转
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        depth = cv2.applyColorMap(gray, cv2.COLORMAP_INFERNO)

        _, c = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        _, d = cv2.imencode('.jpg', depth, [cv2.IMWRITE_JPEG_QUALITY, 85])
        with frame_lock:
            color_jpg = c.tobytes()
            depth_jpg = d.tobytes()

    cap.release()

def make_handler(is_color):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def do_GET(self):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while running:
                    with frame_lock:
                        data = color_jpg if is_color else depth_jpg
                    if data:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + data + b'\r\n')
                    time.sleep(0.033)
            except: pass
    return H

def main():
    global running
    signal.signal(signal.SIGINT,  lambda *a: globals().update(running=False))
    signal.signal(signal.SIGTERM, lambda *a: globals().update(running=False))
    threading.Thread(target=capture_loop, daemon=True).start()
    time.sleep(2)
    s1 = HTTPServer(('0.0.0.0', 8080), make_handler(True))
    s2 = HTTPServer(('0.0.0.0', 8081), make_handler(False))
    threading.Thread(target=s1.serve_forever, daemon=True).start()
    threading.Thread(target=s2.serve_forever, daemon=True).start()
    print("[INFO] 彩色: http://192.168.1.108:8080")
    print("[INFO] 伪彩: http://192.168.1.108:8081")
    while running: time.sleep(0.5)
    s1.shutdown(); s2.shutdown()

if __name__ == '__main__': main()

import cv2
import time
import os
import threading
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

os.environ["OPENCV_LOG_LEVEL"] = "SILENT"

global_jpeg = None
lock = threading.Lock()

def is_valid_rgb_frame(frame):
    """智能算法：通过像素标准差检测，剔除全绿/全黑的虚假元数据节点"""
    if frame is None or not hasattr(frame, 'shape') or len(frame.shape) != 3:
        return False
    if frame.shape[0] == 0 or frame.shape[1] == 0:
        return False
    # 计算画面像素标准差：纯绿/纯黑画面的标准差为 0，真实彩色画面标准差 > 10
    std_dev = np.std(frame)
    return std_dev > 10.0

def camera_thread():
    global global_jpeg
    
    while True:
        cap = None
        # 逐个扫描端口，自动剔除全是绿屏的伪节点
        for dev_idx in [2, 1, 0, 3, 4]:
            dev_path = f"/dev/video{dev_idx}"
            if not os.path.exists(dev_path):
                continue
            
            c = cv2.VideoCapture(dev_idx, cv2.CAP_V4L2)
            if c.isOpened():
                c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                c.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                c.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
                
                # 测试读取 3 帧，验证是否为真实彩色画面
                valid_count = 0
                for _ in range(3):
                    ret, frame = c.read()
                    if ret and is_valid_rgb_frame(frame):
                        valid_count += 1
                    time.sleep(0.03)
                    
                if valid_count >= 1:
                    w = int(c.get(cv2.CAP_PROP_FRAME_WIDTH))
                    h = int(c.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    print(f"✅ 成功锁定真正的彩色摄像头画面: {dev_path} ({w}x{h})")
                    cap = c
                    break
                else:
                    # 尝试默认分辨率
                    c.release()
                    c = cv2.VideoCapture(dev_idx, cv2.CAP_V4L2)
                    ret, frame = c.read()
                    if ret and is_valid_rgb_frame(frame):
                        w = int(c.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(c.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        print(f"✅ 成功锁定真正的彩色摄像头画面: {dev_path} (默认分辨率 {w}x{h})")
                        cap = c
                        break
                    c.release()
        
        if cap is None:
            print("⚠️ 正在扫描寻找有效的彩色摄像头端口...")
            time.sleep(2)
            continue
            
        print("🎥 彩色真实画面稳定传输中...")
        try:
            while True:
                ret, frame = cap.read()
                if not ret or frame is None or not is_valid_rgb_frame(frame):
                    time.sleep(0.02)
                    continue
                
                # 180 度视角矫正
                frame = cv2.flip(frame, -1)
                
                # JPEG 编码
                ret_jpg, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret_jpg:
                    with lock:
                        global_jpeg = jpeg.tobytes()
                        
                time.sleep(0.03)
        except Exception as e:
            print(f"⚠️ 异常捕获: {e}")
        finally:
            if cap:
                cap.release()
            time.sleep(1.0)

class CamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass
        
    def do_GET(self):
        if self.path == '/' or self.path == '/video':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while True:
                with lock:
                    jpg = global_jpeg
                if jpg is not None:
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(jpg)))
                        self.end_headers()
                        self.wfile.write(jpg)
                        self.wfile.write(b'\r\n')
                    except Exception:
                        break
                time.sleep(0.04)
        else:
            self.send_error(404)

class ThreadedServer(ThreadingMixIn, HTTPServer):
    pass

if __name__ == '__main__':
    t = threading.Thread(target=camera_thread, daemon=True)
    t.start()
    
    server_address = ('0.0.0.0', 8080)
    httpd = ThreadedServer(server_address, CamHandler)
    print("🚀 摄像头 Web 服务已在端口 8080 启动！")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("已停止服务。")

import cv2, threading, socketserver
from http.server import BaseHTTPRequestHandler

def start_cam_server(device_idx, port):
    cap = cv2.VideoCapture(device_idx)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    class CamHandler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def do_GET(self):
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    ret, frame = cap.read()
                    if not ret: break
                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(jpeg.tobytes())
                    self.wfile.write(b'\r\n')
            except Exception: pass

    class ThreadedServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
        daemon_threads = True
        allow_reuse_address = True

    print(f"🚀 /dev/video{device_idx} 已在端口 {port} 启动！")
    ThreadedServer(('0.0.0.0', port), CamHandler).serve_forever()

# 同时启动 /dev/video0 (8080) 和 /dev/video2 (8081)
t1 = threading.Thread(target=start_cam_server, args=(0, 8080), daemon=True)
t2 = threading.Thread(target=start_cam_server, args=(2, 8081), daemon=True)
t1.start(); t2.start()
t1.join(); t2.join()

import cv2
import time
import numpy as np
import threading
import signal
import sys
import os

cap0 = None
cap3 = None
running = True

latest_v0 = 0.0
latest_v3 = 0.0
lock0 = threading.Lock()
lock3 = threading.Lock()

def cleanup():
    global cap0, cap3, running
    running = False
    print("\n[INFO] 正在释放资源...")
    time.sleep(0.5)
    if cap0: cap0.release()
    if cap3: cap3.release()
    print("[OK] 已安全关闭。")

def signal_handler(sig, frame):
    cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

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
    global latest_v0, running
    while running:
        if cap0 and cap0.isOpened():
            try:
                ret0, f0 = cap0.read()
                if ret0 and f0 is not None:
                    mean_val = np.mean(f0)
                    with lock0:
                        latest_v0 = mean_val
            except Exception:
                pass
        time.sleep(0.01)

def thread_depth():
    global latest_v3, running
    while running:
        if cap3 and cap3.isOpened():
            try:
                ret3, f3 = cap3.read()
                if ret3 and f3 is not None:
                    h, w = f3.shape[:2]
                    roi = f3[h//2-50:h//2+50, w//2-50:w//2+50]
                    mean_val = np.mean(roi)
                    with lock3:
                        latest_v3 = mean_val
            except Exception:
                pass
        time.sleep(0.01)

def main():
    global cap0, cap3
    print("[INFO] 重置 UVC 驱动...")
    os.system("sudo rmmod uvcvideo 2>/dev/null; sleep 2; sudo modprobe uvcvideo; sleep 2; sudo chmod 777 /dev/video* 2>/dev/null")
    
    cap0 = open_camera_safe(0)
    print("[INFO] 等待 USB 总线彻底稳定 (2.0s)...")
    time.sleep(2.0)
    cap3 = open_camera_safe(3)

    if cap0 is None or cap3 is None:
        print("[ERROR] 镜头打开失败")
        cleanup()
        sys.exit(1)

    # 启动独立双线程
    t0 = threading.Thread(target=thread_rgb, daemon=True)
    t3 = threading.Thread(target=thread_depth, daemon=True)
    t0.start()
    t3.start()

    print("\n==========================================================================")
    print(" 🚀 树莓派 Iris 560 独立双线程实时数据流 (按 Ctrl+C 退出)")
    print("==========================================================================")

    count = 0
    while running:
        time.sleep(0.1)
        count += 1
        with lock0: v0 = latest_v0
        with lock3: v3 = latest_v3
        
        bar_len = int((v3 / 255.0) * 20)
        ascii_bar = "█" * bar_len + "░" * (20 - bar_len)
        
        print(f"[{count:05d}] 📷 RGB彩色明暗度: {v0:6.2f}  |  🌀 硬件深度数值: {v3:6.2f} [{ascii_bar}]", flush=True)

if __name__ == '__main__':
    main()

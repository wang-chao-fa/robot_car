import cv2
import os
import subprocess

print("================ 树莓派摄像头端口硬件深度扫描 ================")
for i in range(10):
    dev = f"/dev/video{i}"
    if not os.path.exists(dev):
        continue
        
    print(f"\n🎥 测试端口: {dev}")
    
    try:
        info = subprocess.check_output(f"v4l2-ctl -d {dev} --info 2>/dev/null", shell=True).decode()
        card = [line for line in info.split('\n') if 'Card type' in line]
        card_str = card[0].strip() if card else "未知设备"
        print(f"   硬件设备名称: {card_str}")
    except Exception:
        print("   硬件设备名称: 无法获取")
        
    cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("   OpenCV 状态: ❌ 无法打开 (端口被占用或无权限)")
        cap.release()
        continue
        
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    ret, frame = cap.read()
    if ret and frame is not None and frame.size > 0:
        h, w = frame.shape[:2]
        channels = frame.shape[2] if len(frame.shape) > 2 else 1
        print(f"   OpenCV 状态: ✅ 成功读到图像！尺寸={w}x{h}, 通道={channels}")
    else:
        print("   OpenCV 状态: ⚠️ 能打开但读取图像为空帧")
        
    cap.release()
    
print("\n================ 扫描完成 ================")

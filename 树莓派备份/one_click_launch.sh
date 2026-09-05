#!/bin/bash

# 自动加载 ROS 2 及个人工作空间环境变量
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# 优雅退出的处理函数
cleanup() {
    echo ""
    echo "🛑 正在一键关停并释放所有底盘、雷达、IMU、SLAM、URDF模型与监控节点..."
    sudo pkill -9 -f "base_control_node|yesense|vanjee_ros2_driver_node|pointcloud_to_laserscan_node|async_slam_toolbox_node|robot_state_publisher|cam_web.py|component_container|static_transform_publisher" 2>/dev/null || true
    echo "✅ 所有硬件资源与后台节点已完全优雅释放！"
    exit 0
}

# 注册信号捕获：Ctrl+C 或 脚本退出时触发 cleanup
trap cleanup INT TERM EXIT

echo "==================================================="
echo "   树莓派 4B 机器人底盘、雷达、IMU、3D模型、SLAM 与相机一键启动"
echo "==================================================="

# 启动前彻底清理旧节点，防止端口和硬件独占
sudo pkill -9 -f "base_control_node|yesense|vanjee_ros2_driver_node|pointcloud_to_laserscan_node|async_slam_toolbox_node|robot_state_publisher|cam_web.py|component_container|static_transform_publisher" 2>/dev/null || true
sleep 1

sudo ufw disable 2>/dev/null || true

# 自动给所有 USB 串口赋予读写权限
sudo chmod 666 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true

echo "=== 1. 启动 STM32 底盘串口通信节点 (/odom) ==="
ros2 run base_control base_control_node > /dev/null 2>&1 &
sleep 2

echo "=== 2. 启动 3D 机器人 URDF 模型发布节点 (/robot_description & TF) ==="
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(xacro ~/ros2_ws/src/robot_description/urdf/robot.urdf.xacro 2>/dev/null)" > /dev/null 2>&1 &
sleep 1

echo "=== 2.1 启动静态坐标双向兼容绑定 (laser_link <-> laser) ==="
ros2 run tf2_ros static_transform_publisher --frame-id laser_link --child-frame-id laser > /dev/null 2>&1 &
sleep 1

echo "=== 3. 启动 Yesense IMU 节点 (/imu/data_raw) ==="
ros2 launch yesense_std_ros2 yesense_node.launch.py > /dev/null 2>&1 &
sleep 2

echo "=== 4. 启动万集 3D 雷达驱动节点 (/vanjee_pointcloud, frame_id:=laser_link) ==="
ros2 launch vanjee_ros2_driver vanjee_ros2_driver.launch.py frame_id:=laser_link > /dev/null 2>&1 &
sleep 3

echo "=== 5. 启动 2D 干净点云转换节点 (/scan) ==="
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r cloud_in:=/vanjee_pointcloud \
  -r scan:=/scan \
  -p target_frame:=laser_link \
  -p transform_tolerance:=0.01 \
  -p min_height:=-1.00 \
  -p max_height:=2.00 \
  -p angle_min:=-3.14159 \
  -p angle_max:=3.14159 \
  -p angle_increment:=0.0087 \
  -p scan_time:=0.1 \
  -p range_min:=0.2 \
  -p range_max:=30.0 \
  -p use_inf:=true > /dev/null 2>&1 &
sleep 2

echo "=== 6. 启动 SLAM 2D 建图引擎 (/map) ==="
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/wjzn/slam_toolbox_params.yaml > /tmp/slam.log 2>&1 &
sleep 3

echo "==================================================="
echo "🎉 所有硬件、3D车体模型与 SLAM 建图节点已在后台静默流畅运行！"
echo "📷 实时画面监视网址: http://192.168.1.108:8080"
echo "💡 按 Ctrl + C 随时一键秒停所有后台节点！"
echo "==================================================="

# 保持前台监控
python3 /home/wjzn/cam_web.py

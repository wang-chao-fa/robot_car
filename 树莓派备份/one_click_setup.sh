#!/bin/bash
set -e

echo "=== 1. 配置 Netplan 静态 IP (Wi-Fi: 192.168.1.108, USB网卡: 192.168.0.110) ==="
cat << 'NETPLAN_EOF' | sudo tee /etc/netplan/50-cloud-init.yaml
network:
  version: 2
  ethernets:
    eth0:
      optional: true
      dhcp4: true
    usb_eth:
      match:
        name: "enx*"
      optional: true
      dhcp4: false
      addresses:
        - 192.168.0.110/24
        - 192.168.0.10/24
  wifis:
    wlan0:
      optional: true
      dhcp4: false
      addresses:
        - 192.168.1.108/24
      routes:
        - to: default
          via: 192.168.1.1
      nameservers:
        addresses:
          - 114.114.114.114
          - 8.8.8.8
      regulatory-domain: "CN"
      access-points:
        "WJZN-5G":
          auth:
            key-management: "psk"
            password: "ba49feea32e8d69932989942e0aa39ab5db248fa83191fef86fea2297ccc1086"
NETPLAN_EOF
sudo netplan apply || true

echo "=== 2. 设置用户串口权限与网络组播路由 ==="
sudo usermod -aG dialout $USER || true
sudo ufw disable || true
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev wlan0 2>/dev/null || true

echo "=== 3. 写入环境变量到 ~/.bashrc ==="
if ! grep -q "ROS_DOMAIN_ID=0" ~/.bashrc; then
  echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
  echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
  echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

echo "=== 4. 一键安装全套 C++ 编译与 ROS 2 依赖包 ==="
sudo apt update
sudo apt install -y build-essential g++ gcc cmake libpcap-dev \
  ros-jazzy-slam-toolbox ros-jazzy-pointcloud-to-laserscan ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro ros-jazzy-urdf ros-jazzy-sensor-msgs-py python3-colcon-common-extensions \
  ros-jazzy-camera-info-manager ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-diagnostic-updater \
  ros-jazzy-image-publisher ros-jazzy-image-proc ros-jazzy-image-pipeline ros-jazzy-laser-geometry \
  ros-jazzy-pcl-conversions ros-jazzy-pcl-ros ros-jazzy-pcl-msgs

echo "=== 5. 生成 SLAM 参数配置文件 ~/slam_toolbox_params.yaml ==="
cat << 'SLAM_EOF' > ~/slam_toolbox_params.yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    mode: mapping

    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan

    resolution: 0.05
    max_laser_range: 20.0
    min_laser_range: 0.25
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    enable_interactive_mode: true
SLAM_EOF

echo "=== 6. 全自动编译工作空间 ==="
if [ -d ~/ros2_ws ]; then
  cd ~/ros2_ws
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install
fi

echo "🎉🎉🎉 全套环境一键安装配置与编译完成！"

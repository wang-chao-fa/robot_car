#!/bin/bash
echo "🛑 正在一键关停所有机器人节点与后台服务..."
sudo pkill -9 -f "base_control_node|yesense|vanjee_ros2_driver_node|pointcloud_to_laserscan_node|async_slam_toolbox_node|robot_state_publisher|cam_web.py|component_container|imu_tf_broadcaster" 2>/dev/null || true
echo "✅ 所有进程已秒级关停，硬件资源完全释放！"

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import struct
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

class VanJee719CDriver(Node):
    def __init__(self):
        super().__init__('vanjee_719c_driver')
        
        # 声明 ROS 2 参数
        self.declare_parameter('port', 58587)
        self.declare_parameter('host', '')
        self.declare_parameter('frame_id', 'laser')
        
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # 创建点云发布者
        self.pub = self.create_publisher(PointCloud2, '/vanjee_pointcloud', 10)
        
        # 创建 UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((self.host, self.port))
            self.get_logger().info(f"成功绑定端口 {self.port}，正在等待 WLR-719C 激光雷达数据...")
        except Exception as e:
            self.get_logger().error(f"绑定端口失败: {e}. 请检查是否有其他程序占用了该端口。")
            return
            
        # WLR-719C 的垂直夹角依次为：-10°，-5°，0°，0.3°
        self.vertical_angles = [
            math.radians(-10.0),
            math.radians(-5.0),
            math.radians(0.0),
            math.radians(0.3)
        ]
        
        # 点云缓存区
        self.points_buffer = []
        
        # 创建定时器快速读取 Socket 数据（1ms 触发一次）
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        try:
            # 设置非阻塞接收
            self.sock.settimeout(0.001)
            # 接收 UDP 数据包，雷达发送的 UDP 负载长度为 1384 字节
            data, addr = self.sock.recvfrom(2048)
            if len(data) != 1384:
                return
            
            # 校验帧头是否为 0xFFAA
            if data[0:2] != b'\xff\xaa':
                return
                
            bank_id = data[18] # 1 ~ 16
            scan_data = data[21:21+1350]
            
            for i in range(450):
                point_offset = i * 3
                intensity = scan_data[point_offset] & 0x7f
                dist_msb = scan_data[point_offset + 1]
                dist_lsb = scan_data[point_offset + 2]
                distance_mm = (dist_msb << 8) | dist_lsb
                distance_m = distance_mm / 1000.0
                
                # 过滤无效数据
                if distance_m < 0.05 or distance_m > 40.0:
                    continue
                
                h_step = i // 4
                v_idx = i % 4
                
                h_angle_deg = (bank_id - 1) * 22.5 + h_step * 0.2
                h_angle_rad = math.radians(h_angle_deg)
                v_angle_rad = self.vertical_angles[v_idx]
                
                cos_v = math.cos(v_angle_rad)
                x = distance_m * cos_v * math.cos(h_angle_rad)
                y = distance_m * cos_v * math.sin(h_angle_rad)
                z = distance_m * math.sin(v_angle_rad)
                
                self.points_buffer.append([x, y, z, float(intensity)])
                
            # 当接收完最后一个 BANK 时，打包并发布整圈点云
            if bank_id == 16:
                if self.points_buffer:
                    header = Header()
                    header.stamp = self.get_clock().now().to_msg()
                    header.frame_id = self.frame_id
                    
                    fields = [
                        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                    ]
                    
                    pc2_msg = pc2.create_cloud(header, fields, self.points_buffer)
                    self.pub.publish(pc2_msg)
                
                # 清空缓存
                self.points_buffer = []
                
        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().warn(f"解析数据包发生异常: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VanJee719CDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

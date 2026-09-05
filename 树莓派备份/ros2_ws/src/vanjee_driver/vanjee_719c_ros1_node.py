#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import struct
import math
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

def main():
    rospy.init_node('vanjee_719c_driver', anonymous=True)
    
    port = rospy.get_param('~port', 58587)
    host = rospy.get_param('~host', '') # 留空表示监听所有网卡
    frame_id = rospy.get_param('~frame_id', 'laser')
    topic_name = rospy.get_param('~topic', '/vanjee_pointcloud')
    
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=10)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((host, port))
        rospy.loginfo("成功绑定端口 %d，正在等待 WLR-719C 激光雷达数据...", port)
    except Exception as e:
        rospy.logerr("绑定端口失败: %s. 请检查是否有其他程序占用了该端口。", e)
        return

    vertical_angles = [
        math.radians(-10.0),
        math.radians(-5.0),
        math.radians(0.0),
        math.radians(0.3)
    ]
    
    points_buffer = []

    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(2048)
            if len(data) != 1384:
                continue
            
            if data[0:2] != b'\xff\xaa':
                continue
                
            bank_id = data[18]
            scan_data = data[21:21+1350]
            
            for i in range(450):
                point_offset = i * 3
                intensity = scan_data[point_offset] & 0x7f
                dist_msb = scan_data[point_offset + 1]
                dist_lsb = scan_data[point_offset + 2]
                distance_mm = (dist_msb << 8) | dist_lsb
                distance_m = distance_mm / 1000.0
                
                if distance_m < 0.05 or distance_m > 40.0:
                    continue
                
                h_step = i // 4
                v_idx = i % 4
                
                h_angle_deg = (bank_id - 1) * 22.5 + h_step * 0.2
                h_angle_rad = math.radians(h_angle_deg)
                v_angle_rad = vertical_angles[v_idx]
                
                cos_v = math.cos(v_angle_rad)
                x = distance_m * cos_v * math.cos(h_angle_rad)
                y = distance_m * cos_v * math.sin(h_angle_rad)
                z = distance_m * math.sin(v_angle_rad)
                
                points_buffer.append([x, y, z, float(intensity)])
                
            if bank_id == 16:
                if points_buffer:
                    header_msg = Header()
                    header_msg.stamp = rospy.Time.now()
                    header_msg.frame_id = frame_id
                    
                    fields = [
                        PointField('x', 0, PointField.FLOAT32, 1),
                        PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1),
                        PointField('intensity', 12, PointField.FLOAT32, 1),
                    ]
                    
                    pc2_msg = pc2.create_cloud(header_msg, fields, points_buffer)
                    pub.publish(pc2_msg)
                    
                points_buffer = []
                
        except socket.timeout:
            continue
        except Exception as e:
            rospy.logwarn("解析数据包时发生异常: %s", e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

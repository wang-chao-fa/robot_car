#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import math

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class BaseControlNode(Node):
    def __init__(self):
        super().__init__('base_control_node')

        # 1. 声明与读取参数 (将 base_frame 修正为 base_footprint)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        port_name = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # 2. 初始化串口
        try:
            self.ser = serial.Serial(port_name, baudrate, timeout=0.05)
            self.get_logger().info(f'成功打开串口: {port_name}, 波特率: {baudrate}')
        except Exception as e:
            self.get_logger().warn(f'无法打开串口 {port_name}: {e}，启用虚构/保底模式')
            self.ser = None

        # 3. ROS 2 发布者与广播器
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 4. ROS 2 订阅者 (/cmd_vel)
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 5. 状态变量与定时器
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = self.get_clock().now()

        # 50Hz 持续高频定时器 (20ms)
        self.timer = self.create_timer(0.02, self.read_serial_timer_callback)

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        send_str = f"$CMD,{v:.3f},{w:.3f}\r\n"
        if self.ser:
            try:
                self.ser.write(send_str.encode('utf-8'))
            except Exception as e:
                pass

    def read_serial_timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if self.ser and self.ser.in_waiting:
            try:
                lines = self.ser.read_all().decode('utf-8', errors='ignore').split('\n')
                for line in lines:
                    line = line.strip()
                    if line.startswith('$ODOM'):
                        parts = line.split(',')
                        if len(parts) == 3:
                            self.v = float(parts[1])
                            self.w = float(parts[2])
                            self.x += (self.v * math.cos(self.th)) * dt
                            self.y += (self.v * math.sin(self.th)) * dt
                            self.th += self.w * dt
            except Exception as e:
                pass

        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        # 高频广播 odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = BaseControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

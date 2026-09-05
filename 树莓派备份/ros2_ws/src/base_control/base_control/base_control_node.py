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

        # ---------------- 1. 声明与读取参数 ----------------
        self.declare_parameter('port', '/dev/ttyUSB0')  # 串口设备名 (按实际修改, 如 /dev/ttyACM0)
        self.declare_parameter('baudrate', 115200)      # 波特率
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        port_name = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # ---------------- 2. 初始化串口 ----------------
        try:
            self.ser = serial.Serial(port_name, baudrate, timeout=0.05)
            self.get_logger().info(f'成功打开串口: {port_name}, 波特率: {baudrate}')
        except serial.SerialException as e:
            self.get_logger().error(f'无法打开串口 {port_name}: {e}')
            exit(1)

        # ---------------- 3. ROS 2 发布者与广播器 ----------------
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- 4. ROS 2 订阅者 (/cmd_vel) ----------------
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ---------------- 5. 状态变量与定时器 ----------------
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # 50Hz 频率 (20ms) 轮询读取串口上行速度数据
        self.timer = self.create_timer(0.02, self.read_serial_timer_callback)

    def cmd_vel_callback(self, msg: Twist):
        """ 接收 /cmd_vel 控制指令并发给 STM32 """
        v = msg.linear.x
        w = msg.angular.z
        # 打包格式: $CMD,<v>,<w>\r\n (例如: $CMD,0.250,-0.100\r\n)
        send_str = f"$CMD,{v:.3f},{w:.3f}\r\n"
        try:
            self.ser.write(send_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"串口下发指令失败: {e}")

    def read_serial_timer_callback(self):
        """ 串口定时轮询: 读取 STM32 发送的 $ODOM,v,w\r\n 帧并积分发布 Odometry """
        if not self.ser.in_waiting:
            return

        try:
            lines = self.ser.read_all().decode('utf-8', errors='ignore').split('\n')
            for line in lines:
                line = line.strip()
                if line.startswith('$ODOM'):
                    parts = line.split(',')
                    if len(parts) == 3:
                        v = float(parts[1])  # 中心线速度 (m/s)
                        w = float(parts[2])  # 旋转角速度 (rad/s)

                        # ------- 积分合成位姿 -------
                        current_time = self.get_clock().now()
                        dt = (current_time - self.last_time).nanoseconds / 1e9
                        self.last_time = current_time

                        delta_x = (v * math.cos(self.th)) * dt
                        delta_y = (v * math.sin(self.th)) * dt
                        delta_th = w * dt

                        self.x += delta_x
                        self.y += delta_y
                        self.th += delta_th

                        # 偏航角 Yaw 转 Quaternion
                        qz = math.sin(self.th / 2.0)
                        qw = math.cos(self.th / 2.0)

                        # ------- 发布 TF 坐标变换 (odom -> base_link) -------
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

                        # ------- 发布 /odom 话题 -------
                        odom = Odometry()
                        odom.header.stamp = current_time.to_msg()
                        odom.header.frame_id = self.odom_frame
                        odom.child_frame_id = self.base_frame

                        odom.pose.pose.position.x = self.x
                        odom.pose.pose.position.y = self.y
                        odom.pose.pose.position.z = 0.0
                        odom.pose.pose.orientation.z = qz
                        odom.pose.pose.orientation.w = qw

                        odom.twist.twist.linear.x = v
                        odom.twist.twist.angular.z = w

                        self.odom_pub.publish(odom)
        except Exception as e:
            self.get_logger().warn(f"解析串口数据异常: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BaseControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

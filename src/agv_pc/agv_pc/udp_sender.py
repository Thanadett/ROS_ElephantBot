#!/usr/bin/env python3
"""
udp_sender.py  — 660610822 Final Project
Node บน PC (ROS2 Jazzy, ROS_DOMAIN_ID=1)

Subscribe /cmd_vel_command (จาก motion_manager)
แล้วส่งผ่าน UDP ไปยัง Robot (ROS2 Galactic)

ใช้ UDP เพราะ Jazzy ↔ Galactic DDS ไม่ compatible กัน
"""

import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class UdpSender(Node):

    def __init__(self):
        super().__init__('udp_sender')

        self.declare_parameter('robot_ip',  '127.0.0.1')
        self.declare_parameter('udp_port',  15000)

        self.robot_ip = self.get_parameter('robot_ip').value
        self.udp_port = self.get_parameter('udp_port').value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.create_subscription(Twist, '/cmd_vel_command', self._cb, 10)

        self.get_logger().info(
            f'[udp_sender] /cmd_vel_command → UDP {self.robot_ip}:{self.udp_port}')

    def _cb(self, msg: Twist):
        data = f"{msg.linear.x} {msg.linear.y} {msg.angular.z}"
        self.sock.sendto(data.encode(), (self.robot_ip, self.udp_port))


def main(args=None):
    rclpy.init(args=args)
    node = UdpSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
udp_gateway.py  — 660610822 Final Project
Runs on PC (ROS2 Jazzy, ROS_DOMAIN_ID=1)

แทนที่ udp_sender.py เดิม — เพิ่มการรับ /scan กลับจากหุ่นด้วย

Flow ส่ง:  /cmd_vel_safe → UDP:15000 → Robot
Flow รับ:  Robot UDP:15001 → /scan  (ให้ lidar_guard ใช้ได้บน PC)
"""

import socket
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class UdpGateway(Node):

    def __init__(self):
        super().__init__('udp_gateway')

        self.declare_parameter('robot_ip',  '172.20.10.3')
        self.declare_parameter('cmd_port',  15000)
        self.declare_parameter('scan_port', 15001)

        self.robot_ip  = self.get_parameter('robot_ip').value
        self.cmd_port  = self.get_parameter('cmd_port').value
        self.scan_port = self.get_parameter('scan_port').value

        # ── ส่ง CMD ──────────────────────────────────────────────
        self.sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # lidar_guard กรองแล้ว → /cmd_vel_safe → ส่งไปหุ่น
        self.create_subscription(Twist, '/cmd_vel_safe', self._cmd_cb, 10)

        # ── รับ SCAN ─────────────────────────────────────────────
        self.scan_pub = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_recv.bind(('0.0.0.0', self.scan_port))
        threading.Thread(target=self._scan_loop, daemon=True).start()

        self.get_logger().info(
            f'[udp_gateway] /cmd_vel_safe → UDP {self.robot_ip}:{self.cmd_port}')
        self.get_logger().info(
            f'[udp_gateway] UDP:{self.scan_port} → /scan')

    def _cmd_cb(self, msg: Twist):
        data = f'{msg.linear.x:.3f} {msg.linear.y:.3f} {msg.angular.z:.3f}'
        self.sock_send.sendto(data.encode(), (self.robot_ip, self.cmd_port))

    def _scan_loop(self):
        self.sock_recv.settimeout(1.0)
        while rclpy.ok():
            try:
                data, _ = self.sock_recv.recvfrom(65535)
                raw = [float(v) for v in data.decode().split(',')]
                if not raw:
                    continue
                n   = len(raw)
                msg = LaserScan()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = 'laser_frame'
                msg.angle_min       = 0.0
                msg.angle_max       = 2.0 * math.pi
                msg.angle_increment = (2.0 * math.pi) / n
                msg.range_min       = 0.12
                msg.range_max       = 8.0
                msg.ranges          = raw
                self.scan_pub.publish(msg)
            except socket.timeout:
                pass
            except Exception as e:
                self.get_logger().warn(f'Scan recv error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UdpGateway()
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
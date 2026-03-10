#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import socket
import threading

PC_IP     = '172.20.10.4'  # ← แก้ให้ตรงกับ IP ของ PC
CMD_PORT  = 15000
SCAN_PORT = 15001


class UdpReceiver(Node):

    def __init__(self):
        super().__init__('udp_receiver')

        # รับ CMD UDP → publish /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_cmd.bind(('0.0.0.0', CMD_PORT))

        # subscribe /scan → ส่ง UDP scan ไป PC
        self.sock_scan = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        threading.Thread(target=self._cmd_loop, daemon=True).start()

        self.get_logger().info(f'[udp_receiver] CMD UDP:{CMD_PORT} → /cmd_vel')
        self.get_logger().info(f'[udp_receiver] /scan → UDP {PC_IP}:{SCAN_PORT}')

    def _cmd_loop(self):
        while rclpy.ok():
            try:
                data, _ = self.sock_cmd.recvfrom(1024)
                x, y, z = map(float, data.decode().split())
                msg = Twist()
                msg.linear.x  = x
                msg.linear.y  = y
                msg.angular.z = z
                self.cmd_pub.publish(msg)
            except Exception:
                pass

    def _scan_cb(self, msg: LaserScan):
        try:
            data = ','.join([f'{r:.3f}' for r in msg.ranges])
            self.sock_scan.sendto(data.encode(), (PC_IP, SCAN_PORT))
        except Exception as e:
            self.get_logger().error(f'Send scan error: {e}')


def main():
    rclpy.init()
    rclpy.spin(UdpReceiver())
    rclpy.shutdown()

if _name_ == '_main_':
    main()
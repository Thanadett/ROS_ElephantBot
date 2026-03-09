#!/usr/bin/env python3
"""
Subscribes to /cmd_vel_command (published by motion_manager)
and forwards the data to the robot over UDP port 15000.

No changes from original — motion_manager outputs to /cmd_vel_command
so this file works unchanged.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket

UDP_IP   = '127.0.0.1'  
UDP_PORT = 15000


class CmdVelToUDP(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_udp')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sub  = self.create_subscription(
            Twist, '/cmd_vel_command', self.cb, 10)
        self.get_logger().info(f'Forwarding /cmd_vel_command → UDP {UDP_IP}:{UDP_PORT}')

    def cb(self, msg):
        data = f"{msg.linear.x} {msg.linear.y} {msg.angular.z}"
        self.sock.sendto(data.encode(), (UDP_IP, UDP_PORT))


def main():
    rclpy.init()
    rclpy.spin(CmdVelToUDP())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

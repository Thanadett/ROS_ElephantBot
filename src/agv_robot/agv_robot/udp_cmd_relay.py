#!/usr/bin/env python3
"""
udp_cmd_relay.py  — 660610822 Final Project
Runs on ROBOT (ROS_DOMAIN_ID=0)

Replaces the original UDP_sender.py.
Receives Twist commands from PC over UDP and publishes to /cmd_vel_INPUT
so that lidar_guard can intercept them before they reach /cmd_vel.

Flow:
  PC (UDP_receiver) → UDP:15000 → HERE → /cmd_vel_input → lidar_guard → /cmd_vel
"""

import socket
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


UDP_IP   = '0.0.0.0'
UDP_PORT = 15000
TIMEOUT  = 0.5   # seconds without data → publish zero twist (safety)


class UdpCmdRelay(Node):

    def __init__(self):
        super().__init__('udp_cmd_relay')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publish to /cmd_vel_INPUT so lidar_guard can intercept
        self.pub = self.create_publisher(Twist, '/cmd_vel_input', qos)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.last_rx = time.time()

        # UDP receive thread
        self._rx_thread = threading.Thread(target=self._udp_loop, daemon=True)
        self._rx_thread.start()

        # Watchdog timer: zero out if no data for TIMEOUT seconds
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info("══ UDP Cmd Relay (660610822) ══")
        self.get_logger().info(f"   Listening on UDP {UDP_IP}:{UDP_PORT}")
        self.get_logger().info(f"   Publishes → /cmd_vel_input")
        self.get_logger().info(f"   Watchdog timeout: {TIMEOUT}s")

    def _udp_loop(self):
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                x, y, z = map(float, data.decode().split())
                msg = Twist()
                msg.linear.x  = x
                msg.linear.y  = y
                msg.angular.z = z
                self.pub.publish(msg)
                self.last_rx = time.time()
            except ValueError:
                self.get_logger().warn(f"Bad UDP packet: {data}")
            except Exception:
                pass

    def _watchdog(self):
        """Publish zero twist if no data received within TIMEOUT."""
        if time.time() - self.last_rx > TIMEOUT:
            self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = UdpCmdRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt → stopping")
        node.pub.publish(Twist())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

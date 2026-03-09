#!/usr/bin/env python3
"""
motion_manager.py  — 660610822 Final Project
Runs on PC (ROS_DOMAIN_ID=1)

Service server that acts as motion command gatekeeper.
- Subscribes to /hand/cmd_vel (from hand_controller)
- Subscribes to /system/cmd_vel (direct commands when override active)
- Provides service /motion/set_override to enable/disable human control
- Publishes final /cmd_vel_command to UDP bridge

When override is ENABLED:
  - Human hand commands are BLOCKED
  - Only /system/cmd_vel passes through
  - Service caller can also send direct velocity in the request

When override is DISABLED (default):
  - Hand commands pass through normally
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from agv_interfaces.srv import SetMotionOverride

PUBLISH_HZ = 20


class MotionManager(Node):

    def __init__(self):
        super().__init__('motion_manager')

        # ── Publishers ────────────────────────────────────────────
        # Final output to UDP bridge (→ robot)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_command', 10)
        # Status broadcast (UI / logging)
        self.status_pub = self.create_publisher(String, '/motion/status', 10)

        # ── Subscribers ───────────────────────────────────────────
        # Commands from hand controller
        self.create_subscription(Twist, '/hand/cmd_vel',
                                 self._hand_cmd_cb, 10)
        # Direct system commands (used when override is active)
        self.create_subscription(Twist, '/system/cmd_vel',
                                 self._system_cmd_cb, 10)

        # ── Service Server ────────────────────────────────────────
        self.srv = self.create_service(
            SetMotionOverride,
            '/motion/set_override',
            self._override_srv_cb)

        # ── State ─────────────────────────────────────────────────
        self.override_active  = False   # True = block human, system controls
        self.override_source  = ''
        self.hand_twist       = Twist()
        self.system_twist     = Twist()

        # ── Publish timer ─────────────────────────────────────────
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_loop)

        self.get_logger().info("══ Motion Manager (660610822) ══")
        self.get_logger().info("   Subscribes  ← /hand/cmd_vel")
        self.get_logger().info("   Subscribes  ← /system/cmd_vel")
        self.get_logger().info("   Publishes   → /cmd_vel_command")
        self.get_logger().info("   Service     : /motion/set_override")

    # ── Callbacks ────────────────────────────────────────────────
    def _hand_cmd_cb(self, msg: Twist):
        """Store latest hand command. Only forwarded when not overridden."""
        self.hand_twist = msg

    def _system_cmd_cb(self, msg: Twist):
        """Store latest system command. Only forwarded when override is active."""
        if self.override_active:
            self.system_twist = msg

    def _override_srv_cb(self, request: SetMotionOverride.Request,
                          response: SetMotionOverride.Response):
        """
        Service: /motion/set_override
        enable_override=True  → block human, take system control
        enable_override=False → restore human control

        Caller can optionally provide a direct velocity in the request
        which will be immediately applied if override is enabled.
        """
        prev = self.override_active
        self.override_active = request.enable_override
        self.override_source = request.requester if request.enable_override else ''

        # Apply direct velocity from service request if provided
        if request.enable_override:
            self.system_twist.linear.x  = request.direct_linear_x
            self.system_twist.linear.y  = request.direct_linear_y
            self.system_twist.angular.z = request.direct_angular_z
            self.get_logger().warn(
                f"[OVERRIDE ENABLED] requester='{request.requester}' "
                f"cmd=({request.direct_linear_x:.2f}, "
                f"{request.direct_linear_y:.2f}, "
                f"{request.direct_angular_z:.2f})")
        else:
            self.system_twist = Twist()   # clear system command
            if prev:
                self.get_logger().info(
                    f"[OVERRIDE DISABLED] human control restored")

        self._broadcast_status()

        response.success        = True
        response.override_active = self.override_active
        response.message = (
            f"Override {'ENABLED' if self.override_active else 'DISABLED'} "
            f"by '{request.requester}'"
        )
        return response

    # ── Publish loop ─────────────────────────────────────────────
    def _publish_loop(self):
        if self.override_active:
            cmd = self.system_twist
        else:
            cmd = self.hand_twist

        self.cmd_pub.publish(cmd)

    # ── Status broadcast ─────────────────────────────────────────
    def _broadcast_status(self):
        msg = String()
        msg.data = (
            f"override={'ON' if self.override_active else 'OFF'}"
            f"|source={self.override_source}"
        )
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt → stopping")
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

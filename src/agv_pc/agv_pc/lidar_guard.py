#!/usr/bin/env python3
"""
lidar_guard_pro.py
Advanced safety controller for AGV (ROS2 Jazzy)

Features
--------
• Predictive braking (velocity based stopping distance)
• Velocity adaptive safety zones
• Repulsive obstacle vector field
• Anti oscillation (zone hysteresis)
• 40Hz safety loop
• Side corridor centering
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from agv_interfaces.msg import ObstacleAlert
from agv_interfaces.msg import ObstacleInfo


# ═══════════════════════════════════════════════════════
# Robot parameters
# ═══════════════════════════════════════════════════════

HALF_LENGTH = 0.311 / 2
HALF_WIDTH  = 0.190 / 2

LIDAR_OFFSET_X = 0.070
LIDAR_OFFSET_Y = 0.0

SELF_FILTER = 0.018


# ═══════════════════════════════════════════════════════
# Safety parameters
# ═══════════════════════════════════════════════════════

CLEAR_DIST     = 0.20
TARGET_DIST    = 0.15
EMERGENCY_DIST = 0.10

KP_FIELD = 0.8

MAX_DECEL = 0.8
SAFE_MARGIN = 0.05

ZONE_HYST = 0.02

AVOID_LIMIT = 0.12
MAX_SAFE_SPEED = 0.50

ALPHA = 0.6

CONTROL_HZ = 40


class LidarGuardPro(Node):

    def __init__(self):

        super().__init__('lidar_guard_pro')

        qos_scan = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_rel  = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, qos_scan)

        self.create_subscription(
            Twist, '/cmd_vel_command', self.cmd_cb, qos_rel)

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel_safe', qos_rel)

        self.alert_pub = self.create_publisher(
            ObstacleAlert, '/obstacle_alert', qos_rel)

        self.info_pub = self.create_publisher(
            ObstacleInfo, '/obstacle_info', qos_rel)

        self.latest_scan = None
        self.current_cmd = Twist()

        self.smooth_vx = 0.0
        self.smooth_vy = 0.0

        self.last_zone = "CLEAR"

        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)

        self.get_logger().info("════ LIDAR GUARD PRO ════")
        self.get_logger().info("Control loop: 40Hz")
        self.get_logger().info("Predictive braking enabled")
        self.get_logger().info("Vector field avoidance enabled")


    def scan_cb(self, msg):
        self.latest_scan = msg


    def cmd_cb(self, msg):
        self.current_cmd = msg


    def control_loop(self):

        if self.latest_scan is None:
            self.cmd_pub.publish(self.current_cmd)
            return

        scan = self.latest_scan

        raw_vx = 0.0
        raw_vy = 0.0

        left_min = float('inf')
        right_min = float('inf')

        min_eff = float('inf')
        min_angle = 0.0


        for i, dist in enumerate(scan.ranges):
            if not math.isfinite(dist) or dist < 0.05:
                continue

            angle = scan.angle_min + i * scan.angle_increment

            lx = dist * math.cos(angle)
            ly = dist * math.sin(angle)

            bx = lx + LIDAR_OFFSET_X
            by = ly + LIDAR_OFFSET_Y

            d = math.sqrt(bx*bx + by*by)

            ang = math.atan2(by, bx)

            cos_a = abs(math.cos(ang))
            sin_a = abs(math.sin(ang))

            edge = min(
                HALF_LENGTH / max(cos_a, 1e-6),
                HALF_WIDTH  / max(sin_a, 1e-6)
            )

            eff = d - edge

            if eff < SELF_FILTER:
                continue

            deg = (math.degrees(ang) + 360) % 360

            if eff < min_eff:
                min_eff = eff
                min_angle = deg

            if 80 <= deg <= 100:
                left_min = min(left_min, eff)

            elif 260 <= deg <= 280:
                right_min = min(right_min, eff)

            # repulsive vector field

            if eff < TARGET_DIST:

                d = max(eff, 0.02)

                rep = KP_FIELD * (1.0/d - 1.0/TARGET_DIST)

                raw_vx -= math.cos(ang) * rep
                raw_vy -= math.sin(ang) * rep


        # side centering

        if left_min < TARGET_DIST or right_min < TARGET_DIST:

            l = left_min if left_min < TARGET_DIST else TARGET_DIST
            r = right_min if right_min < TARGET_DIST else TARGET_DIST

            raw_vy = (l - r) * 0.8
            raw_vx = 0.0


        # smoothing

        self.smooth_vx = ALPHA * raw_vx + (1-ALPHA) * self.smooth_vx
        self.smooth_vy = ALPHA * raw_vy + (1-ALPHA) * self.smooth_vy


        # predictive braking

        v = abs(self.current_cmd.linear.x)

        stop_dist = (v*v) / (2*MAX_DECEL)

        dynamic_emergency = max(EMERGENCY_DIST, stop_dist + SAFE_MARGIN)

        dynamic_target = max(TARGET_DIST, stop_dist * 1.4)

        dynamic_clear = max(CLEAR_DIST, stop_dist * 2.0)


        # zone detection

        if min_eff < dynamic_emergency:
            zone = "EMERGENCY"

        elif min_eff < dynamic_target - ZONE_HYST:
            zone = "CRITICAL"

        elif min_eff < dynamic_clear - ZONE_HYST:
            zone = "WARNING"

        else:
            zone = "CLEAR"


        # log zone changes

        if zone != self.last_zone:

            dist_mm = min_eff * 1000

            if zone == "CLEAR":
                self.get_logger().info("CLEAR path")

            elif zone == "WARNING":
                self.get_logger().warn(
                    f"WARNING {dist_mm:.0f}mm")

            elif zone == "CRITICAL":
                self.get_logger().warn(
                    f"CRITICAL {dist_mm:.0f}mm")

            elif zone == "EMERGENCY":
                self.get_logger().error(
                    f"EMERGENCY {dist_mm:.0f}mm")

            self.last_zone = zone


        # publish obstacle

        if zone != "CLEAR":
            self.publish_obstacle(min_eff, min_angle, zone)


        # actions

        if zone == "EMERGENCY":

            self.cmd_pub.publish(Twist())
            return


        if zone == "CRITICAL":

            cmd = Twist()

            cmd.linear.x = max(min(self.smooth_vx, AVOID_LIMIT), -AVOID_LIMIT)
            cmd.linear.y = max(min(self.smooth_vy, AVOID_LIMIT), -AVOID_LIMIT)

            self.cmd_pub.publish(cmd)
            return


        if zone == "WARNING":

            span = dynamic_clear - dynamic_target

            scale = (min_eff - dynamic_target) / span
            scale = max(0.0, min(1.0, scale))

            scale = scale * scale

            cmd = Twist()

            cmd.linear.x = self.current_cmd.linear.x * scale
            cmd.linear.y = self.current_cmd.linear.y * scale
            cmd.angular.z = self.current_cmd.angular.z * scale

            cmd.linear.x = max(min(cmd.linear.x, 0.08), -0.08)

            self.cmd_pub.publish(cmd)
            return


        cmd = self.current_cmd

        cmd.linear.x = max(min(cmd.linear.x, MAX_SAFE_SPEED), -MAX_SAFE_SPEED)

        self.cmd_pub.publish(cmd)


    def publish_obstacle(self, dist, angle, zone):

        now = self.get_clock().now().to_msg()

        alert = ObstacleAlert()

        alert.header.stamp = now
        alert.header.frame_id = "base_link"

        alert.obstacle_present = True
        alert.angle_deg = angle
        alert.distance_mm = dist * 1000
        alert.zone = zone

        self.alert_pub.publish(alert)


        info = ObstacleInfo()

        info.header.stamp = now
        info.distance = dist * 1000
        info.angle = angle
        info.warning = True
        info.emergency = zone in ("CRITICAL", "EMERGENCY")

        self.info_pub.publish(info)


def main(args=None):

    rclpy.init(args=args)

    node = LidarGuardPro()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
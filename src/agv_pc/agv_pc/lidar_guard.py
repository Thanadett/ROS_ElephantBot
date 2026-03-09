#!/usr/bin/env python3
"""
lidar_guard.py — 660610822 Final Project
Runs on PC (ROS2 Jazzy, ROS_DOMAIN_ID=1)

Flow:
  motion_manager → /cmd_vel_command
  lidar_guard    → /cmd_vel_safe
  udp_gateway    → UDP → Robot

การคำนวณระยะ:
  - แปลงจุด LiDAR → frame หุ่น (รวม lidar_offset)
  - คำนวณ effective_dist = ระยะจากกึ่งกลาง - ระยะถึงขอบหุ่น
  - P-controller ผลักออกเมื่อ effective_dist < target_dist (120mm)
  - Low-pass filter ทำให้การเคลื่อนที่นุ่มนวล
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from agv_interfaces.msg import ObstacleAlert
from agv_interfaces.msg import ObstacleInfo


# ══ Thresholds ══════════════════════════════════════════════════
TARGET_DIST    = 0.120   # 120mm: ระยะรักษา (error=0 → หยุดผลัก)
ALERT_DIST     = 0.150   # 150mm: เริ่มแจ้งเตือน WARNING
STOP_THRESHOLD = 0.050   # 50mm:  ฉุกเฉิน PINCH stop

KP             = 0.5     # P-Gain avoidance
ALPHA          = 0.6     # Low-pass filter coefficient
AVOID_LIMIT    = 0.115   # m/s สูงสุดของ avoidance velocity
PUBLISH_HZ     = 20

# ══ Robot Body (myAGV Pro) ══════════════════════════════════════
HALF_LENGTH    = 0.311 / 2   # ครึ่งความยาวหุ่น (m)
HALF_WIDTH     = 0.190 / 2   # ครึ่งความกว้างหุ่น (m)
LIDAR_OFFSET_X = 0.070       # LiDAR อยู่ข้างหน้ากึ่งกลาง 70mm
LIDAR_OFFSET_Y = 0.000
SELF_FILTER    = 0.018       # ละเว้นจุดที่ effective_dist < 18mm (ตัวหุ่นเอง)


class LidarGuard(Node):

    def __init__(self):
        super().__init__('lidar_guard')

        qos_scan = QoSProfile(depth=5,  reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_rel  = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(LaserScan, '/scan',           self._scan_cb, qos_scan)
        self.create_subscription(Twist,     '/cmd_vel_command',self._cmd_cb,  qos_rel)

        # ── Publishers ───────────────────────────────────────────
        self.cmd_pub   = self.create_publisher(Twist,         '/cmd_vel_safe',   qos_rel)
        self.alert_pub = self.create_publisher(ObstacleAlert, '/obstacle_alert', qos_rel)
        self.info_pub  = self.create_publisher(ObstacleInfo,  '/obstacle_info',  qos_rel)

        # ── State ────────────────────────────────────────────────
        self.current_cmd  = Twist()
        self.latest_scan  = None
        self.smooth_vx    = 0.0
        self.smooth_vy    = 0.0
        self.avoid_active = False

        self.create_timer(1.0 / PUBLISH_HZ, self._publish_loop)

        self.get_logger().info("══ Lidar Guard (660610822) ══")
        self.get_logger().info("   ← /scan  ← /cmd_vel_command")
        self.get_logger().info("   → /cmd_vel_safe  → /obstacle_alert  → /obstacle_info")

    # ── Callbacks ────────────────────────────────────────────────
    def _scan_cb(self, msg): self.latest_scan = msg
    def _cmd_cb(self, msg):  self.current_cmd = msg

    # ── Main loop ─────────────────────────────────────────────────
    def _publish_loop(self):
        if self.latest_scan is None:
            self.cmd_pub.publish(self.current_cmd)
            return

        scan = self.latest_scan
        raw_vx = 0.0
        raw_vy = 0.0

        left_min  = float('inf')   # ~90°  ด้านซ้าย
        right_min = float('inf')   # ~270° ด้านขวา
        min_eff   = float('inf')   # obstacle ที่ใกล้สุด
        min_angle_body_deg = 0.0   # มุมของ obstacle นั้นในระบบ body frame (CW จากหน้า)

        for i, raw_dist in enumerate(scan.ranges):

            if not math.isfinite(raw_dist) or raw_dist <= 0.05:
                continue

            # ── แปลงจุดเป็น body frame ─────────────────────────
            angle_lidar = scan.angle_min + i * scan.angle_increment
            lx = raw_dist * math.cos(angle_lidar)
            ly = raw_dist * math.sin(angle_lidar)

            bx = lx + LIDAR_OFFSET_X   # body frame (x=หน้า, y=ซ้าย)
            by = ly + LIDAR_OFFSET_Y

            dist_center   = math.sqrt(bx**2 + by**2)
            angle_body    = math.atan2(by, bx)     # rad, CCW จากหน้า

            # ── คำนวณระยะถึงขอบหุ่น ────────────────────────────
            cos_a = abs(math.cos(angle_body))
            sin_a = abs(math.sin(angle_body))
            dist_to_edge = min(HALF_LENGTH / max(cos_a, 1e-6),
                               HALF_WIDTH  / max(sin_a, 1e-6))

            effective_dist = dist_center - dist_to_edge

            if effective_dist < SELF_FILTER:
                continue   # จุดของตัวหุ่นเอง ละเว้น

            # ── มุมในระบบ CW จากหน้า (สำหรับ dashboard) ────────
            deg_body     = (math.degrees(angle_body)  + 360) % 360   # CCW
            deg_body_cw  = (-math.degrees(angle_body) + 360) % 360   # CW จากหน้า

            # ── track obstacle ใกล้สุด ──────────────────────────
            if effective_dist < min_eff:
                min_eff          = effective_dist
                min_angle_body_deg = deg_body_cw

            # ── เก็บค่าด้านซ้าย/ขวา (80–100°, 260–280°) ────────
            if 80.0 <= deg_body <= 100.0:
                left_min  = min(left_min,  effective_dist)
            elif 260.0 <= deg_body <= 280.0:
                right_min = min(right_min, effective_dist)

            # ── P-controller: ผลักเมื่อใกล้กว่า 120mm ───────────
            if effective_dist < TARGET_DIST:
                error  = TARGET_DIST - effective_dist
                raw_vx -= math.cos(angle_body) * error * KP
                raw_vy -= math.sin(angle_body) * error * KP

        # ── Side centering logic ─────────────────────────────────
        side_vy       = 0.0
        in_side_ctrl  = False

        if left_min < TARGET_DIST or right_min < TARGET_DIST:
            in_side_ctrl = True
            l_val = left_min  if left_min  < TARGET_DIST else TARGET_DIST
            r_val = right_min if right_min < TARGET_DIST else TARGET_DIST
            side_vy = (l_val - r_val) * KP
            raw_vx  = 0.0   # ล็อก X เมื่ออยู่ใน side control

        target_vx = 0.0    if in_side_ctrl else raw_vx
        target_vy = side_vy if in_side_ctrl else raw_vy

        # ── Low-pass filter ──────────────────────────────────────
        self.smooth_vx = ALPHA * target_vx + (1.0 - ALPHA) * self.smooth_vx
        self.smooth_vy = ALPHA * target_vy + (1.0 - ALPHA) * self.smooth_vy

        # dead-band: ถ้าใกล้ถึง 120mm แล้ว ส่ง 0 เลย
        if abs(self.smooth_vx) < 0.005: self.smooth_vx = 0.0
        if abs(self.smooth_vy) < 0.005: self.smooth_vy = 0.0

        # ── Determine zone ───────────────────────────────────────
        if min_eff < TARGET_DIST:
            zone = "CRITICAL"
        elif min_eff < ALERT_DIST:
            zone = "WARNING"
        else:
            zone = "CLEAR"

        # ── Publish obstacle info ────────────────────────────────
        if zone != "CLEAR":
            self._publish_obstacle(min_eff, min_angle_body_deg, zone)

        # ── Emergency PINCH stop ─────────────────────────────────
        if left_min < STOP_THRESHOLD and right_min < STOP_THRESHOLD:
            self.get_logger().warn("PINCHED! Emergency Stop.")
            self.cmd_pub.publish(Twist())   # zero
            self.smooth_vx = 0.0
            self.smooth_vy = 0.0
            return

        # ── Publish /cmd_vel_safe ────────────────────────────────
        if zone == "CRITICAL":
            if not self.avoid_active:
                self.get_logger().warn(f"CRITICAL {min_eff*1000:.0f}mm @ {min_angle_body_deg:.0f}°")
            self.avoid_active = True

            cmd = Twist()
            cmd.linear.x = max(min(self.smooth_vx, AVOID_LIMIT), -AVOID_LIMIT)
            cmd.linear.y = max(min(self.smooth_vy, AVOID_LIMIT), -AVOID_LIMIT)
            self.cmd_pub.publish(cmd)

        else:
            if self.avoid_active:
                self.get_logger().info("Path clear — resuming")
            self.avoid_active = False
            self.smooth_vx = 0.0
            self.smooth_vy = 0.0
            self.cmd_pub.publish(self.current_cmd)

    # ── Obstacle publishers ───────────────────────────────────────
    def _publish_obstacle(self, dist_m, angle_cw_deg, zone):
        now = self.get_clock().now().to_msg()

        alert = ObstacleAlert()
        alert.header.stamp    = now
        alert.header.frame_id = 'base_link'
        alert.obstacle_present = True
        alert.angle_deg   = angle_cw_deg
        alert.distance_mm = dist_m * 1000.0
        alert.zone        = zone
        if zone == "CRITICAL":
            # คำนวณ avoidance vector สำหรับ log
            angle_rad = math.radians((360.0 - angle_cw_deg) % 360.0)  # CW → CCW rad
            alert.avoid_linear_x = -math.cos(angle_rad) * AVOID_LIMIT
            alert.avoid_linear_y = -math.sin(angle_rad) * AVOID_LIMIT
        self.alert_pub.publish(alert)

        info = ObstacleInfo()
        info.header.stamp = now
        info.distance  = alert.distance_mm
        info.angle     = alert.angle_deg
        info.warning   = True
        info.emergency = (zone == "CRITICAL")
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = LidarGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
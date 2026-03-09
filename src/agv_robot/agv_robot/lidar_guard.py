#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from agv_interfaces.msg import ObstacleAlert
from agv_interfaces.msg import ObstacleInfo

# ── Thresholds ───────────────────────────────────────────────────
WARN_DIST_M     = 0.150   # 150 mm — trigger warning
CRIT_DIST_M     = 0.120   # 120 mm — trigger avoidance
AVOID_SPEED     = 0.12    # m/s max avoidance velocity
PUBLISH_HZ      = 20
ANGLE_SECTOR_DEG = 10     # group scan into sectors for cleaner reporting


class LidarGuard(Node):

    def __init__(self):
        super().__init__('lidar_guard')

        qos_best = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_rel  = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # ── Subscribers ───────────────────────────────────────────
        # LiDAR scan from robot hardware (via ROS1 bridge)
        self.create_subscription(LaserScan, '/scan',
                                 self._scan_cb, qos_best)
        # Motion commands coming from UDP relay
        self.create_subscription(Twist, '/cmd_vel_input',
                                 self._cmd_cb, qos_best)

        # ── Publishers ────────────────────────────────────────────
        # Final safe velocity to robot motor controller (via ROS1 bridge)
        self.cmd_pub   = self.create_publisher(Twist,         '/cmd_vel',      qos_best)
        self.alert_pub = self.create_publisher(ObstacleAlert, '/obstacle_alert', qos_rel)
        # ObstacleInfo → UDP bridge → PC dashboard
        self.info_pub  = self.create_publisher(ObstacleInfo,  '/obstacle_info',  qos_rel)

        # ── State ─────────────────────────────────────────────────
        self.current_cmd      = Twist()
        self.latest_scan      = None
        self.avoidance_active = False

        # ── Publish timer ─────────────────────────────────────────
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_loop)

        self.get_logger().info("══ Lidar Guard (660610822) ══")
        self.get_logger().info("   Subscribes  ← /scan")
        self.get_logger().info("   Subscribes  ← /cmd_vel_input")
        self.get_logger().info("   Publishes   → /cmd_vel")
        self.get_logger().info("   Publishes   → /obstacle_alert")
        self.get_logger().info(f"   WARN  threshold : {WARN_DIST_M*1000:.0f} mm")
        self.get_logger().info(f"   CRIT  threshold : {CRIT_DIST_M*1000:.0f} mm")

    # ── Callbacks ────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _cmd_cb(self, msg: Twist):
        self.current_cmd = msg

    # ── Main publish loop ─────────────────────────────────────────
    def _publish_loop(self):
        if self.latest_scan is None:
            # No scan yet — pass through human command but at reduced speed
            self.cmd_pub.publish(self.current_cmd)
            return

        scan = self.latest_scan
        critical_obs = []   # (angle_rad_ROS, dist_m) with dist < CRIT_DIST_M
        warning_obs  = []   # dist < WARN_DIST_M

        for i, r in enumerate(scan.ranges):
            # Filter invalid readings
            if not math.isfinite(r):
                continue
            if not (scan.range_min <= r <= scan.range_max):
                continue

            if r < CRIT_DIST_M:
                angle = scan.angle_min + i * scan.angle_increment
                critical_obs.append((angle, r))
            elif r < WARN_DIST_M:
                angle = scan.angle_min + i * scan.angle_increment
                warning_obs.append((angle, r))

        # ── Publish obstacle alerts ───────────────────────────────
        all_obs = critical_obs + warning_obs
        if all_obs:
            # Find the closest obstacle overall
            closest_angle, closest_dist = min(all_obs, key=lambda x: x[1])
            zone = "CRITICAL" if closest_dist < CRIT_DIST_M else "WARNING"
            self._publish_alert(closest_angle, closest_dist, zone, critical_obs)

        # ── Decide what velocity to publish ──────────────────────
        if critical_obs:
            # CRITICAL: compute avoidance and override human command
            avoid_x, avoid_y = self._compute_avoidance(critical_obs)
            cmd = Twist()
            cmd.linear.x  = avoid_x
            cmd.linear.y  = avoid_y
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.avoidance_active = True

            # log once per transition
            if not self.avoidance_active:
                self.get_logger().warn(
                    f"[CRITICAL] Obstacle at "
                    f"{self._to_cw_deg(closest_angle):.1f}°  "
                    f"{closest_dist*1000:.1f}mm — avoidance active")
        else:
            # CLEAR or WARNING: pass human command through
            self.cmd_pub.publish(self.current_cmd)
            if self.avoidance_active:
                self.get_logger().info("[CLEAR] Path clear — resuming human control")
                self.avoidance_active = False

    # ── Avoidance vector ─────────────────────────────────────────
    def _compute_avoidance(self, obs_list):
        """
        Compute avoidance velocity vector.
        Each obstacle contributes a push-away force weighted by proximity.
        Returns (vx, vy) normalized to AVOID_SPEED.
        """
        ax, ay = 0.0, 0.0
        for angle_rad, dist_m in obs_list:
            # obstacle direction in robot frame (ROS: x=fwd, y=left)
            obs_x = math.cos(angle_rad)
            obs_y = math.sin(angle_rad)
            # weight: stronger when closer
            weight = max(0.0, (CRIT_DIST_M - dist_m) / CRIT_DIST_M) + 0.1
            ax -= obs_x * weight
            ay -= obs_y * weight

        magnitude = math.sqrt(ax**2 + ay**2)
        if magnitude < 1e-6:
            return 0.0, 0.0

        # Normalize and scale
        scale = AVOID_SPEED
        vx = (ax / magnitude) * scale
        vy = (ay / magnitude) * scale
        return vx, vy

    # ── Obstacle alert publisher ──────────────────────────────────
    def _publish_alert(self, closest_angle_rad, closest_dist_m, zone, critical_obs):
        alert = ObstacleAlert()
        alert.header.stamp    = self.get_clock().now().to_msg()
        alert.header.frame_id = 'base_link'
        alert.obstacle_present = True
        alert.angle_deg  = self._to_cw_deg(closest_angle_rad)
        alert.distance_mm = closest_dist_m * 1000.0
        alert.zone       = zone

        if critical_obs:
            av_x, av_y = self._compute_avoidance(critical_obs)
            alert.avoid_linear_x = av_x
            alert.avoid_linear_y = av_y
            alert.avoid_angular_z = 0.0

        self.alert_pub.publish(alert)

        # ── also publish ObstacleInfo (compact, for dashboard) ────
        info = ObstacleInfo()
        info.header.stamp    = alert.header.stamp
        info.header.frame_id = 'base_link'
        info.distance  = alert.distance_mm
        info.angle     = alert.angle_deg
        info.warning   = (zone in ('WARNING', 'CRITICAL'))
        info.emergency = (zone == 'CRITICAL')
        self.info_pub.publish(info)

        if zone == "CRITICAL":
            self.get_logger().warn(
                f"[{zone}] Obstacle: "
                f"{alert.angle_deg:.1f}°  "
                f"{alert.distance_mm:.1f}mm  "
                f"→ avoidance ({alert.avoid_linear_x:+.3f}, "
                f"{alert.avoid_linear_y:+.3f}) m/s")
        else:
            self.get_logger().info(
                f"[{zone}] Obstacle: "
                f"{alert.angle_deg:.1f}°  "
                f"{alert.distance_mm:.1f}mm")

    # ── Angle conversion ─────────────────────────────────────────
    @staticmethod
    def _to_cw_deg(scan_angle_rad: float) -> float:
        """
        Convert ROS LaserScan angle (CCW positive, 0=front)
        to clockwise-from-front degrees (0=front, 90=right, 180=back, 270=left).
        """
        deg = math.degrees(scan_angle_rad)
        return (-deg) % 360.0


def main(args=None):
    rclpy.init(args=args)
    node = LidarGuard()
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
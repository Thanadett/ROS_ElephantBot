#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from agv_interfaces.msg import HandControl

# ══════════════════════════════════════════════════════════════════
#  Palette (BGR) — ตรงกับ hand_controller.py เดิม
# ══════════════════════════════════════════════════════════════════
C = {
    'fwd':      ( 90, 200,  90),
    'bwd':      ( 90, 110, 210),
    'left':     (190, 190,  60),
    'right':    ( 60, 190, 210),
    'cw':       (170,  90, 190),
    'ccw':      ( 90, 190, 190),
    'strL':     (160,  90, 160),
    'strR':     (100, 170, 100),
    'stop':     ( 70,  70, 200),
    'idle':     (100, 110, 115),
    'rhand':    ( 80, 200, 130),
    'lhand':    (190, 110, 190),
    'speed':    ( 90, 180,  90),
    'warn':     ( 60, 170, 210),
    'override': ( 60,  90, 190),
    'text':     (200, 205, 210),
    'dim':      (100, 108, 115),
    'panel':    ( 22,  26,  30),
}

GESTURE_COLOR = {
    'STOP':       C['stop'],  'Forward':    C['fwd'],
    'Backward':   C['bwd'],   'Turn L':     C['left'],
    'Turn R':     C['right'], 'Rotate CW':  C['cw'],
    'Rotate CCW': C['ccw'],   'Strafe L':   C['strL'],
    'Strafe R':   C['strR'],  'Idle':       C['idle'],
}

FONT = cv2.FONT_HERSHEY_SIMPLEX

# ══════════════════════════════════════════════════════════════════
#  Draw helpers  (เหมือน hand_controller.py เดิม)
# ══════════════════════════════════════════════════════════════════
def alpha_rect(frame, x1, y1, x2, y2, color, alpha=0.55):
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)
    if x2 <= x1 or y2 <= y1:
        return
    roi = frame[y1:y2, x1:x2]
    bg  = np.full_like(roi, color)
    cv2.addWeighted(bg, alpha, roi, 1 - alpha, 0, roi)
    frame[y1:y2, x1:x2] = roi


def draw_speed_bar(frame, speed_f, w, h):
    bx, by = 10, 12
    bw, bh = 10, int(h * 0.30)

    alpha_rect(frame, bx - 3, by - 16, bx + bw + 3, by + bh + 16, C['panel'], 0.65)
    cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (45, 50, 55), -1)

    fill = int(bh * speed_f)
    if fill > 0:
        g = int(180 * speed_f)
        r = int(180 * (1 - speed_f))
        cv2.rectangle(frame, (bx, by + bh - fill), (bx + bw, by + bh), (r, g, 60), -1)

    cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (60, 65, 70), 1)
    cv2.putText(frame, "SPD", (bx - 1, by - 4),        FONT, 0.32, C['dim'],  1)
    cv2.putText(frame, f"{int(speed_f * 100)}%",
                (bx - 2, by + bh + 12),                FONT, 0.32, C['text'], 1)


def draw_status_panel(frame, state, w, h):
    pw, ph = 240, 148
    sx, sy = w - pw - 8, 8

    alpha_rect(frame, sx, sy, sx + pw, sy + ph, C['panel'], 0.72)
    cv2.rectangle(frame, (sx, sy), (sx + pw, sy + ph), (50, 55, 60), 1)

    gesture = state['gesture']
    gcol    = GESTURE_COLOR.get(gesture, C['idle'])

    cv2.putText(frame, gesture, (sx + 8, sy + 20), FONT, 0.52, gcol, 2)
    cv2.line(frame, (sx + 6, sy + 26), (sx + pw - 6, sy + 26), (45, 50, 55), 1)

    rows = [
        (f"X {state['lin_x']:+.2f}  Y {state['lin_y']:+.2f}  m/s", (150, 200, 150), 0.38),
        (f"Ang {state['ang_z']:+.2f} rad/s",                        (190, 180, 140), 0.38),
        (f"Spd {int(state['speed_f'] * 100):3d}%",                  (150, 185, 210), 0.38),
    ]
    y = sy + 42
    for txt, col, sc in rows:
        cv2.putText(frame, txt, (sx + 8, y), FONT, sc, col, 1)
        y += 18

    cv2.line(frame, (sx + 6, y + 1), (sx + pw - 6, y + 1), (45, 50, 55), 1)
    y += 8

    cv2.putText(frame, f"R: {state['r_mode']}", (sx + 8, y),      FONT, 0.36, C['rhand'], 1)
    cv2.putText(frame, f"L: {state['l_mode']}", (sx + 8, y + 16), FONT, 0.36, C['lhand'], 1)
    y += 32

    # steering state badge
    steer = state.get('steering_state', '')
    gear  = state.get('gear', 'P')
    badge_col = C['fwd'] if gear == 'D' else C['bwd'] if gear == 'R' else C['dim']
    cv2.putText(frame, f"Gear:{gear}  {steer}", (sx + 8, y), FONT, 0.34, badge_col, 1)


def draw_direction_arrow(frame, gesture, w, h):
    cx = w // 2
    cy = int(h * 0.88)
    s  = 28
    arrows = {
        'Forward':    ((cx, cy + s), (cx, cy - s),  C['fwd']),
        'Backward':   ((cx, cy - s), (cx, cy + s),  C['bwd']),
        'Turn L':     ((cx + s, cy), (cx - s, cy),  C['left']),
        'Turn R':     ((cx - s, cy), (cx + s, cy),  C['right']),
        'Rotate CW':  ((cx - s, cy), (cx + s, cy),  C['cw']),
        'Rotate CCW': ((cx + s, cy), (cx - s, cy),  C['ccw']),
        'Strafe L':   ((cx + s, cy), (cx - s, cy),  C['strL']),
        'Strafe R':   ((cx - s, cy), (cx + s, cy),  C['strR']),
    }
    if gesture in arrows:
        p1, p2, col = arrows[gesture]
        alpha_rect(frame, cx - 70, cy - s - 18, cx + 70, cy + s + 8, C['panel'], 0.5)
        cv2.arrowedLine(frame, p1, p2, col, 3, tipLength=0.35, line_type=cv2.LINE_AA)
        sz = cv2.getTextSize(gesture, FONT, 0.52, 1)[0]
        cv2.putText(frame, gesture, (cx - sz[0] // 2, cy - s - 4), FONT, 0.52, col, 1)
    elif gesture == 'STOP':
        alpha_rect(frame, cx - 70, cy - 20, cx + 70, cy + 20, C['panel'], 0.65)
        sz = cv2.getTextSize("STOP", FONT, 1.0, 2)[0]
        cv2.putText(frame, "STOP", (cx - sz[0] // 2, cy + 10), FONT, 1.0, C['stop'], 2)


def draw_info_bar(frame, state, w, h):
    bh = 26
    by = h - bh
    alpha_rect(frame, 0, by, w, h, C['panel'], 0.78)
    cv2.line(frame, (0, by), (w, by), (50, 55, 60), 1)

    gesture   = state['gesture']
    gcol      = GESTURE_COLOR.get(gesture, C['idle'])
    speed_pct = int(state['speed_f'] * 100)

    seg_x = [6]

    def seg(txt, col):
        cv2.putText(frame, txt, (seg_x[0], by + 17), FONT, 0.40, col, 1, cv2.LINE_AA)
        seg_x[0] += cv2.getTextSize(txt, FONT, 0.40, 1)[0][0] + 10

    def divider():
        cv2.line(frame, (seg_x[0], by + 5), (seg_x[0], by + bh - 5), (50, 55, 60), 1)
        seg_x[0] += 10

    seg(f"[ {gesture} ]",           gcol)
    divider()
    seg(f"X {state['lin_x']:+.2f}", (150, 200, 150))
    seg(f"Y {state['lin_y']:+.2f}", (150, 200, 150))
    seg(f"Z {state['ang_z']:+.2f}", (180, 175, 140))
    divider()
    seg(f"SPD {speed_pct}%",        C['speed'])
    divider()
    seg(f"R: {state['r_mode']}", C['rhand'])
    seg(f"L: {state['l_mode']}", C['lhand'])
    divider()
    seg(f"Gear:{state.get('gear','?')}  {state.get('steering_state','')}", C['text'])


# ══════════════════════════════════════════════════════════════════
#  ROS 2 Node: DashboardUI
# ══════════════════════════════════════════════════════════════════
class DashboardUI(Node):

    def __init__(self):
        super().__init__('dashboard_ui')

        # ── State (เริ่มต้น) ──────────────────────────────────────
        self.state = {
            'gesture':        'Idle',
            'lin_x':          0.0,
            'lin_y':          0.0,
            'ang_z':          0.0,
            'speed_f':        0.5,
            'r_mode':         '—',
            'l_mode':         '—',
            'gear':           'P',
            'steering_state': 'STRAIGHT',
            'steering_angle': 0.0,
        }

        # ── blank frame ───────────────────────────────────────────
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # ── Subscribers ───────────────────────────────────────────
        self.create_subscription(
            CompressedImage,
            '/camera_image/compressed',
            self._cb_image,
            10,
        )
        self.create_subscription(
            HandControl,
            '/hand_control_state',
            self._cb_state,
            10,
        )
        self.create_subscription(
            Twist,
            '/hand/cmd_vel',
            self._cb_twist,
            10,
        )

        # ── Render timer (30 fps) ─────────────────────────────────
        self.create_timer(1.0 / 30.0, self._render)

        self.get_logger().info("══ Dashboard UI ══")
        self.get_logger().info("   /camera_image/compressed  ← CompressedImage")
        self.get_logger().info("   /hand_control_state       ← HandControl")
        self.get_logger().info("   /hand/cmd_vel             ← Twist")
        self.get_logger().info("   Press q in window to quit")

    # ── Callbacks ─────────────────────────────────────────────────
    def _cb_image(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img is not None:
            self.frame = img

    def _cb_state(self, msg: HandControl):
        self.state['speed_f']        = float(msg.speed_percent) / 100.0
        self.state['gear']           = msg.gear
        self.state['steering_state'] = msg.steering_state
        self.state['steering_angle'] = float(msg.steering_angle)

        # optional fields (ถ้า HandControl มี)
        if hasattr(msg, 'gesture'):    self.state['gesture']  = msg.gesture
        if hasattr(msg, 'lin_x'):      self.state['lin_x']    = float(msg.lin_x)
        if hasattr(msg, 'lin_y'):      self.state['lin_y']    = float(msg.lin_y)
        if hasattr(msg, 'ang_z'):      self.state['ang_z']    = float(msg.ang_z)
        if hasattr(msg, 'right_mode'): self.state['r_mode']   = msg.right_mode
        if hasattr(msg, 'left_mode'):  self.state['l_mode']   = msg.left_mode

    def _cb_twist(self, msg: Twist):
        # fallback: อัปเดต velocity จาก Twist ถ้า HandControl ไม่มี field เหล่านี้
        self.state['lin_x'] = float(msg.linear.x)
        self.state['lin_y'] = float(msg.linear.y)
        self.state['ang_z'] = float(msg.angular.z)

        # อนุมาน gesture คร่าว ๆ จาก twist (ใช้เมื่อ HandControl ไม่ส่ง gesture)
        lx = msg.linear.x
        ly = msg.linear.y
        az = msg.angular.z
        if abs(lx) < 0.01 and abs(ly) < 0.01 and abs(az) < 0.01:
            pass  # ไม่เขียนทับ gesture ที่มาจาก HandControl
        else:
            # อนุมานแบบง่าย
            if lx > 0.01:   g = 'Forward'
            elif lx < -0.01: g = 'Backward'
            elif az > 0.05:  g = 'Turn L'
            elif az < -0.05: g = 'Turn R'
            elif ly > 0.01:  g = 'Strafe L'
            elif ly < -0.01: g = 'Strafe R'
            else:            g = 'Idle'
            # เขียนทับเฉพาะถ้า HandControl ไม่มี gesture field
            # (ป้องกัน race: ใช้ค่าเก่าถ้า HandControl ส่งมาแล้ว)
            if self.state['gesture'] == 'Idle':
                self.state['gesture'] = g

    # ── Render ────────────────────────────────────────────────────
    def _render(self):
        frame = self.frame.copy()
        h, w  = frame.shape[:2]

        draw_speed_bar(frame, self.state['speed_f'], w, h)
        draw_status_panel(frame, self.state, w, h)
        draw_direction_arrow(frame, self.state['gesture'], w, h)
        draw_info_bar(frame, self.state, w, h)

        cv2.imshow("myAGV | Dashboard UI [660610822]", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("q pressed → exiting")
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


# ══════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = DashboardUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt → stopping")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
hand_controller_headless.py — 660610822
Hand controller WITHOUT any cv2.imshow / UI drawing.

เพิ่ม publish:
  /camera_image/compressed   (sensor_msgs/CompressedImage)
  /hand_control_state        (agv_interfaces/HandControl)

ลบออก:
  draw_speed_bar / draw_status_box / draw_direction_arrow / cv2.imshow
"""

import os, sys, math, time, urllib.request
import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision
from mediapipe.tasks.python.vision import HandLandmarksConnections

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from agv_interfaces.msg import HandControl

# ══════════════════════════════════════════════════════════════════
#  Model auto-download  (ไม่เปลี่ยน)
# ══════════════════════════════════════════════════════════════════
MODEL_PATH = "hand_landmarker.task"
MODEL_URL  = ("https://storage.googleapis.com/mediapipe-models/"
              "hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task")

def ensure_model():
    if os.path.exists(MODEL_PATH):
        print(f"[INFO] Found model: {MODEL_PATH}"); return
    print("[INFO] Downloading model (~25 MB)...")
    def _prog(b, bs, ts):
        if ts > 0:
            p = min(b*bs/ts*100, 100)
            sys.stdout.write(f"\r  [{'█'*int(p/5)}{'░'*(20-int(p/5))}] {p:.1f}%")
            sys.stdout.flush()
    try:
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH, _prog)
        print("\n[INFO] Download complete")
    except Exception as e:
        print(f"\n[ERROR] {e}\n  Manual: wget -O {MODEL_PATH} '{MODEL_URL}'")
        sys.exit(1)

# ══════════════════════════════════════════════════════════════════
#  Constants
# ══════════════════════════════════════════════════════════════════
MAX_LIN           = 0.50
MIN_LIN           = 0.10
MAX_ANG           = 1.00
MIN_ANG           = 0.20
PUBLISH_HZ        = 20
SPEED_CENTER_HOLD = 1.0

TIP = [4, 8, 12, 16, 20]
PIP = [3, 6, 10, 14, 18]

HAND_CONNECTIONS = HandLandmarksConnections.HAND_CONNECTIONS

C = {
    'rhand': (0, 230, 110),
    'lhand': (220, 80, 220),
    'warn':  (30, 200, 220),
    'speed': (80, 200, 80),
}

# ══════════════════════════════════════════════════════════════════
#  Landmark helpers  (ไม่เปลี่ยน)
# ══════════════════════════════════════════════════════════════════
def px(lm, w, h):
    return int(lm.x * w), int(lm.y * h)

def is_finger_down(lms, finger_idx):
    return lms[TIP[finger_idx]].y > lms[PIP[finger_idx]].y + 0.02

def is_fist(lms):
    return sum(1 for i in range(1,5)
               if lms[TIP[i]].y > lms[PIP[i]].y + 0.05) >= 4

def is_open_palm(lms):
    return sum(1 for i in range(1,5)
               if lms[TIP[i]].y < lms[PIP[i]].y - 0.02) >= 4

# ══════════════════════════════════════════════════════════════════
#  Minimal skeleton draw (ไม่มี UI box แล้ว)
# ══════════════════════════════════════════════════════════════════
def draw_skeleton(frame, lms, color, w, h):
    for c in HAND_CONNECTIONS:
        x1,y1 = px(lms[c.start], w, h)
        x2,y2 = px(lms[c.end],   w, h)
        cv2.line(frame, (x1,y1), (x2,y2), color, 2)
    for lm in lms:
        cx, cy = px(lm, w, h)
        cv2.circle(frame, (cx,cy), 4, (255,255,255), -1)
        cv2.circle(frame, (cx,cy), 4, color, 1)

# ══════════════════════════════════════════════════════════════════
#  Speed Center Tracker  (ไม่เปลี่ยน)
# ══════════════════════════════════════════════════════════════════
class SpeedCenterTracker:
    def __init__(self):
        self.state      = 'idle'
        self.hold_start = None
        self.held_secs  = 0.0
        self.center_y   = None
        self.center_px  = None
        self.speed_f    = 0.5

    def update(self, mid_down, ring_down, mid_y, mid_px, now):
        both = mid_down and ring_down
        if self.state == 'idle':
            if both:
                self.state = 'holding'; self.hold_start = now
                self.held_secs = 0.0
                self.center_y  = mid_y; self.center_px = mid_px
        elif self.state == 'holding':
            if not both:
                self.state = 'idle'; self.hold_start = None
            else:
                self.held_secs = now - self.hold_start
                if self.held_secs >= SPEED_CENTER_HOLD:
                    self.state     = 'active'
                    self.center_y  = mid_y; self.center_px = mid_px
        elif self.state == 'active':
            if not both:
                self.state = 'idle'; self.hold_start = None
            else:
                dy = self.center_y - mid_y
                self.speed_f = float(np.clip(self.speed_f + dy*0.015*1.5, 0.15, 1.0))
        return self.state, self.speed_f, self.held_secs

    def reset(self):
        self.state = 'idle'; self.hold_start = None; self.held_secs = 0.0

# ══════════════════════════════════════════════════════════════════
#  ROS 2 Node: HandController (headless)
# ══════════════════════════════════════════════════════════════════
class HandController(Node):

    def __init__(self):
        super().__init__('hand_controller')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('flip',      True)
        self.declare_parameter('max_lin',   MAX_LIN)
        self.declare_parameter('min_lin',   MIN_LIN)
        self.declare_parameter('max_ang',   MAX_ANG)
        self.declare_parameter('min_ang',   MIN_ANG)

        cam_id       = self.get_parameter('camera_id').value
        self.flip    = self.get_parameter('flip').value
        self.max_lin = self.get_parameter('max_lin').value
        self.min_lin = self.get_parameter('min_lin').value
        self.max_ang = self.get_parameter('max_ang').value
        self.min_ang = self.get_parameter('min_ang').value

        # ── Publishers ────────────────────────────────────────────
        self.pub_twist = self.create_publisher(Twist, '/hand/cmd_vel', 10)
        self.pub_img   = self.create_publisher(CompressedImage,
                                               '/camera_image/compressed', 10)
        self.pub_state = self.create_publisher(HandControl,
                                               '/hand_control_state', 10)

        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.loop)

        ensure_model()
        opts = mp_vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=mp_vision.RunningMode.IMAGE,
            num_hands=2,
            min_hand_detection_confidence=0.65,
            min_hand_presence_confidence=0.65,
            min_tracking_confidence=0.65,
        )
        self.detector = mp_vision.HandLandmarker.create_from_options(opts)

        self.cap = cv2.VideoCapture(cam_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera id={cam_id}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.sc_tracker = SpeedCenterTracker()
        self.speed_f    = 0.5

        self.get_logger().info("══ Hand Controller (headless) ══")
        self.get_logger().info("   /hand/cmd_vel        ← Twist")
        self.get_logger().info("   /camera_image/compressed ← CompressedImage")
        self.get_logger().info("   /hand_control_state  ← HandControl")

    def _lin(self, v): return v*(self.min_lin + self.speed_f*(self.max_lin-self.min_lin))
    def _ang(self, v): return v*(self.min_ang + self.speed_f*(self.max_ang-self.min_ang))

    def loop(self):
        ret, frame = self.cap.read()
        if not ret: return
        if self.flip: frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]
        now  = time.time()

        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB,
                          data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        result = self.detector.detect(mp_img)

        right_lms = None
        left_lms  = None
        for i, hl in enumerate(result.handedness):
            raw   = hl[0].category_name
            label = "Left" if raw == "Right" else "Right"
            lms   = result.hand_landmarks[i]
            color = C['rhand'] if label == "Right" else C['lhand']
            draw_skeleton(frame, lms, color, w, h)          # draw on frame only
            if label == "Right": right_lms = lms
            else:                left_lms  = lms

        r_open = right_lms is not None and is_open_palm(right_lms)
        l_open = left_lms  is not None and is_open_palm(left_lms)
        r_fist = right_lms is not None and is_fist(right_lms)
        l_fist = left_lms  is not None and is_fist(left_lms)
        both_open = r_open and l_open
        no_hand   = right_lms is None and left_lms is None

        l_speed_gesture = False
        if left_lms is not None:
            _lm   = left_lms
            _mid  = is_finger_down(_lm, 2)
            _ring = is_finger_down(_lm, 3)
            _idx  = is_finger_down(_lm, 1)
            _pnk  = is_finger_down(_lm, 4)
            _fst  = is_fist(_lm)
            pure_speed_gesture = _mid and _ring and not _idx and not _pnk and not _fst
            l_speed_gesture = pure_speed_gesture or self.sc_tracker.state != 'idle'

        right_active = (right_lms is not None and not r_open and
                        (left_lms is None or l_open or l_speed_gesture))
        left_active  = (left_lms  is not None and not l_open and
                        (right_lms is None or r_open))
        left_speed_only = (left_lms is not None and not l_open
                           and l_speed_gesture and not left_active)

        lin_x, lin_y, ang_z = 0., 0., 0.
        gesture = 'Idle'; r_mode = '—'; l_mode = '—'

        # ── Right Hand ───────────────────────────────────────────
        if both_open or no_hand:
            gesture = 'STOP' if both_open else 'Idle'
        elif right_lms is not None:
            lms = right_lms
            index_dn = is_finger_down(lms, 1)
            mid_dn   = is_finger_down(lms, 2)
            ring_dn  = is_finger_down(lms, 3)
            pinky_dn = is_finger_down(lms, 4)
            fist     = r_fist

            fwd_bwd_conflict = mid_dn and pinky_dn
            lr_conflict      = index_dn and ring_dn
            cmds = []

            if not fwd_bwd_conflict:
                if mid_dn:   cmds.append(('lin_x', +1.0, 'Forward'))
                if pinky_dn: cmds.append(('lin_x', -1.0, 'Backward'))
            if not lr_conflict and not fist:
                if index_dn: cmds.append(('ang_z', +1.0, 'Turn L'))
                if ring_dn:  cmds.append(('ang_z', -1.0, 'Turn R'))
            if fist:
                cmds = [('ang_z', -1.5, 'Rotate CW')]

            for axis, val, _ in cmds:
                if axis == 'lin_x': lin_x += self._lin(val)
                else:               ang_z += self._ang(val)

            lin_x = float(np.clip(lin_x, -self.max_lin, self.max_lin))
            ang_z = float(np.clip(ang_z, -self.max_ang, self.max_ang))

            names = []
            if not fwd_bwd_conflict:
                if mid_dn:   names.append('Forward')
                if pinky_dn: names.append('Backward')
            if fist:         names.append('Rotate CW')
            elif not lr_conflict:
                if index_dn: names.append('Turn L')
                if ring_dn:  names.append('Turn R')

            if len(names) == 1: gesture = names[0]
            elif 'Forward'  in names and 'Turn L' in names: gesture = 'Fwd+TurnL'
            elif 'Forward'  in names and 'Turn R' in names: gesture = 'Fwd+TurnR'
            elif 'Backward' in names and 'Turn L' in names: gesture = 'Bwd+TurnL'
            elif 'Backward' in names and 'Turn R' in names: gesture = 'Bwd+TurnR'
            elif names: gesture = names[0]
            else:       gesture = 'Idle'

            fn = {1:'Idx',2:'Mid',3:'Rng',4:'Pnk'}
            active = [fn[i] for i,d in
                      [(1,index_dn),(2,mid_dn),(3,ring_dn),(4,pinky_dn)] if d]
            if fist: active = ['FIST']
            r_mode = ' '.join(active) if active else 'Ready'

        # ── Left Hand ────────────────────────────────────────────
        if (left_active or left_speed_only) and left_lms:
            lms = left_lms
            index_dn = is_finger_down(lms, 1)
            mid_dn   = is_finger_down(lms, 2)
            ring_dn  = is_finger_down(lms, 3)
            pinky_dn = is_finger_down(lms, 4)
            fist     = l_fist

            pure_mid_ring = (mid_dn and ring_dn
                             and not index_dn and not pinky_dn and not fist)
            if fist and self.sc_tracker.state != 'idle':
                self.sc_tracker.reset()

            mid_y  = lms[TIP[2]].y
            mid_px = px(lms[TIP[2]], w, h)
            sc_state, self.speed_f, sc_held = self.sc_tracker.update(
                pure_mid_ring, pure_mid_ring, mid_y, mid_px, now)

            sc_active = (sc_state == 'active')

            if left_active and not sc_active:
                if fist:
                    ang_z += self._ang(1.0)
                    gesture = 'Rotate CCW'
                else:
                    if index_dn:
                        tip_x   = lms[TIP[1]].x
                        wrist_x = lms[0].x
                        hand_w  = abs(lms[5].x - lms[17].x) + 0.05
                        sv = float(np.clip((tip_x - wrist_x)/hand_w, -1.5, 1.5))
                        lin_y += -self._lin(sv)
                    if pinky_dn:
                        tip_x   = lms[TIP[4]].x
                        wrist_x = lms[0].x
                        hand_w  = abs(lms[5].x - lms[17].x) + 0.05
                        sv = float(np.clip((tip_x - wrist_x)/hand_w, -1.5, 1.5))
                        lin_y += -self._lin(sv)
                    lin_y = float(np.clip(lin_y, -self.max_lin, self.max_lin))
                    if lin_y < -0.01 and gesture == 'Idle': gesture = 'Strafe R'
                    if lin_y >  0.01 and gesture == 'Idle': gesture = 'Strafe L'

            if sc_active:      l_mode = f'SpeedCtrl {int(self.speed_f*100)}%'
            elif sc_state == 'holding':
                                l_mode = f'Hold {self.sc_tracker.held_secs:.1f}s'
            elif fist:         l_mode = 'FIST (CCW)'
            else:
                parts = []
                if index_dn: parts.append('Idx')
                if pinky_dn: parts.append('Pnk')
                l_mode = ' '.join(parts) if parts else 'Ready'

        ang_z = float(np.clip(ang_z, -self.max_ang, self.max_ang))

        if both_open or no_hand:
            lin_x = lin_y = ang_z = 0.
            gesture = 'STOP' if both_open else 'Idle'

        # ── Publish Twist ─────────────────────────────────────────
        twist = Twist()
        twist.linear.x  = float(lin_x)
        twist.linear.y  = float(lin_y)
        twist.angular.z = float(ang_z)
        self.pub_twist.publish(twist)

        # ── Publish HandControl state ─────────────────────────────
        state_msg = HandControl()
        state_msg.speed_percent  = float(self.speed_f * 100)
        state_msg.gear           = ('D' if lin_x > 0.01 else
                                    'R' if lin_x < -0.01 else 'P')
        state_msg.steering_angle = float(math.degrees(ang_z) * 0.3)
        state_msg.steering_state = ('LEFT'  if ang_z >  0.05 else
                                    'RIGHT' if ang_z < -0.05 else 'STRAIGHT')
        if hasattr(state_msg, 'gesture'):      state_msg.gesture     = gesture
        if hasattr(state_msg, 'lin_x'):        state_msg.lin_x       = float(lin_x)
        if hasattr(state_msg, 'lin_y'):        state_msg.lin_y       = float(lin_y)
        if hasattr(state_msg, 'ang_z'):        state_msg.ang_z       = float(ang_z)
        if hasattr(state_msg, 'right_mode'):   state_msg.right_mode  = r_mode
        if hasattr(state_msg, 'left_mode'):    state_msg.left_mode   = l_mode
        self.pub_state.publish(state_msg)

        # ── Publish compressed image (skeleton drawn) ─────────────
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            img_msg = CompressedImage()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.format = 'jpeg'
            img_msg.data   = buf.tobytes()
            self.pub_img.publish(img_msg)

    def destroy_node(self):
        self.pub_twist.publish(Twist())
        self.cap.release()
        self.detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt → stopping")
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
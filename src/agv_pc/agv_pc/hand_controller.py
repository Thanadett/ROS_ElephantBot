#!/usr/bin/env python3
"""
hand_controller.py — 660610822
Single-file Hand Controller: UI + publish /camera_image/compressed + /hand_control_state
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
from agv_interfaces.msg import HandControl, ObstacleAlert

# ══════════════════════════════════════════════════════════════════
#  Model auto-download
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

# ── Muted, easy-on-eyes palette (BGR) ────────────────────────────
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
#  Landmark helpers
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
#  Draw helpers
# ══════════════════════════════════════════════════════════════════
def draw_skeleton(frame, lms, color, w, h):
    for c in HAND_CONNECTIONS:
        x1, y1 = px(lms[c.start], w, h)
        x2, y2 = px(lms[c.end],   w, h)
        cv2.line(frame, (x1,y1), (x2,y2), color, 2)
    for lm in lms:
        cx, cy = px(lm, w, h)
        cv2.circle(frame, (cx,cy), 4, (255,255,255), -1)
        cv2.circle(frame, (cx,cy), 4, color, 1)

def alpha_rect(frame, x1, y1, x2, y2, color, alpha=0.55):
    x1,y1 = max(0,x1), max(0,y1)
    x2,y2 = min(frame.shape[1],x2), min(frame.shape[0],y2)
    if x2<=x1 or y2<=y1: return
    roi = frame[y1:y2, x1:x2]
    bg  = np.full_like(roi, color)
    cv2.addWeighted(bg, alpha, roi, 1-alpha, 0, roi)
    frame[y1:y2, x1:x2] = roi

def draw_center_cross(frame, cx, cy, color, size=14):
    cv2.line(frame, (cx-size,cy), (cx+size,cy), color, 2)
    cv2.line(frame, (cx,cy-size), (cx,cy+size), color, 2)
    cv2.circle(frame, (cx,cy), 5, color, -1)

def draw_speed_bar(frame, speed_f, w, h):
    bx, by  = 10, 12
    bw, bh  = 10, int(h * 0.30)
    alpha_rect(frame, bx-3, by-16, bx+bw+3, by+bh+16, C['panel'], 0.65)
    cv2.rectangle(frame, (bx,by), (bx+bw,by+bh), (45,50,55), -1)
    fill = int(bh * speed_f)
    if fill > 0:
        g = int(180 * speed_f)
        r = int(180 * (1-speed_f))
        cv2.rectangle(frame, (bx, by+bh-fill), (bx+bw, by+bh), (r, g, 60), -1)
    cv2.rectangle(frame, (bx,by), (bx+bw,by+bh), (60,65,70), 1)
    cv2.putText(frame, "SPD",                  (bx-1, by-4),    FONT, 0.32, C['dim'],  1)
    cv2.putText(frame, f"{int(speed_f*100)}%", (bx-2, by+bh+12), FONT, 0.32, C['text'], 1)

def draw_lidar_panel(frame, lidar, w, h):
    """
    มุมขวาบน — แสดงสถานะ LiDAR obstacle จาก /obstacle_alert
    lidar = {
        'active':    bool,           # มี obstacle หรือไม่
        'zone':      str,            # 'FRONT' / 'REAR' / 'LEFT' / 'RIGHT' / 'NONE'
        'distance':  float,          # เมตร  (-1 = ไม่มีข้อมูล)
        'level':     str,            # 'CLEAR' / 'WARNING' / 'DANGER'
        'stamp':     float,          # time.time() ตอนรับ msg ล่าสุด
        'source':    str,            # topic ที่ใช้จริง
    }
    """
    pw, ph = 230, 130
    sx, sy = w - pw - 8, 8

    # ── สี border/header ตาม level ────────────────────────────────
    level  = lidar.get('level', 'CLEAR')
    active = lidar.get('active', False)
    age    = time.time() - lidar.get('stamp', 0)
    stale  = age > 2.0   # ไม่ได้รับ msg นานกว่า 2 วิ

    if stale:
        border_col = C['dim'];   head_col = C['dim']
    elif level == 'DANGER':
        border_col = C['stop'];  head_col = C['stop']
    elif level == 'WARNING':
        border_col = C['warn'];  head_col = C['warn']
    else:
        border_col = (50,55,60); head_col = C['dim']

    alpha_rect(frame, sx, sy, sx+pw, sy+ph, C['panel'], 0.72)
    cv2.rectangle(frame, (sx,sy), (sx+pw,sy+ph), border_col, 1)

    # ── Header: "LiDAR" + source topic ───────────────────────────
    src = lidar.get('source', '/obstacle_alert')
    cv2.putText(frame, "LiDAR",  (sx+8, sy+18), FONT, 0.52, head_col, 2)
    cv2.putText(frame, src,      (sx+70, sy+16), FONT, 0.30, C['dim'],  1)
    cv2.line(frame, (sx+6, sy+24), (sx+pw-6, sy+24), (45,50,55), 1)

    y = sy + 40

    # ── Status: CLEAR / WARNING / DANGER (หรือ NO DATA) ──────────
    if stale:
        cv2.putText(frame, "NO DATA", (sx+8, y), FONT, 0.50, C['dim'], 1)
        y += 22
    else:
        lvl_col = (C['stop'] if level=='DANGER' else
                   C['warn'] if level=='WARNING' else C['speed'])
        cv2.putText(frame, level, (sx+8, y), FONT, 0.55, lvl_col, 2)
        y += 22

    # ── Zone ─────────────────────────────────────────────────────
    zone = lidar.get('zone', 'NONE')
    zone_col = C['warn'] if zone != 'NONE' else C['dim']
    cv2.putText(frame, f"Zone : {zone}", (sx+8, y), FONT, 0.38, zone_col, 1)
    y += 18

    # ── Distance ─────────────────────────────────────────────────
    dist = lidar.get('distance', -1.0)
    if dist >= 0:
        dist_col = (C['stop'] if dist < 0.40 else
                    C['warn'] if dist < 0.80 else C['text'])
        cv2.putText(frame, f"Dist : {dist:.2f} m", (sx+8, y), FONT, 0.38, dist_col, 1)
    else:
        cv2.putText(frame, "Dist : ---", (sx+8, y), FONT, 0.38, C['dim'], 1)
    y += 18

    # ── Age indicator (dot blink) ─────────────────────────────────
    dot_col = C['speed'] if not stale else C['dim']
    cv2.circle(frame, (sx+pw-16, sy+12), 5, dot_col, -1)

    # ── Distance bar (mini, horizontal) ──────────────────────────
    if dist >= 0 and not stale:
        bar_x, bar_y = sx+8, y+4
        bar_w = pw - 20
        max_d = 2.0
        fill  = int(bar_w * min(dist / max_d, 1.0))
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x+bar_w, bar_y+6), (40,44,48), -1)
        fill_col = (C['stop'] if dist < 0.40 else
                    C['warn'] if dist < 0.80 else C['speed'])
        if fill > 0:
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x+fill, bar_y+6), fill_col, -1)
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x+bar_w, bar_y+6), (60,65,70), 1)
        cv2.putText(frame, "0",      (bar_x,        bar_y+18), FONT, 0.28, C['dim'], 1)
        cv2.putText(frame, f"{max_d:.0f}m", (bar_x+bar_w-14, bar_y+18), FONT, 0.28, C['dim'], 1)

def draw_direction_arrow(frame, gesture, w, h):
    cx = w//2;  cy = int(h*0.88);  s = 28
    arrows = {
        'Forward':    ((cx,cy+s),(cx,cy-s),  C['fwd']),
        'Backward':   ((cx,cy-s),(cx,cy+s),  C['bwd']),
        'Turn L':     ((cx+s,cy),(cx-s,cy),  C['left']),
        'Turn R':     ((cx-s,cy),(cx+s,cy),  C['right']),
        'Rotate CW':  ((cx-s,cy),(cx+s,cy),  C['cw']),
        'Rotate CCW': ((cx+s,cy),(cx-s,cy),  C['ccw']),
        'Strafe L':   ((cx+s,cy),(cx-s,cy),  C['strL']),
        'Strafe R':   ((cx-s,cy),(cx+s,cy),  C['strR']),
    }
    if gesture in arrows:
        p1, p2, col = arrows[gesture]
        alpha_rect(frame, cx-70, cy-s-18, cx+70, cy+s+8, C['panel'], 0.5)
        cv2.arrowedLine(frame, p1, p2, col, 3, tipLength=0.35, line_type=cv2.LINE_AA)
        sz = cv2.getTextSize(gesture, FONT, 0.52, 1)[0]
        cv2.putText(frame, gesture, (cx-sz[0]//2, cy-s-4), FONT, 0.52, col, 1)
    elif gesture == 'STOP':
        alpha_rect(frame, cx-70, cy-20, cx+70, cy+20, C['panel'], 0.65)
        sz = cv2.getTextSize("STOP", FONT, 1.0, 2)[0]
        cv2.putText(frame, "STOP", (cx-sz[0]//2, cy+10), FONT, 1.0, C['stop'], 2)

def draw_info_bar(frame, state, w, h):
    bh = 26
    by = h - bh
    alpha_rect(frame, 0, by, w, h, C['panel'], 0.78)
    cv2.line(frame, (0,by), (w,by), (50,55,60), 1)

    gesture   = state['gesture']
    gcol      = GESTURE_COLOR.get(gesture, C['idle'])
    speed_pct = int(state['speed_f']*100)
    sc_state  = state.get('sc_state','idle')

    seg_x = 6
    def seg(txt, col):
        nonlocal seg_x
        cv2.putText(frame, txt, (seg_x, by+17), FONT, 0.40, col, 1, cv2.LINE_AA)
        seg_x += cv2.getTextSize(txt, FONT, 0.40, 1)[0][0] + 10

    def divider():
        nonlocal seg_x
        cv2.line(frame, (seg_x, by+5), (seg_x, by+bh-5), (50,55,60), 1)
        seg_x += 10

    seg(f"[ {gesture} ]",           gcol)
    divider()
    seg(f"X {state['lin_x']:+.2f}", (150,200,150))
    seg(f"Y {state['lin_y']:+.2f}", (150,200,150))
    seg(f"Z {state['ang_z']:+.2f}", (180,175,140))
    divider()
    seg(f"SPD {speed_pct}%",        C['speed'] if sc_state=='active' else C['dim'])
    if sc_state == 'holding':
        seg("hold…", C['warn'])
    elif sc_state == 'active':
        seg("SpeedCtrl", C['speed'])
    divider()
    seg(f"R: {state['r_mode']}", C['rhand'])
    seg(f"L: {state['l_mode']}", C['lhand'])

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
                    self.state    = 'active'
                    self.center_y = mid_y; self.center_px = mid_px
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
#  ROS 2 Node
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
        self.pub       = self.create_publisher(Twist,            '/hand/cmd_vel',             10)
        self.pub_img   = self.create_publisher(CompressedImage,  '/camera_image/compressed',  10)
        self.pub_state = self.create_publisher(HandControl,      '/hand_control_state',        10)

        self.create_subscription(Twist, '/system/status_twist', lambda msg: None, 10)
        self.create_subscription(ObstacleAlert, '/obstacle_alert',
                                 self._cb_obstacle, 10)
        self.timer = self.create_timer(1.0/PUBLISH_HZ, self.loop)

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
        self.lidar = {
            'active':   False,
            'zone':     'NONE',
            'distance': -1.0,
            'level':    'CLEAR',
            'stamp':    0.0,
            'source':   '/obstacle_alert',
        }
        self.state = {
            'gesture':'Idle','lin_x':0.,'lin_y':0.,'ang_z':0.,
            'speed_f':0.5,'r_mode':'—','l_mode':'—',
            'sc_state':'idle','sc_held':0.,
        }
        self.get_logger().info("══ Hand Controller (660610822) ══")
        self.get_logger().info("   /hand/cmd_vel              ← Twist")
        self.get_logger().info("   /camera_image/compressed   ← CompressedImage")
        self.get_logger().info("   /hand_control_state        ← HandControl")
        self.get_logger().info(f"  mediapipe {mp.__version__}")
        self.get_logger().info("   Press q to quit")

    def _cb_obstacle(self, msg: ObstacleAlert):
        """
        รับ ObstacleAlert — รองรับ field หลายแบบ (ปรับตาม .msg จริง)
        field ที่พยายาม read:
          zone      (str)   หรือ direction
          distance  (float) หรือ distance_m
          active    (bool)
          level     (str)   หรือ อนุมานจาก distance
        """
        d = {}
        # zone / direction
        if   hasattr(msg, 'zone'):        d['zone']     = str(msg.zone).upper()
        elif hasattr(msg, 'direction'):   d['zone']     = str(msg.direction).upper()
        else:                             d['zone']     = 'UNKNOWN'
        # distance
        if   hasattr(msg, 'distance'):    d['distance'] = float(msg.distance)
        elif hasattr(msg, 'distance_m'):  d['distance'] = float(msg.distance_m)
        else:                             d['distance'] = -1.0
        # active
        if   hasattr(msg, 'active'):      d['active']   = bool(msg.active)
        else:                             d['active']   = d['distance'] >= 0 and d['zone'] != 'NONE'
        # level
        if   hasattr(msg, 'level'):       d['level']    = str(msg.level).upper()
        else:
            dist = d['distance']
            d['level'] = ('DANGER'  if 0 <= dist < 0.40 else
                          'WARNING' if 0 <= dist < 0.80 else 'CLEAR')
        d['stamp']  = time.time()
        d['source'] = '/obstacle_alert'
        self.lidar.update(d)

    def _lin(self, v): return v*(self.min_lin+self.speed_f*(self.max_lin-self.min_lin))
    def _ang(self, v): return v*(self.min_ang+self.speed_f*(self.max_ang-self.min_ang))

    def loop(self):
        ret, frame = self.cap.read()
        if not ret: return
        if self.flip: frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]
        now  = time.time()

        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB,
                          data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        result = self.detector.detect(mp_img)

        right_lms = left_lms = None
        for i, hl in enumerate(result.handedness):
            raw   = hl[0].category_name
            label = "Left" if raw=="Right" else "Right"
            lms   = result.hand_landmarks[i]
            color = C['rhand'] if label=="Right" else C['lhand']
            draw_skeleton(frame, lms, color, w, h)
            tx, ty = px(lms[9], w, h)
            cv2.putText(frame, label, (tx-15,ty-18), FONT, 0.50, color, 1)
            if label=="Right": right_lms=lms
            else:              left_lms=lms

        r_open    = right_lms is not None and is_open_palm(right_lms)
        l_open    = left_lms  is not None and is_open_palm(left_lms)
        r_fist    = right_lms is not None and is_fist(right_lms)
        l_fist    = left_lms  is not None and is_fist(left_lms)
        both_open = r_open and l_open
        no_hand   = right_lms is None and left_lms is None

        l_speed_gesture = False
        if left_lms is not None:
            _lm  = left_lms
            _mid = is_finger_down(_lm,2); _ring = is_finger_down(_lm,3)
            _idx = is_finger_down(_lm,1); _pnk  = is_finger_down(_lm,4)
            _fst = is_fist(_lm)
            pure_speed_gesture = _mid and _ring and not _idx and not _pnk and not _fst
            l_speed_gesture = pure_speed_gesture or self.sc_tracker.state!='idle'

        right_active    = (right_lms is not None and not r_open and
                           (left_lms is None or l_open or l_speed_gesture or l_fist))
        left_active     = (left_lms  is not None and not l_open and
                           (right_lms is None or r_open or r_fist))
        left_speed_only = (left_lms  is not None and not l_open
                           and l_speed_gesture and not left_active)

        lin_x=lin_y=ang_z=0.; gesture='Idle'; r_mode='—'; l_mode='—'

        # ── Right Hand ───────────────────────────────────────────
        if both_open or no_hand:
            gesture = 'STOP' if both_open else 'Idle'
        elif right_lms is not None:
            lms=right_lms
            index_dn=is_finger_down(lms,1); mid_dn  =is_finger_down(lms,2)
            ring_dn =is_finger_down(lms,3); pinky_dn=is_finger_down(lms,4)
            fist=r_fist

            fwd_bwd_conflict=mid_dn and pinky_dn
            lr_conflict=index_dn and ring_dn
            cmds=[]

            if not fwd_bwd_conflict:
                if mid_dn:   cmds.append(('lin_x',+1.0,'Forward'))
                if pinky_dn: cmds.append(('lin_x',-1.0,'Backward'))
            if not lr_conflict and not fist:
                if index_dn: cmds.append(('ang_z',+1.0,'Turn L'))
                if ring_dn:  cmds.append(('ang_z',-1.0,'Turn R'))
            if fist: cmds=[('ang_z',-1.5,'Rotate CW')]

            for axis,val,_ in cmds:
                if axis=='lin_x': lin_x+=self._lin(val)
                else:             ang_z+=self._ang(val)

            lin_x=float(np.clip(lin_x,-self.max_lin,self.max_lin))
            ang_z=float(np.clip(ang_z,-self.max_ang,self.max_ang))

            names=[]
            if not fwd_bwd_conflict:
                if mid_dn:   names.append('Forward')
                if pinky_dn: names.append('Backward')
            if fist: names.append('Rotate CW')
            elif not lr_conflict:
                if index_dn: names.append('Turn L')
                if ring_dn:  names.append('Turn R')
            if fwd_bwd_conflict: names.append('⚠F/B')
            if lr_conflict:      names.append('⚠L/R')

            if len(names)==1: gesture=names[0]
            elif 'Forward'  in names and 'Turn L' in names: gesture='Fwd+TurnL'
            elif 'Forward'  in names and 'Turn R' in names: gesture='Fwd+TurnR'
            elif 'Backward' in names and 'Turn L' in names: gesture='Bwd+TurnL'
            elif 'Backward' in names and 'Turn R' in names: gesture='Bwd+TurnR'
            elif names: gesture=names[0]
            else:       gesture='Idle'

            fn={1:'Idx',2:'Mid',3:'Rng',4:'Pnk'}
            active=[fn[i] for i,d in
                    [(1,index_dn),(2,mid_dn),(3,ring_dn),(4,pinky_dn)] if d]
            if fist: active=['FIST']
            r_mode=' '.join(active) if active else 'Ready'

            for i,dn in [(1,index_dn),(2,mid_dn),(3,ring_dn),(4,pinky_dn)]:
                if dn:
                    tx,ty=px(lms[TIP[i]],w,h)
                    cv2.circle(frame,(tx,ty),10,C['rhand'],2)

        # ── Left Hand ────────────────────────────────────────────
        if (left_active or left_speed_only) and left_lms:
            lms=left_lms
            index_dn=is_finger_down(lms,1); mid_dn  =is_finger_down(lms,2)
            ring_dn =is_finger_down(lms,3); pinky_dn=is_finger_down(lms,4)
            fist=l_fist

            pure_mid_ring=(mid_dn and ring_dn
                           and not index_dn and not pinky_dn and not fist)
            if fist and self.sc_tracker.state!='idle':
                self.sc_tracker.reset()

            mid_y=lms[TIP[2]].y; mid_px=px(lms[TIP[2]],w,h)
            sc_state,self.speed_f,sc_held=self.sc_tracker.update(
                pure_mid_ring,pure_mid_ring,mid_y,mid_px,now)

            if sc_state=='holding':
                cv2.circle(frame,mid_px,16,C['warn'],2)
                remain=max(0,SPEED_CENTER_HOLD-sc_held)
                cv2.putText(frame,f"{remain:.1f}s",(mid_px[0]+12,mid_px[1]),
                            FONT,0.55,C['warn'],1)
            elif sc_state=='active' and self.sc_tracker.center_px:
                cpx,cpy=self.sc_tracker.center_px
                draw_center_cross(frame,cpx,cpy,C['speed'])
                cy_now=px(lms[TIP[2]],w,h)[1]
                cv2.line(frame,(cpx,cpy),(cpx,cy_now),C['speed'],1)

            sc_active=(sc_state=='active')

            if left_active and not sc_active:
                if fist:
                    ang_z+=self._ang(1.0); gesture='Rotate CCW'
                else:
                    if index_dn:
                        tip_x=lms[TIP[1]].x; wrist_x=lms[0].x
                        hand_w=abs(lms[5].x-lms[17].x)+0.05
                        sv=float(np.clip((tip_x-wrist_x)/hand_w,-1.5,1.5))
                        lin_y+=-self._lin(sv)
                        cv2.circle(frame,px(lms[TIP[1]],w,h),11,C['strR'],2)
                    if pinky_dn:
                        tip_x=lms[TIP[4]].x; wrist_x=lms[0].x
                        hand_w=abs(lms[5].x-lms[17].x)+0.05
                        sv=float(np.clip((tip_x-wrist_x)/hand_w,-1.5,1.5))
                        lin_y+=-self._lin(sv)
                        cv2.circle(frame,px(lms[TIP[4]],w,h),11,C['strL'],2)

                    lin_y=float(np.clip(lin_y,-self.max_lin,self.max_lin))
                    if lin_y<-0.01 and gesture=='Idle': gesture='Strafe R'
                    if lin_y> 0.01 and gesture=='Idle': gesture='Strafe L'

            if sc_active:             l_mode=f'SpeedCtrl {int(self.speed_f*100)}%'
            elif sc_state=='holding': l_mode=f'Hold {self.sc_tracker.held_secs:.1f}s'
            elif fist:                l_mode='FIST (CCW)'
            else:
                parts=[]
                if index_dn: parts.append('Idx')
                if pinky_dn: parts.append('Pnk')
                l_mode=' '.join(parts) if parts else 'Ready'

        ang_z=float(np.clip(ang_z,-self.max_ang,self.max_ang))

        if both_open or no_hand:
            lin_x=lin_y=ang_z=0.
            gesture='STOP' if both_open else 'Idle'

        # ── Publish Twist ─────────────────────────────────────────
        twist=Twist()
        twist.linear.x=float(lin_x); twist.linear.y=float(lin_y)
        twist.angular.z=float(ang_z)
        self.pub.publish(twist)

        # ── Publish HandControl state ─────────────────────────────
        state_msg = HandControl()
        state_msg.speed_percent  = float(self.speed_f * 100)
        state_msg.gear           = ('D' if lin_x >  0.01 else
                                    'R' if lin_x < -0.01 else 'P')
        state_msg.steering_angle = float(math.degrees(ang_z) * 0.3)
        state_msg.steering_state = ('LEFT'  if ang_z >  0.05 else
                                    'RIGHT' if ang_z < -0.05 else 'STRAIGHT')
        if hasattr(state_msg, 'gesture'):    state_msg.gesture    = gesture
        if hasattr(state_msg, 'lin_x'):      state_msg.lin_x      = float(lin_x)
        if hasattr(state_msg, 'lin_y'):      state_msg.lin_y      = float(lin_y)
        if hasattr(state_msg, 'ang_z'):      state_msg.ang_z      = float(ang_z)
        if hasattr(state_msg, 'right_mode'): state_msg.right_mode = r_mode
        if hasattr(state_msg, 'left_mode'):  state_msg.left_mode  = l_mode
        self.pub_state.publish(state_msg)

        self.state.update({
            'gesture':gesture,'lin_x':lin_x,'lin_y':lin_y,'ang_z':ang_z,
            'speed_f':self.speed_f,'r_mode':r_mode,'l_mode':l_mode,
            'sc_state':self.sc_tracker.state,'sc_held':self.sc_tracker.held_secs,
        })

        # ── Draw UI ───────────────────────────────────────────────
        draw_speed_bar(frame, self.speed_f, w, h)
        draw_lidar_panel(frame, self.lidar, w, h)
        draw_direction_arrow(frame, gesture, w, h)
        draw_info_bar(frame, self.state, w, h)

        if r_open or l_open:
            tags = []
            if r_open: tags.append("R: open")
            if l_open: tags.append("L: open")
            cv2.putText(frame, "  ".join(tags),
                        (w//2-60, 22), FONT, 0.44, C['warn'], 1)

        cv2.imshow("myAGV | Hand Controller [660610822]", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("q pressed → exiting")
            self.pub.publish(Twist())
            rclpy.shutdown()

        # ── Publish compressed image (after UI draw) ──────────────
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            img_msg = CompressedImage()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.format = 'jpeg'
            img_msg.data   = buf.tobytes()
            self.pub_img.publish(img_msg)

    def destroy_node(self):
        self.pub.publish(Twist())
        self.cap.release()
        cv2.destroyAllWindows()
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
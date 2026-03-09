#!/usr/bin/env python3
"""
test_udp.py  — 660610822 Test Utility
ทดสอบการส่ง UDP จาก udp_sender node

วิธีใช้:
  Terminal 1 (PC, DOMAIN_ID=1):
      ros2 run agv_pc udp_sender --ros-args -p robot_ip:=127.0.0.1 -p udp_port:=15000

  Terminal 2:
      python test/test_udp.py

  หรือทดสอบแบบไม่ต้องรัน node:
      python test/test_udp.py --standalone
      (จะส่ง UDP packet เองโดยตรงแล้วรับกลับ)
"""

import argparse
import socket
import threading
import time
import sys

UDP_PORT = 15000
TIMEOUT  = 3.0


# ══════════════════════════════════════════════════════════════════
#  Test A: รับ UDP ที่ถูกส่งจาก udp_sender node จริงๆ
# ══════════════════════════════════════════════════════════════════
def test_receive_from_node():
    """
    เปิด UDP socket รับฝั่งเดียว
    แล้วรอ packet จาก udp_sender node
    ใช้เพื่อยืนยันว่า node ส่งจริง
    """
    print("\n═══ Test A: Receive UDP from udp_sender node ═══")
    print(f"   Listening on UDP 0.0.0.0:{UDP_PORT}")
    print(f"   Timeout: {TIMEOUT}s")
    print(f"   ── ต้องรัน udp_sender node ใน terminal อื่นก่อน ──")
    print(f"   ── และ publish บน /cmd_vel_command ──\n")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', UDP_PORT))
    sock.settimeout(TIMEOUT)

    received = []
    start = time.time()

    print("  รอ packet... (Ctrl+C เพื่อหยุด)")
    try:
        while time.time() - start < 10.0:
            try:
                data, addr = sock.recvfrom(1024)
                decoded = data.decode().strip()
                parts = decoded.split()
                if len(parts) == 3:
                    lx, ly, az = map(float, parts)
                    received.append((time.time(), lx, ly, az, addr))
                    print(f"  ✅ [{len(received):3d}] from {addr[0]}:{addr[1]}"
                          f"  lin_x={lx:+.3f}  lin_y={ly:+.3f}  ang_z={az:+.3f}")
                else:
                    print(f"  ⚠️  Bad packet: {repr(decoded)}")
            except socket.timeout:
                if received:
                    break
                print(f"  ⏳ ยังไม่ได้รับ packet... "
                      f"(รัน udp_sender node และ publish /cmd_vel_command)")
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    if received:
        rates = []
        for i in range(1, len(received)):
            dt = received[i][0] - received[i-1][0]
            if dt > 0: rates.append(1.0/dt)
        avg_hz = sum(rates)/len(rates) if rates else 0
        print(f"\n  สรุป: รับได้ {len(received)} packets")
        print(f"  อัตราเฉลี่ย: {avg_hz:.1f} Hz")
        print(f"  ✅ UDP ส่งได้ปกติ")
    else:
        print(f"\n  ❌ ไม่ได้รับ packet ใดเลย")
        print(f"  → ตรวจสอบว่า udp_sender รันอยู่และมี /cmd_vel_command")


# ══════════════════════════════════════════════════════════════════
#  Test B: Standalone loopback (ไม่ต้องรัน node)
# ══════════════════════════════════════════════════════════════════
def test_standalone_loopback():
    """
    ส่ง UDP packet หาตัวเองแล้วรับกลับ
    ทดสอบว่า format ถูกต้องและ network ทำงาน
    ไม่ต้องรัน ROS node ใดๆ
    """
    print("\n═══ Test B: Standalone UDP loopback ═══")
    print(f"   127.0.0.1:{UDP_PORT} → self\n")

    received_packets = []
    server_ready = threading.Event()

    def server():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', UDP_PORT))
        s.settimeout(2.0)
        server_ready.set()
        try:
            while True:
                data, addr = s.recvfrom(1024)
                received_packets.append(data.decode().strip())
        except socket.timeout:
            pass
        finally:
            s.close()

    t = threading.Thread(target=server, daemon=True)
    t.start()
    server_ready.wait()

    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    test_cases = [
        (0.0,   0.0,  0.0,  "Zero / Stop"),
        (0.3,   0.0,  0.0,  "Forward"),
        (-0.3,  0.0,  0.0,  "Backward"),
        (0.0,   0.15, 0.0,  "Strafe Left"),
        (0.0,  -0.15, 0.0,  "Strafe Right"),
        (0.0,   0.0,  1.0,  "Rotate CCW"),
        (0.0,   0.0, -1.0,  "Rotate CW"),
        (0.3,   0.0,  0.5,  "Forward + Turn"),
        (0.5,   0.15, 0.0,  "Forward + Strafe"),
        (-0.5,  0.0, -1.0,  "Backward + CW"),
    ]

    print(f"  {'#':>3}  {'Label':<20}  {'Sent':^28}  {'Received':^28}  Result")
    print(f"  {'─'*3}  {'─'*20}  {'─'*28}  {'─'*28}  {'─'*6}")

    all_pass = True
    for i, (lx, ly, az, label) in enumerate(test_cases):
        payload = f"{lx} {ly} {az}"
        sender.sendto(payload.encode(), ('127.0.0.1', UDP_PORT))
        time.sleep(0.05)

        if received_packets:
            rx = received_packets.pop(0)
            try:
                rx_lx, rx_ly, rx_az = map(float, rx.split())
                ok = (abs(rx_lx-lx)<1e-6 and abs(rx_ly-ly)<1e-6 and abs(rx_az-az)<1e-6)
                status = "✅" if ok else "❌"
                if not ok: all_pass = False
            except Exception:
                status = "❌ parse error"
                all_pass = False
            print(f"  {i+1:>3}  {label:<20}  {payload:^28}  {rx:^28}  {status}")
        else:
            print(f"  {i+1:>3}  {label:<20}  {payload:^28}  {'(no response)':^28}  ❌")
            all_pass = False

    sender.close()
    t.join(timeout=2.5)

    print(f"\n  {'✅ ทุก packet ถูกต้อง' if all_pass else '❌ มี packet ผิดพลาด'}")


# ══════════════════════════════════════════════════════════════════
#  Test C: ทดสอบ format ที่ udp_cmd_relay (Robot) คาดหวัง
# ══════════════════════════════════════════════════════════════════
def test_packet_format():
    print("\n═══ Test C: Packet format validation ═══\n")

    test_payloads = [
        ("0.3 0.0 0.5",    True,  "valid — forward+turn"),
        ("0.0 0.0 0.0",    True,  "valid — stop"),
        ("-0.5 0.15 -1.0", True,  "valid — all negative/mixed"),
        ("0.3 0.0",         False, "invalid — missing field"),
        ("abc 0.0 0.5",     False, "invalid — non-numeric"),
        ("0.3 0.0 0.5 0.1", False, "invalid — extra field"),
        ("",                False, "invalid — empty"),
    ]

    all_pass = True
    for payload, expect_ok, desc in test_payloads:
        try:
            parts = payload.split()
            if len(parts) != 3:
                raise ValueError("wrong field count")
            lx, ly, az = map(float, parts)
            result = True
        except Exception:
            result = False

        ok = (result == expect_ok)
        if not ok: all_pass = False
        status = "✅" if ok else "❌"
        print(f"  {status}  {repr(payload):<30}  → {desc}")

    print(f"\n  {'✅ format validation ผ่านหมด' if all_pass else '❌ มี case ผิดพลาด'}")


# ══════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='UDP sender test')
    parser.add_argument('--standalone', action='store_true',
                        help='Run loopback + format test only (no ROS needed)')
    args = parser.parse_args()

    if args.standalone:
        test_standalone_loopback()
        test_packet_format()
    else:
        print("เลือกโหมดทดสอบ:")
        print("  A = รับ packet จาก udp_sender node จริงๆ")
        print("  B = loopback test (ไม่ต้องรัน node)")
        print("  C = format validation")
        print("  หรือรัน: python test_udp.py --standalone  (ทำ B+C อัตโนมัติ)")
        choice = input("\nเลือก (A/B/C): ").strip().upper()
        if choice == 'A':
            test_receive_from_node()
        elif choice == 'B':
            test_standalone_loopback()
        elif choice == 'C':
            test_packet_format()
        else:
            print("ไม่รู้จัก option")
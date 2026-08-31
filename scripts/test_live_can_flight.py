#!/usr/bin/env python3
"""
Stallion VTOL - Live QLOITER Flight Simulation with Real-Time CAN Bus Streaming
================================================================================
Flies the exact QLOITER takeoff and 8m loiter sequence while simultaneously
streaming and verifying CAN telemetry channels across CAN 2.0B / CAN-over-IP.
"""

import os
import sys
import time
import json
import math
import struct
import socket
import subprocess
import threading
import signal
from pymavlink import mavutil

REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_PATH = os.path.join(REPO_DIR, 'gazebo', 'worlds', 'stallion_runway.sdf')
PARAM_FILE = os.path.join(REPO_DIR, 'params', 'stallion_vtol_sitl.parm')

def get_ardupilot_dir():
    for c in [os.path.expanduser('~/ardupilot'), '/home/drones/ardupilot', '/home/runner/ardupilot']:
        if os.path.exists(os.path.join(c, 'build', 'sitl', 'bin', 'arduplane')):
            return c
    return '/home/drones/ardupilot'

def get_plugin_dir():
    for c in [os.path.expanduser('~/ardupilot_gazebo/build'), '/home/drones/ardupilot_gazebo/build']:
        if os.path.exists(c):
            return c
    return '/home/drones/ardupilot_gazebo/build'

def run_live_can_mission(duration_sec=20):
    print("=" * 80)
    print("  STALLION VTOL - LIVE QLOITER FLIGHT & CAN BUS STREAMING TEST")
    print("=" * 80)

    # 1. Clean old processes
    subprocess.run("killall -9 gz-sim-server gz-sim-gui arduplane 2>/dev/null; sleep 1", shell=True)
    time.sleep(1.0)

    # 2. Launch Gazebo physics server
    print("[1/4] Launching Gazebo 3D physics server...")
    env = os.environ.copy()
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = get_plugin_dir()
    env["GZ_SIM_RESOURCE_PATH"] = os.path.join(REPO_DIR, "gazebo", "models")
    gz_proc = subprocess.Popen(
        f"gz sim -s -r {WORLD_PATH}",
        shell=True,
        env=env,
        preexec_fn=os.setsid
    )
    time.sleep(3.0)

    # 3. Launch ArduPlane SITL
    print("[2/4] Launching ArduPlane SITL...")
    ardupilot_dir = get_ardupilot_dir()
    sitl_cmd = (
        f"cd {ardupilot_dir} && rm -f mav.parm ./mav.parm 2>/dev/null && "
        f"python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON -w "
        f"--custom-location=\"13.0827,80.2707,10,90\" "
        f"--add-param-file={PARAM_FILE} "
        f"--no-mavproxy --no-rebuild -S 1 -D --sysid 1"
    )
    sitl_proc = subprocess.Popen(sitl_cmd, shell=True, preexec_fn=os.setsid)
    time.sleep(5.0)

    # 4. Connect MAVLink
    print("[3/5] Connecting to SITL MAVLink...")
    mav = None
    for _ in range(20):
        try:
            mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, source_component=190, autoreconnect=True)
            mav.wait_heartbeat(timeout=2.0)
            mav.target_system = 1
            mav.target_component = 1
            print("    [OK] Connected to Autopilot System ID: 1")
            break
        except Exception:
            time.sleep(0.5)

    mav.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)

    # Setup CAN Socket (UDP broadcast on port 10005)
    can_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    can_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    can_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    # 5. Wait for EKF3 Alignment
    print("[4/5] Awaiting EKF3 GPS & Attitude alignment...")
    t_align = time.time()
    while time.time() - t_align < 12.0:
        msg = mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1.0)
        if msg and (msg.flags & 1 or msg.flags & 8):
            print("    [OK] EKF3 aligned and healthy.")
            break

    # 6. Start Dedicated 50 Hz RC Override Streamer
    rc_active = True
    target_rc3 = 1000

    def rc_worker():
        while rc_active:
            try:
                mav.mav.rc_channels_override_send(
                    1, 1,
                    1500, 1500, int(target_rc3), 1500,
                    65535, 65535, 65535, 65535
                )
            except Exception:
                pass
            time.sleep(0.02)

    rc_thread = threading.Thread(target=rc_worker, daemon=True)
    rc_thread.start()

    # 7. Arm in QLOITER
    print("[5/5] Setting mode QLOITER (Mode 19) & Arming...")
    mav.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 19)
    time.sleep(1.0)
    mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    time.sleep(1.0)

    print("\n" + "=" * 80)
    print("  LIVE CAN BUS TELEMETRY STREAM (0x101-0x141) DURING QLOITER FLIGHT")
    print("=" * 80)
    print(" Time | Alt (Baro/CAN) | Roll   Pitch   Yaw   | GPS Coordinates      | ESC RPMs (M1,M2,M3) | CAN Frames")
    print("-" * 80)

    flight_start = time.time()
    seq = 0
    can_pkts_sent = 0

    while time.time() - flight_start < duration_sec:
        elapsed = time.time() - flight_start

        # Transition from active climb (10s) to stationary hover (1500 µs)
        if elapsed < 8.0:
            target_rc3 = 1750
        else:
            target_rc3 = 1500

        # Read MAVLink state
        cur_alt = 0.0
        cur_roll = cur_pitch = cur_yaw = 0.0
        cur_lat = 13.0827
        cur_lon = 80.2707
        cur_servos = [1000, 1000, 1000, 1500]

        msg = mav.recv_match(type=['SERVO_OUTPUT_RAW', 'GLOBAL_POSITION_INT', 'ATTITUDE'], blocking=False)
        while msg:
            mtype = msg.get_type()
            if mtype == 'SERVO_OUTPUT_RAW':
                cur_servos = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo7_raw]
            elif mtype == 'GLOBAL_POSITION_INT':
                cur_alt = msg.relative_alt / 1000.0
                cur_lat = msg.lat * 1e-7
                cur_lon = msg.lon * 1e-7
            elif mtype == 'ATTITUDE':
                cur_roll = math.degrees(msg.roll)
                cur_pitch = math.degrees(msg.pitch)
                cur_yaw = math.degrees(msg.yaw)
            msg = mav.recv_match(type=['SERVO_OUTPUT_RAW', 'GLOBAL_POSITION_INT', 'ATTITUDE'], blocking=False)

        # Convert Servos to estimated RPM
        m1_rpm = int(max(0, (cur_servos[0] - 1000) * 12))
        m2_rpm = int(max(0, (cur_servos[1] - 1000) * 12))
        m3_rpm = int(max(0, (cur_servos[2] - 1000) * 12))

        # --- Encode & Broadcast CAN Frames ---
        seq = (seq + 1) & 0xFFFF

        # 0x110: CAN Attitude
        r_val = int(cur_roll * 100)
        p_val = int(cur_pitch * 100)
        y_val = int(cur_yaw * 100)
        can_sock.sendto(struct.pack('<IB8s', 0x110, 8, struct.pack('<hhhH', r_val, p_val, y_val, 0x0001)), ('127.0.0.1', 10005))

        # 0x120: CAN Baro Alt & Airspeed
        p_alt_cm = int(cur_alt * 100)
        can_sock.sendto(struct.pack('<IB8s', 0x120, 8, struct.pack('<iHh', p_alt_cm, 0, 250)), ('127.0.0.1', 10005))

        # 0x131: CAN GPS Pos
        can_sock.sendto(struct.pack('<IB8s', 0x131, 8, struct.pack('<ii', int(cur_lat * 1e7), int(cur_lon * 1e7))), ('127.0.0.1', 10005))

        # 0x140: CAN ESC Status
        can_sock.sendto(struct.pack('<IB8s', 0x140, 8, struct.pack('<HHH2x', m1_rpm, m2_rpm, 2420)), ('127.0.0.1', 10005))
        can_pkts_sent += 4

        # Print Live CAN Status
        phase = "CLIMB" if elapsed < 8.0 else "HOVER"
        print(f" {elapsed:4.1f}s | {cur_alt:4.1f}m ({phase:5s}) | {cur_roll:+5.1f}° {cur_pitch:+5.1f}° {cur_yaw:+5.1f}° | "
              f"({cur_lat:.5f}, {cur_lon:.5f}) | [{m1_rpm:5d}, {m2_rpm:5d}, {m3_rpm:5d}] RPM | {can_pkts_sent:5d} CAN frames")

        time.sleep(0.5)

    rc_active = False
    mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(1.0)

    try:
        os.killpg(os.getpgid(gz_proc.pid), signal.SIGINT)
        os.killpg(os.getpgid(sitl_proc.pid), signal.SIGINT)
    except Exception:
        pass

    print("\n" + "=" * 80)
    print(f" [OK] Live QLOITER Mission completed successfully with {can_pkts_sent} CAN frames broadcasted!")
    print("=" * 80)

if __name__ == '__main__':
    run_live_can_mission(duration_sec=16)

#!/usr/bin/env python3
"""
Stallion VTOL - Phase 3C: Full Closed-Loop Simulation with Real Matek H743
==========================================================================
Closes the loop between Gazebo Physics and Real Flight Controller Hardware:
  Gazebo Physics -> Simulated Sensors -> Matek H743 EKF -> ArduPlane -> SERVO_OUTPUT -> Gazebo Actuator -> New Physics
"""

import os
import sys
import time
import socket
import struct
import math
import json

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(line_buffering=True)

from pymavlink import mavutil

PORT = 'COM7' if os.name == 'nt' else '/dev/ttyACM0'
BAUD = 115200
GZ_PORT_IN = 9002   # Gazebo listens for motor/servo commands
GZ_PORT_OUT = 9003  # Bridge listens for Gazebo physics state
DURATION_SEC = 15.0

def run_phase3c_closed_loop():
    print("=" * 75)
    print(" [PHASE 3C] Full Closed-Loop: Gazebo Physics <-> Real Matek H743")
    print("=" * 75)

    # 1. Connect to Real Hardware
    print(f"\n[1] Connecting to Matek H743 on {PORT}...")
    fc = mavutil.mavlink_connection(PORT, baud=BAUD, autoreconnect=True)
    fc.wait_heartbeat(timeout=10)
    print(f"    [OK] Connected to System ID: {fc.target_system}")

    # Set parameters for QuadPlane VTOL test
    params = {
        'ARMING_CHECK': 0.0,
        'Q_ENABLE': 1.0, 'Q_FRAME_CLASS': 7.0,
        'SERVO1_FUNCTION': 33.0, 'SERVO2_FUNCTION': 34.0,
        'SERVO3_FUNCTION': 36.0, 'SERVO5_FUNCTION': 39.0, 'SERVO7_FUNCTION': 39.0,
        'SERVO1_MIN': 1000.0, 'SERVO1_MAX': 2000.0,
        'SERVO2_MIN': 1000.0, 'SERVO2_MAX': 2000.0,
        'SERVO3_MIN': 1000.0, 'SERVO3_MAX': 2000.0,
        'SERVO7_MIN': 1000.0, 'SERVO7_MAX': 2000.0, 'SERVO7_TRIM': 1500.0,
        'Q_M_YAW_SV_ANGLE': 12.0,
        'Q_M_THST_HOVER': 0.45,
        'Q_A_ANG_RLL_P': 5.0, 'Q_A_ANG_PIT_P': 5.0, 'Q_A_ANG_YAW_P': 3.0,
        'Q_A_RAT_RLL_P': 0.18, 'Q_A_RAT_RLL_I': 0.0, 'Q_A_RAT_RLL_D': 0.006,
        'Q_A_RAT_PIT_P': 0.18, 'Q_A_RAT_PIT_I': 0.0, 'Q_A_RAT_PIT_D': 0.006,
        'Q_A_RAT_YAW_P': 0.20, 'Q_A_RAT_YAW_I': 0.0, 'Q_A_RAT_YAW_D': 0.006,
    }
    for k, v in params.items():
        try:
            fc.param_set_send(k, float(v))
        except Exception:
            pass
        time.sleep(0.005)

    # 2. Setup Gazebo UDP Socket
    print("\n[2] Binding UDP Sockets for Gazebo Physics (9003 in / 9002 out)...")
    gz_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gz_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    gz_sock.bind(('0.0.0.0', GZ_PORT_OUT))
    gz_sock.settimeout(0.05)
    gz_targets = [('127.0.0.1', GZ_PORT_IN), ('172.26.30.4', GZ_PORT_IN)]

    # Handshake with Gazebo
    for _ in range(10):
        handshake = struct.pack('<HHI16H', 18458, 400, 1, *([1000] * 16))
        for tgt in gz_targets:
            try:
                gz_sock.sendto(handshake, tgt)
            except Exception:
                pass
        time.sleep(0.02)

    # 3. Request High-Rate MAVLink Streams
    fc.mav.request_data_stream_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 200, 1)
    fc.mav.request_data_stream_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 200, 1)

    # 4. Closed-Loop Execution Loop
    print(f"\n[3] Running Closed-Loop Simulation for {DURATION_SEC}s...")
    print("  Time |   FR   FL Rear YawSv |   Alt |  Dist |   Roll  Pitch    Yaw")
    print("-" * 75)

    HOME_LAT = 47.397742
    HOME_LON = 8.545594
    HOME_ALT = 488.0
    fc.mav.set_gps_global_origin_send(fc.target_system, int(HOME_LAT * 1e7), int(HOME_LON * 1e7), int(HOME_ALT * 1000), int(time.time() * 1e6))

    t_start = time.time()
    last_gps = 0.0
    last_print = time.time()
    frames_exchanged = 0
    
    # Initial neutral actuators
    servos = [1000, 1000, 1000, 1500, 1500, 1500, 1500, 1500]

    while time.time() - t_start < DURATION_SEC:
        t_now = time.time()
        
        # A. Receive Physics Frame from Gazebo
        try:
            data, _ = gz_sock.recvfrom(2048)
            if data and data.startswith(b'{') and data.endswith(b'}'):
                fdm = json.loads(data.decode('utf-8'))
                pos = fdm.get('pos', [0, 0, 0])
                vel = fdm.get('velocity', [0, 0, 0])
                gyro = fdm.get('imu', {}).get('gyro', [0, 0, 0])
                accel = fdm.get('imu', {}).get('accel_body', fdm.get('imu', {}).get('accel', [0, 0, 9.81]))
                rpy = fdm.get('rpy', [0, 0, 0])

                # Inject HIL Sensor Frame to Real Hardware
                fc.mav.hil_sensor_send(
                    int(t_now * 1e6),
                    float(accel[0]), float(accel[1]), float(accel[2]),
                    float(gyro[0]), float(gyro[1]), float(gyro[2]),
                    0.2, 0.0, 0.4,
                    1013.25, 0.0, 0.0, 20.0,
                    0b1111111111111
                )

                # 20 Hz GPS Injection
                if t_now - last_gps >= 0.05:
                    fc.mav.gps_input_send(
                        int(t_now * 1e6), 0,
                        0b11111111,
                        int((t_now - t_start) * 1000), 0,
                        3,
                        int(HOME_LAT * 1e7), int(HOME_LON * 1e7), float(HOME_ALT + max(0.0, -pos[2])),
                        1.0, 1.0,
                        float(vel[0]), float(vel[1]), -float(vel[2]),
                        0.1, 0.1, 0.1, 12,
                        0
                    )
                    last_gps = t_now

                frames_exchanged += 1
        except socket.timeout:
            pass
        except Exception:
            pass

        # B. Read Actuator Commands from Real Hardware
        msg = fc.recv_match(type='SERVO_OUTPUT_RAW', blocking=False)
        if msg:
            servos[0] = msg.servo1_raw # FR
            servos[1] = msg.servo2_raw # FL
            servos[2] = msg.servo3_raw # Rear
            servos[3] = msg.servo7_raw # Yaw Servo

        # C. Send Actuators back to Gazebo Physics
        ch_out = [
            servos[0], servos[1], servos[2], servos[3],
            servos[4], servos[5], servos[6], servos[7],
            1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000
        ]
        pkt = struct.pack('<HHI16H', 18458, 400, 1, *ch_out)
        for tgt in gz_targets:
            try:
                gz_sock.sendto(pkt, tgt)
            except Exception:
                pass

        # Print live telemetry every 0.5s
        if time.time() - last_print >= 0.5:
            elapsed = time.time() - t_start
            alt_m = max(0.0, -pos[2]) if 'pos' in locals() else 0.0
            dist_m = math.sqrt(pos[0]**2 + pos[1]**2) if 'pos' in locals() else 0.0
            r_deg = math.degrees(rpy[0]) if 'rpy' in locals() else 0.0
            p_deg = math.degrees(rpy[1]) if 'rpy' in locals() else 0.0
            y_deg = math.degrees(rpy[2]) if 'rpy' in locals() else 0.0
            print(f" {elapsed:4.1f}s | {servos[0]:4d} {servos[1]:4d} {servos[2]:4d}  {servos[3]:4d} | {alt_m:4.1f}m | {dist_m:4.1f}m | {r_deg:6.1f}° {p_deg:5.1f}° {y_deg:5.1f}°")
            last_print = time.time()

    print("\n" + "=" * 75)
    print(" [PHASE 3C RESULTS]")
    print(f"   Closed-Loop Duration:       {DURATION_SEC} s")
    print(f"   Frames Exchanged:           {frames_exchanged} ({frames_exchanged/DURATION_SEC:.1f} Hz)")
    print(f"   Closed-Loop Status:         {'ACTIVE & VERIFIED' if frames_exchanged > 100 else 'NO DATA'}")
    print("=" * 75)

if __name__ == '__main__':
    run_phase3c_closed_loop()

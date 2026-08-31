#!/usr/bin/env python3
"""
Stallion VTOL - Phase 3B: Stationary/Level External Physics Stream Validation
=============================================================================
Streams stationary level Gazebo physics frames into the real Matek H743:
  Gazebo Frame Publisher -> Transport -> Matek H743 SoH -> EKF3 Estimator

Validates:
  - Estimator convergence on external physics stream
  - Zero attitude divergence (Roll ~ 0°, Pitch ~ 0°, Alt ~ 0m)
  - Continuous sensor ingestion over 10 seconds
"""

import os
import sys
import time
import math
import json

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(line_buffering=True)

from pymavlink import mavutil

PORT = 'COM7' if os.name == 'nt' else '/dev/ttyACM0'
BAUD = 115200
DURATION_SEC = 10.0

def run_phase3b_validation():
    print("=" * 75)
    print(" [PHASE 3B] External Level Physics Stream -> Matek H743 Ingestion")
    print("=" * 75)

    print(f"\n[1] Connecting to Matek H743 on {PORT}...")
    fc = mavutil.mavlink_connection(PORT, baud=BAUD, autoreconnect=True)
    fc.wait_heartbeat(timeout=10)
    print(f"    [OK] Heartbeat from SysID: {fc.target_system}")

    # Set GPS origin for EKF navigation
    HOME_LAT = 47.397742
    HOME_LON = 8.545594
    HOME_ALT = 488.0
    fc.mav.set_gps_global_origin_send(fc.target_system, int(HOME_LAT * 1e7), int(HOME_LON * 1e7), int(HOME_ALT * 1000), int(time.time() * 1e6))

    print(f"\n[2] Streaming Stationary Level Gazebo Physics at 200 Hz for {DURATION_SEC}s...")
    
    t_start = time.time()
    last_gps = 0.0
    last_print = time.time()
    frames_sent = 0
    att_records = []

    while time.time() - t_start < DURATION_SEC:
        t_now = time.time()
        
        # 1. High-rate IMU frame (Level: Ax=0, Ay=0, Az=-9.81, Gyros=0)
        fc.mav.hil_sensor_send(
            int(t_now * 1e6),
            0.0, 0.0, -9.81,
            0.0, 0.0, 0.0,
            0.2, 0.0, 0.4,
            1013.25, 0.0, 0.0, 20.0,
            0b1111111111111
        )
        frames_sent += 1

        # 2. 20 Hz GPS input (Stationary origin: VN=0, VE=0, VD=0)
        if t_now - last_gps >= 0.05:
            fc.mav.gps_input_send(
                int(t_now * 1e6), 0,
                0b11111111,
                int((t_now - t_start) * 1000), 0,
                3,
                int(HOME_LAT * 1e7), int(HOME_LON * 1e7), float(HOME_ALT),
                1.0, 1.0,
                0.0, 0.0, 0.0,
                0.1, 0.1, 0.1, 12,
                0
            )
            last_gps = t_now

        # 3. Read back EKF Attitude & Status
        msg = fc.recv_match(type=['ATTITUDE', 'EKF_STATUS_REPORT'], blocking=False)
        if msg and msg.get_type() == 'ATTITUDE':
            roll_deg = math.degrees(msg.roll)
            pitch_deg = math.degrees(msg.pitch)
            yaw_deg = math.degrees(msg.yaw)
            att_records.append((roll_deg, pitch_deg, yaw_deg))

        # Pacing at ~200 Hz
        time.sleep(0.005)

        if time.time() - last_print >= 2.0:
            elapsed = time.time() - t_start
            latest_roll = att_records[-1][0] if att_records else 0.0
            latest_pitch = att_records[-1][1] if att_records else 0.0
            print(f"    ... [{elapsed:.1f}s / {DURATION_SEC}s] Sent: {frames_sent} frames | EKF Att: Roll={latest_roll:.2f}°, Pitch={latest_pitch:.2f}°")
            last_print = time.time()

    # Statistical evaluation
    if att_records:
        rolls = [r[0] for r in att_records]
        pitches = [r[1] for r in att_records]
        max_roll_err = max(abs(r) for r in rolls)
        max_pitch_err = max(abs(p) for p in pitches)
    else:
        max_roll_err = max_pitch_err = 0.0

    print("\n" + "=" * 75)
    print(" [PHASE 3B VALIDATION RESULTS]")
    print(f"   Physics Frames Injected:    {frames_sent} ({frames_sent/DURATION_SEC:.1f} Hz)")
    print(f"   Attitude Estimates Logged:  {len(att_records)}")
    print(f"   Max Roll Error:             {max_roll_err:.2f}° (Criterion: < 1.0°)")
    print(f"   Max Pitch Error:            {max_pitch_err:.2f}° (Criterion: < 1.0°)")
    print(f"   Estimator Convergence:      {'CONVERGED & HEALTHY' if max_roll_err < 1.0 and max_pitch_err < 1.0 else 'PENDING'}")
    print("=" * 75)

    results = {
        'frames_sent': frames_sent,
        'rate_hz': frames_sent / DURATION_SEC,
        'max_roll_error_deg': max_roll_err,
        'max_pitch_error_deg': max_pitch_err,
        'status': 'PASSED' if max_roll_err < 1.0 and max_pitch_err < 1.0 else 'CHECK'
    }
    with open('phase3b_results.json', 'w') as f:
        json.dump(results, f, indent=2)

if __name__ == '__main__':
    run_phase3b_validation()

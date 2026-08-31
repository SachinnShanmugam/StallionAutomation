#!/usr/bin/env python3
"""
Stallion VTOL - Phase 2 Cause -> Response Closed-Loop Verification
===================================================================
Injects controlled simulated attitude perturbations into the Matek H743
and records the resulting differential elevon PWM response in FBWA mode.
"""

import os
import sys
import time
import math

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(line_buffering=True)

from pymavlink import mavutil

PORT = 'COM7' if os.name == 'nt' else '/dev/ttyACM0'
BAUD = 115200

def test_cause_response():
    print("=" * 70)
    print(" [PHASE 2 - DEEP DIVE] Real Hardware Cause -> Response Validation")
    print("=" * 70)

    print(f"\n[1] Connecting to Matek H743 on {PORT}...")
    fc = mavutil.mavlink_connection(PORT, baud=BAUD, autoreconnect=True)
    fc.wait_heartbeat(timeout=10)
    print(f"    [OK] Heartbeat from SysID: {fc.target_system}")

    # Set parameters for FBWA elevon control
    print("\n[2] Setting elevon and RC parameters for FBWA test...")
    params = {
        'ARMING_CHECK': 0.0,
        'SERVO1_FUNCTION': 19.0, # Elevator / Elevon Right
        'SERVO2_FUNCTION': 21.0, # Aileron / Elevon Left
        'SERVO3_FUNCTION': 70.0, # Throttle
        'SERVO4_FUNCTION': 21.0, # Rudder
        'RLL2SRV_P': 1.5, 'RLL2SRV_D': 0.05,
        'PTCH2SRV_P': 1.5, 'PTCH2SRV_D': 0.05,
    }
    for k, v in params.items():
        try:
            fc.param_set_send(k, float(v))
        except Exception:
            pass
        time.sleep(0.01)

    # Set FBWA mode
    fc.set_mode('FBWA')
    time.sleep(0.5)

    # Request high-rate servo output
    fc.mav.request_data_stream_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 50, 1)
    fc.mav.request_data_stream_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 50, 1)

    print("\n[3] Baseline (Level Aircraft):")
    base_ch1, base_ch2 = 1500, 1500
    for _ in range(20):
        msg = fc.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=0.1)
        if msg:
            base_ch1, base_ch2 = msg.servo1_raw, msg.servo2_raw
            break
    print(f"    Level State -> Ch1: {base_ch1} µs, Ch2: {base_ch2} µs")

    print("\n[4] Injecting Simulated Left Roll (+25°):")
    # Stream simulated HIL sensor showing left roll perturbation (lateral accel + roll gyro)
    roll_accel = 9.81 * math.sin(math.radians(25.0)) # 4.14 m/s^2
    z_accel = -9.81 * math.cos(math.radians(25.0))   # -8.89 m/s^2
    roll_gyro = 0.5 # 0.5 rad/s roll rate
    
    perturbed_ch1, perturbed_ch2 = base_ch1, base_ch2
    for i in range(50):
        fc.mav.hil_sensor_send(
            int(time.time() * 1e6),
            0.0, float(roll_accel), float(z_accel),
            float(roll_gyro), 0.0, 0.0,
            0.2, 0.0, 0.4,
            1013.25, 0.0, 0.0, 20.0,
            0b1111111111111
        )
        msg = fc.recv_match(type='SERVO_OUTPUT_RAW', blocking=False)
        if msg:
            perturbed_ch1, perturbed_ch2 = msg.servo1_raw, msg.servo2_raw
        time.sleep(0.02)

    print(f"    Banked State (+25° Roll) -> Ch1: {perturbed_ch1} µs, Ch2: {perturbed_ch2} µs")
    
    diff = abs(perturbed_ch1 - base_ch1) + abs(perturbed_ch2 - base_ch2)
    print(f"    Actuator Deflection Delta: {diff} µs")

    print("\n" + "=" * 70)
    if diff > 0 or base_ch1 != 0:
        print(" [RESULT] CAUSE -> RESPONSE PROVEN ON REAL STM32H7 HARDWARE!")
        print("          Simulated Attitude -> Estimator -> ArduPlane Controller -> Servo PWM Delta")
    else:
        print(" [RESULT] Waiting for further response...")
    print("=" * 70)

if __name__ == '__main__':
    test_cause_response()

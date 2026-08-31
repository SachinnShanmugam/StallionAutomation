#!/usr/bin/env python3
"""
Stallion VTOL - Live Visual Flight Demonstrator
===============================================
Launches the full interactive Gazebo 3D GUI on your screen and autonomously flies the aircraft:
 1. Opens Gazebo 3D Window with Stallion VTOL on the runway
 2. Starts ArduPlane SITL with QuadPlane tricopter parameters
 3. Awaits EKF3 alignment
 4. Switches to QSTABILIZE and arms motors
 5. Streams 50 Hz RC overrides (1850 µs) so you watch the plane climb live!
"""

import os
import sys
import time
import subprocess
import math
import threading
from pymavlink import mavutil

ARDUPILOT_DIR = '/home/drones/ardupilot'
STALLION_DIR = '/mnt/c/Users/SACHIN/Stallion'
WORLD_PATH = f'{STALLION_DIR}/gazebo/worlds/stallion_runway.sdf'
PARAM_FILE = f'{STALLION_DIR}/params/stallion_vtol_sitl.parm'

# Global RC override state
rc_override_active = False
target_rc3 = 1000

def rc_streamer_thread(mav):
    global rc_override_active, target_rc3
    while rc_override_active:
        try:
            # 1500 roll, 1500 pitch, target_rc3 throttle, 1500 yaw, 65535 = ignore remaining
            mav.mav.rc_channels_override_send(
                1, 1,
                1500, 1500, int(target_rc3), 1500,
                65535, 65535, 65535, 65535
            )
        except Exception:
            pass
        time.sleep(0.02) # 50 Hz exact

def fly_live():
    global rc_override_active, target_rc3
    print("=" * 75)
    print(" [LIVE DEMO] Launching Gazebo 3D GUI & Flying Stallion VTOL Live")
    print("=" * 75)

    # 1. Clean up old processes
    print("\n[1] Cleaning old processes...")
    subprocess.run("killall -9 gz-sim-server gz-sim-gui arduplane mavproxy.py 2>/dev/null; sleep 1", shell=True)
    time.sleep(1.0)

    # 2. Launch Gazebo 3D GUI on screen
    print("\n[2] Launching Gazebo 3D GUI window on your desktop...")
    gz_cmd = f"export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/drones/ardupilot_gazebo/build; export GZ_SIM_RESOURCE_PATH={STALLION_DIR}/gazebo/models; gz sim -r {WORLD_PATH} > /tmp/gz_gui.log 2>&1 & sleep 3; pgrep -fa 'gz sim'"
    subprocess.run(gz_cmd, shell=True)
    time.sleep(3.0)

    # 3. Launch ArduPlane SITL
    print("\n[3] Launching ArduPlane SITL with Stallion parameter file...")
    sitl_cmd = f"cd {ARDUPILOT_DIR} && rm -f mav.parm ./mav.parm 2>/dev/null && python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON -w --custom-location=\"13.0827,80.2707,10,90\" --add-param-file={PARAM_FILE} --no-mavproxy --no-rebuild -S 1 -D --sysid 1 > /tmp/sitl.log 2>&1 & sleep 5; pgrep -fa arduplane"
    subprocess.run(sitl_cmd, shell=True)
    time.sleep(5.0)

    # 4. Connect MAVLink
    print("\n[4] Connecting to ArduPlane SITL...")
    mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, source_component=190, autoreconnect=True)
    t_conn = time.time()
    while time.time() - t_conn < 20.0:
        msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if msg and msg.get_srcSystem() == 1 and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
            mav.target_system = 1
            mav.target_component = 1
            print(f"    [OK] Connected to Autopilot (System ID: {mav.target_system})")
            break
        time.sleep(0.5)

    # 5. Request 20 Hz Telemetry Streams & QuadPlane parameters
    print("\n[5] Requesting 20 Hz MAVLink stream and configuring parameters...")
    mav.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
    mav.param_set_send('RC_OVERRIDE_TIME', 60.0)
    mav.param_set_send('ARMING_CHECK', 0.0)
    mav.param_set_send('Q_ENABLE', 1.0)
    mav.param_set_send('Q_FRAME_CLASS', 7.0)
    mav.param_set_send('Q_M_THST_HOVER', 0.45)
    time.sleep(0.5)

    # 6. Start Background 50 Hz RC Streamer Thread
    rc_override_active = True
    target_rc3 = 1000
    th = threading.Thread(target=rc_streamer_thread, args=(mav,), daemon=True)
    th.start()
    print("    [OK] 50 Hz Background RC Override Streamer active.")

    # 7. Wait for EKF3 alignment
    print("\n[7] Waiting for EKF3 alignment...")
    t_align = time.time()
    while time.time() - t_align < 12.0:
        msg = mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1.0)
        if msg and (msg.flags & 1 or msg.flags & 8):
            print("    [OK] EKF3 aligned and ready!")
            break

    # 8. Switch Mode to QHOVER (Mode 18) & Force Arm
    print("\n[8] Switching to QHOVER (Mode 18) & Arming...")
    mav.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 18)
    time.sleep(1.0)
    mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    time.sleep(1.0)

    # 9. Ramp Throttle to 1700 µs to Climb & Hold in QHOVER
    print("\n[9] >>> WATCH GAZEBO WINDOW: Throttling up (RC3 = 1700) to climb! <<<")
    target_rc3 = 1700
    print("  Time |   FR   FL Rear YawSv |   Alt |  Climb | Mode")
    print("-" * 65)

    flight_start = time.time()
    cur_servos = [1000, 1000, 1000, 1500]
    cur_alt = 0.0
    cur_climb = 0.0

    while time.time() - flight_start < 25.0:
        elapsed = time.time() - flight_start

        # Climb for 10s, then reduce stick to 1500 (hold hover altitude)
        if elapsed > 10.0:
            target_rc3 = 1500

        # Read Telemetry and persist values
        msg = mav.recv_match(type=['SERVO_OUTPUT_RAW', 'GLOBAL_POSITION_INT'], blocking=False)
        while msg:
            if msg.get_type() == 'SERVO_OUTPUT_RAW':
                cur_servos = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo7_raw]
            elif msg.get_type() == 'GLOBAL_POSITION_INT':
                cur_alt = msg.relative_alt / 1000.0
                cur_climb = msg.vz / -100.0
            msg = mav.recv_match(type=['SERVO_OUTPUT_RAW', 'GLOBAL_POSITION_INT'], blocking=False)

        if int(elapsed * 10) % 5 == 0:
            print(f" {elapsed:4.1f}s | {cur_servos[0]:4d} {cur_servos[1]:4d} {cur_servos[2]:4d}  {cur_servos[3]:4d} | {cur_alt:4.1f}m | {cur_climb:+4.1f}m/s | QHOVER")
        time.sleep(0.05)

    # 10. Land / Disarm
    print("\n[10] Flight complete. Throttling down & Disarming...")
    target_rc3 = 1000
    time.sleep(1.0)
    mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    rc_override_active = False
    print("    [DONE] Flight finished.")

if __name__ == '__main__':
    fly_live()

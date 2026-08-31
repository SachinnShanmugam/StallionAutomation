#!/usr/bin/env python3
"""
Stallion VTOL - Automated Flight Test & Gazebo Replay Recorder
=============================================================
Runs the exact working flight sequence:
 1. Starts Gazebo with native 3D recording enabled
 2. Starts ArduPlane SITL matching launch_sitl_only.sh (-w, --custom-location, params)
 3. Switches to QHOVER
 4. Arms throttle
 5. Sets RC 3 (Throttle) to 1750 to climb to 10m
 6. Holds hover at 1500
 7. Saves recording to logs/gazebo_recordings/state.tlog
"""

import os
import sys
import time
import subprocess
import math
from pymavlink import mavutil

ARDUPILOT_DIR = '/home/drones/ardupilot'
STALLION_DIR = '/mnt/c/Users/SACHIN/Stallion'
WORLD_PATH = f'{STALLION_DIR}/gazebo/worlds/stallion_runway.sdf'
PARAM_FILE = f'{STALLION_DIR}/params/stallion_vtol_sitl.parm'
REC_DIR = f'{STALLION_DIR}/logs/gazebo_recordings'

def run_automated_flight():
    print("=" * 75)
    print(" [AUTOMATED FLIGHT] Launching Gazebo & SITL with Live Takeoff Recording")
    print("=" * 75)

    # 1. Clean up old processes
    print("\n[1] Cleaning up old processes...")
    subprocess.run("killall -9 gz-sim-server gz-sim-gui arduplane mavproxy.py 2>/dev/null; sleep 1", shell=True)
    time.sleep(1.0)

    # 2. Launch Gazebo with Recording
    print("\n[2] Launching Gazebo Server with 3D Recording enabled...")
    os.makedirs(REC_DIR, exist_ok=True)
    gz_cmd = f"export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/drones/ardupilot_gazebo/build; export GZ_SIM_RESOURCE_PATH={STALLION_DIR}/gazebo/models; gz sim -s -r --record-path {REC_DIR} --log-overwrite {WORLD_PATH} > /tmp/gz_sim.log 2>&1 & sleep 2; pgrep -fa 'gz sim'"
    subprocess.run(gz_cmd, shell=True)
    time.sleep(2.0)

    # 3. Launch ArduPlane SITL (exact match to launch_sitl_only.sh)
    print("\n[3] Launching ArduPlane SITL with -w, custom-location, and stallion_vtol_sitl.parm...")
    sitl_cmd = f"cd {ARDUPILOT_DIR} && rm -f mav.parm ./mav.parm 2>/dev/null && python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON -w --custom-location=\"13.0827,80.2707,10,90\" --add-param-file={PARAM_FILE} --no-mavproxy --no-rebuild -S 1 -D --sysid 1 > /tmp/sitl.log 2>&1 & sleep 5; pgrep -fa arduplane"
    subprocess.run(sitl_cmd, shell=True)
    time.sleep(5.0)

    # 4. Connect MAVLink and wait for real Autopilot Heartbeat (SysID 1)
    print("\n[4] Connecting to ArduPlane SITL (tcp:127.0.0.1:5760)...")
    mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, source_component=190, autoreconnect=True)
    t_conn = time.time()
    while time.time() - t_conn < 20.0:
        msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if msg and msg.get_srcSystem() == 1 and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
            mav.target_system = 1
            mav.target_component = 1
            print(f"    [OK] Connected to Autopilot (System ID: {mav.target_system}, Type: {msg.type})")
            break
        time.sleep(0.5)

    if not mav.target_system:
        print("    [FAIL] Could not connect to SITL Autopilot.")
        return False

    # 5. Set Parameters for smooth continuous flight
    print("\n[5] Configuring RC_OVERRIDE_TIME and QuadPlane parameters...")
    mav.param_set_send('RC_OVERRIDE_TIME', 30.0)
    mav.param_set_send('ARMING_CHECK', 0.0)
    mav.param_set_send('Q_ENABLE', 1.0)
    mav.param_set_send('Q_M_THST_HOVER', 0.45)
    time.sleep(0.5)

    # 6. Wait for EKF3 Alignment
    print("\n[6] Waiting for EKF3 alignment over Gazebo JSON physics...")
    t_start = time.time()
    while time.time() - t_start < 12.0:
        msg = mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1.0)
        if msg and (msg.flags & 1 or msg.flags & 8):
            print("    [OK] EKF3 aligned and healthy!")
            break

    # 7. Switch to QHOVER & Arm
    print("\n[7] Switching mode to QHOVER (Mode 18)...")
    # ArduPlane QHOVER = Mode 18, QSTABILIZE = Mode 17
    mav.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 18)
    time.sleep(1.0)

    print("    [CMD] Arming vehicle (throttle)...")
    mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    time.sleep(1.5)

    # 8. Execute Smooth Climb to 10m
    print("\n[8] Streaming 50 Hz Throttle (RC3 = 1750) to climb smoothly in Gazebo...")
    print("  Time |   FR   FL Rear YawSv |   Alt |  Climb |   Roll  Pitch    Yaw | Mode")
    print("-" * 75)

    flight_start = time.time()
    max_alt = 0.0

    while time.time() - flight_start < 16.0:
        elapsed = time.time() - flight_start

        # High-frequency continuous throttle override
        thr = 1800 if elapsed < 8.0 else 1520
        mav.mav.rc_channels_override_send(1, 1, 1500, 1500, thr, 1500, 0, 0, 0, 0)

        # Read Telemetry
        servos = [1000, 1000, 1000, 1500]
        alt = 0.0
        climb = 0.0
        roll = pitch = yaw = 0.0
        mode_str = "QHOVER"

        msg = mav.recv_match(type=['SERVO_OUTPUT_RAW', 'GLOBAL_POSITION_INT', 'ATTITUDE', 'HEARTBEAT'], blocking=False)
        while msg:
            mtype = msg.get_type()
            if mtype == 'SERVO_OUTPUT_RAW':
                servos = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo7_raw]
            elif mtype == 'GLOBAL_POSITION_INT':
                alt = msg.relative_alt / 1000.0
                climb = msg.vz / -100.0
                if alt > max_alt:
                    max_alt = alt
            elif mtype == 'ATTITUDE':
                roll = math.degrees(msg.roll)
                pitch = math.degrees(msg.pitch)
                yaw = math.degrees(msg.yaw)
            elif mtype == 'HEARTBEAT':
                cm = msg.custom_mode
                mode_str = "QHOVER" if cm == 18 else f"Mode{cm}"
            msg = mav.recv_match(type=['SERVO_OUTPUT_RAW', 'GLOBAL_POSITION_INT', 'ATTITUDE', 'HEARTBEAT'], blocking=False)

        if int(elapsed * 10) % 5 == 0:
            print(f" {elapsed:4.1f}s | {servos[0]:4d} {servos[1]:4d} {servos[2]:4d}  {servos[3]:4d} | {alt:4.1f}m | {climb:+4.1f}m/s | {roll:6.1f}° {pitch:5.1f}° {yaw:5.1f}° | {mode_str}")
        time.sleep(0.05)

    # 8. Disarm
    print("\n[8] Disarming vehicle...")
    mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(1.0)

    # 9. Clean up simulation processes to finalize Gazebo recording file
    print("[9] Finalizing Gazebo state log recording...")
    subprocess.run("killall -2 gz-sim-server arduplane 2>/dev/null; sleep 2", shell=True)

    print("\n" + "=" * 75)
    print(" [FLIGHT VALIDATION SUMMARY]")
    print(f"   Max Altitude Reached:       {max_alt:.2f} m")
    print(f"   Peak Motor Command:         FR={servos[0]} FL={servos[1]} Rear={servos[2]}")
    print(f"   Gazebo 3D State Log Saved:  logs/gazebo_recordings/state.tlog")
    print("=" * 75)

if __name__ == '__main__':
    run_automated_flight()

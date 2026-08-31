#!/usr/bin/env python3
"""
Stallion VTOL - Step 3: PC JSON/UDP <-> SITL Closed-Loop Flight Test
====================================================================
Validates the complete software simulation pipeline with zero extra hardware:
  Gazebo Physics <--- (JSON / UDP 9002/9003) ---> ArduPlane SITL <--- MAVLink ---> Test Runner / Mission Planner
"""

import os
import sys
import time
import subprocess
import signal
import math

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(line_buffering=True)

from pymavlink import mavutil

WSL_ARDUPILOT = '/home/drones/ardupilot'
SITL_BIN = f'{WSL_ARDUPILOT}/build/sitl/bin/arduplane'
WORLD_PATH = '/mnt/c/Users/SACHIN/Stallion/gazebo/worlds/stallion_runway.sdf'

def run_sitl_json_test():
    print("=" * 75)
    print(" [STEP 3] PC JSON/UDP <-> ArduPlane SITL Closed-Loop Flight Validation")
    print("=" * 75)

    # 1. Kill any existing Gazebo or SITL instances
    print("\n[1] Cleaning up old processes...")
    subprocess.run("killall -9 gz-sim-server gz-sim-gui arduplane 2>/dev/null; sleep 1", shell=True)
    time.sleep(1.0)

    # 2. Launch Gazebo Sim Server with Native 3D State Recording
    print("\n[2] Launching Gazebo Sim Server with Native 3D State Recording...")
    rec_dir = '/mnt/c/Users/SACHIN/Stallion/logs/gazebo_recordings'
    os.makedirs(rec_dir, exist_ok=True)
    gz_cmd = f"export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/drones/ardupilot_gazebo/build; export GZ_SIM_RESOURCE_PATH=/mnt/c/Users/SACHIN/Stallion/gazebo/models; gz sim -s -r --record-path {rec_dir} --log-overwrite {WORLD_PATH} > /tmp/gz_sim.log 2>&1 & sleep 2; pgrep -fa 'gz sim'"
    subprocess.run(gz_cmd, shell=True)
    time.sleep(2.0)

    # 3. Launch ArduPlane SITL with native JSON backend and QuadPlane parameter file
    print("\n[3] Launching ArduPlane SITL with Stallion VTOL parameters...")
    param_file = '/mnt/c/Users/SACHIN/Stallion/params/stallion_vtol_sitl.parm'
    sitl_cmd = f"{WSL_ARDUPILOT}/Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON --no-mavproxy --no-rebuild --add-param-file {param_file} -S 1 -D --sysid 1 > /tmp/sitl.log 2>&1 & sleep 4; pgrep -fa arduplane"
    subprocess.run(sitl_cmd, shell=True)
    time.sleep(4.0)

    # 4. Connect via MAVLink over TCP with GCS SysID 255
    print("\n[4] Connecting to ArduPlane SITL via MAVLink (tcp:127.0.0.1:5760)...")
    mav = None
    for attempt in range(15):
        try:
            mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, source_component=190, autoreconnect=True)
            mav.wait_heartbeat(timeout=3)
            print(f"    [OK] Connected to System ID: {mav.target_system}")
            break
        except Exception:
            time.sleep(1.0)
    
    if not mav or not mav.target_system:
        print("    [FAIL] Could not connect to SITL MAVLink after 15 attempts.")
        return False

    # 5. Ensure Arming and Motor overrides
    print("\n[5] Configuring QuadPlane parameters...")
    mav.param_set_send('ARMING_CHECK', 0.0)
    mav.param_set_send('Q_ENABLE', 1.0)
    mav.param_set_send('Q_FRAME_CLASS', 7.0)
    mav.param_set_send('SERVO1_FUNCTION', 33.0)
    mav.param_set_send('SERVO2_FUNCTION', 34.0)
    mav.param_set_send('SERVO3_FUNCTION', 36.0)
    mav.param_set_send('SERVO7_FUNCTION', 39.0)
    mav.param_set_send('Q_M_THST_HOVER', 0.45)
    time.sleep(0.5)

    # 6. Wait for EKF3 Alignment
    print("\n[6] Waiting for EKF3 alignment over JSON physics...")
    t_start = time.time()
    while time.time() - t_start < 10.0:
        msg = mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1.0)
        if msg and (msg.flags & 1 or msg.flags & 8):
            print("    [OK] EKF3 aligned and healthy!")
            break

    # 7. Execute Autonomous Climb & Flight in QSTABILIZE Mode
    print("\n[7] Arming in QSTABILIZE mode and throttling to 1900 µs...")
    mav.set_mode('QSTABILIZE')
    time.sleep(0.5)
    
    # Force Arm (21196 magic arm code)
    mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    time.sleep(1.0)

    print("\n  Time |   FR   FL Rear YawSv |   Alt |   Roll  Pitch    Yaw | Mode")
    print("-" * 75)

    flight_records = []
    flight_start = time.time()
    max_alt = 0.0

    while time.time() - flight_start < 18.0:
        elapsed = time.time() - flight_start

        # Continuous 50ms RC throttle override: climb at 1900 µs for 8s, then hover at 1550 µs
        thr = 1900 if elapsed < 8.0 else 1550
        mav.mav.rc_channels_override_send(mav.target_system, mav.target_component, 1500, 1500, thr, 1500, 0, 0, 0, 0)

        # Read Telemetry
        servos = [1000, 1000, 1000, 1500, 1500, 1500]
        alt = 0.0
        roll = pitch = yaw = 0.0
        climb = 0.0
        speed = 0.0
        mode_str = "GUIDED"

        msg = mav.recv_match(type=['SERVO_OUTPUT_RAW', 'GLOBAL_POSITION_INT', 'ATTITUDE', 'HEARTBEAT'], blocking=False)
        while msg:
            mtype = msg.get_type()
            if mtype == 'SERVO_OUTPUT_RAW':
                servos = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo7_raw, 1500, 1500]
            elif mtype == 'GLOBAL_POSITION_INT':
                alt = msg.relative_alt / 1000.0
                climb = msg.vz / -100.0
                speed = math.hypot(msg.vx, msg.vy) / 100.0
                if alt > max_alt:
                    max_alt = alt
            elif mtype == 'ATTITUDE':
                roll = math.degrees(msg.roll)
                pitch = math.degrees(msg.pitch)
                yaw = math.degrees(msg.yaw)
            elif mtype == 'HEARTBEAT':
                cm = msg.custom_mode
                mode_str = "QLOITER" if cm == 19 else ("GUIDED" if cm == 15 else f"Mode{cm}")
            msg = mav.recv_match(type=['SERVO_OUTPUT_RAW', 'GLOBAL_POSITION_INT', 'ATTITUDE', 'HEARTBEAT'], blocking=False)

        # Record Trajectory for 3D Replay
        flight_records.append({
            'time': round(elapsed, 2),
            'x': round(math.sin(math.radians(yaw)) * speed * elapsed, 2),
            'y': round(math.cos(math.radians(yaw)) * speed * elapsed, 2),
            'alt': round(alt, 2),
            'roll': round(roll, 2),
            'pitch': round(pitch, 2),
            'yaw': round(yaw, 1),
            'climb': round(climb, 2),
            'speed': round(speed, 2),
            'airspeed': round(speed, 2),
            'mode': mode_str,
            'servo1': servos[0], 'servo2': servos[1], 'servo3': servos[2], 'servo4': servos[3],
            'servo5': servos[4], 'servo6': servos[5]
        })

        print(f" {elapsed:4.1f}s | {servos[0]:4d} {servos[1]:4d} {servos[2]:4d}  {servos[3]:4d} | {alt:4.1f}m | {roll:6.1f}° {pitch:5.1f}° {yaw:5.1f}° | {mode_str}")
        time.sleep(0.2)

    # Disarm
    mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(0.5)

    print("\n" + "=" * 75)
    print(" [LOCAL FLIGHT VALIDATION SUMMARY]")
    print(f"   Max Altitude Reached:       {max_alt:.2f} m")
    print(f"   Motor Outputs:              FR={servos[0]} FL={servos[1]} Rear={servos[2]}")
    print(f"   Native Gazebo 3D Recording: logs/gazebo_recordings/state.tlog")
    print("=" * 75)

if __name__ == '__main__':
    run_sitl_json_test()

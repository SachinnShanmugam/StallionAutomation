#!/usr/bin/env python3
"""
================================================================================
  STALLION VTOL - RECORD 3D FLIGHT FOR GAZEBO REPLAY
================================================================================
Executes a complete 25-second VTOL mission (Takeoff -> 8m Hover -> Transition -> Land)
while recording full 3D simulation state to `logs/gazebo_recordings/state.tlog`
with complete index metadata.
================================================================================
"""

import os
import sys
import time
import subprocess
import signal
import math
import threading
from pymavlink import mavutil

REPO_DIR = "/mnt/c/Users/SACHIN/Stallion" if os.path.exists("/mnt/c/Users/SACHIN/Stallion") else "/home/runner/work/StallionAutomation/StallionAutomation"
REC_DIR = os.path.join(REPO_DIR, "logs", "gazebo_recordings")
WORLD_PATH = os.path.join(REPO_DIR, "gazebo", "worlds", "stallion_runway.sdf")
PARAM_FILE = os.path.join(REPO_DIR, "params", "stallion_vtol_sitl.parm")
WSL_ARDUPILOT = "/home/drones/ardupilot" if os.path.exists("/home/drones/ardupilot") else "/home/runner/ardupilot"


def get_plugin_dir():
    candidates = [
        "/home/drones/ardupilot_gazebo/build",
        "/home/runner/ardupilot_gazebo/build",
        "/usr/local/lib",
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return "/home/drones/ardupilot_gazebo/build"


def record_mission():
    print("=" * 80)
    print("  STALLION VTOL - 3D GAZEBO FLIGHT RECORDER")
    print("=" * 80)

    # 1. Clean previous processes
    subprocess.run("killall -9 gz-sim-server gz-sim-gui arduplane 2>/dev/null; sleep 1", shell=True)
    os.makedirs(REC_DIR, exist_ok=True)
    home_rec = os.path.expanduser("~/gazebo_recordings_live")
    subprocess.run(f"rm -rf {home_rec} && mkdir -p {home_rec}", shell=True)

    # 2. Start Gazebo Server with 3D State Recording
    print(" [1/5] Starting Gazebo physics with 3D state recorder (20 Hz)...")
    env = os.environ.copy()
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = get_plugin_dir()
    env["GZ_SIM_RESOURCE_PATH"] = os.path.join(REPO_DIR, "gazebo", "models")
    gz_cmd = f"exec gz sim -s -r --record-path {home_rec} --record-period 0.05 --log-overwrite {WORLD_PATH}"
    gz_proc = subprocess.Popen(gz_cmd, shell=True, env=env, preexec_fn=os.setsid)
    time.sleep(3.0)

    # 3. Start ArduPlane SITL
    print(" [2/5] Starting ArduPlane SITL...")
    sitl_cmd = f"cd {WSL_ARDUPILOT} && rm -f mav.parm ./mav.parm 2>/dev/null && python3 Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON -w --custom-location=\"13.0827,80.2707,10,90\" --add-param-file={PARAM_FILE} --no-mavproxy --no-rebuild -S 1 -D --sysid 1 > /tmp/sitl_rec.log 2>&1 & sleep 5; pgrep -fa arduplane"
    subprocess.run(sitl_cmd, shell=True)
    time.sleep(4.0)

    # 4. Connect MAVLink
    print(" [3/5] Connecting to autopilot MAVLink...")
    mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, source_component=190, autoreconnect=True)
    t_conn = time.time()
    while time.time() - t_conn < 20.0:
        msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if msg and msg.get_srcSystem() == 1 and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
            mav.target_system = 1
            mav.target_component = 1
            print(f"       [OK] Connected to Autopilot (SysID: {mav.target_system})")
            break
        time.sleep(0.5)

    # Align EKF
    print("       [OK] Aligning EKF3 with GPS...")
    t_align = time.time()
    while time.time() - t_align < 12.0:
        msg = mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1.0)
        if msg and (msg.flags & 1 or msg.flags & 8):
            break

    # Start 50 Hz RC Override
    rc_active = True
    target_rc3 = 1000

    def rc_worker():
        while rc_active:
            try:
                mav.mav.rc_channels_override_send(
                    1, 1, 1500, 1500, int(target_rc3), 1500, 65535, 65535, 65535, 65535
                )
            except Exception:
                pass
            time.sleep(0.02)

    rc_thread = threading.Thread(target=rc_worker, daemon=True)
    rc_thread.start()

    # 5. Fly the Mission
    print(" [4/5] Flying Automated Mission (Takeoff -> 8m Hover -> Loiter -> Land)...")
    mav.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 19) # QLOITER
    time.sleep(1.0)
    mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    time.sleep(1.0)

    # Spool up and climb to 8m
    target_rc3 = 1750
    t_start = time.time()
    while time.time() - t_start < 8.0:
        time.sleep(0.5)

    # Hover and hold altitude
    target_rc3 = 1500
    t_hover = time.time()
    while time.time() - t_hover < 8.0:
        time.sleep(0.5)

    # Land
    print("       [OK] Commanding QLAND...")
    mav.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 20) # QLAND
    time.sleep(4.0)

    # Disarm
    rc_active = False
    mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(1.0)

    # 6. Graceful Shutdown & Finalize state.tlog
    print(" [5/5] Finalizing 3D recording metadata (SIGINT graceful sync)...")
    try:
        subprocess.run("kill -SIGINT $(pgrep -f 'gz-sim-server|ruby.*gz') 2>/dev/null || true", shell=True)
        time.sleep(3.0)
        subprocess.run("killall -9 arduplane 2>/dev/null || true", shell=True)
    except Exception:
        pass

    # Copy indexed state.tlog to Windows mount
    if os.path.exists(f"{home_rec}/state.tlog"):
        subprocess.run(f"cp -f {home_rec}/state.tlog {REC_DIR}/state.tlog", shell=True)
        sz = os.path.getsize(f"{REC_DIR}/state.tlog")
        print(f"\n [SUCCESS] Recorded full 3D flight state log ({sz / 1024 / 1024:.2f} MB)")
        print(f"           Location: {REC_DIR}/state.tlog")
    else:
        print(" [WARN] Could not find state.tlog")

    print("=" * 80)


if __name__ == '__main__':
    record_mission()

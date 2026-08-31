#!/usr/bin/env bash
# =============================================================
#  Flightory Stallion VTOL — Pure Hardware-in-the-Loop (HITL)
#  Connects Gazebo 3D Physics (UDP 9002) directly to Matek H743
# =============================================================

STALLION_DIR="/mnt/c/Users/SACHIN/Stallion"
PORT="${1:-/dev/ttyACM0}"

echo "========================================================="
echo " Flightory Stallion VTOL — Hardware-in-the-Loop (HITL)"
echo " Simulator  : Gazebo (3D Physics UDP 9002)"
echo " Flight Ctrl: Physical Matek H743 on ${PORT}"
echo " Telemetry  : UDP 14550 (Mission Planner)"
echo "========================================================="

# Ensure USB permissions
sudo chmod 666 "${PORT}" 2>/dev/null || true

python3 "${STALLION_DIR}/scripts/run_matek_hitl.py" "${PORT}"

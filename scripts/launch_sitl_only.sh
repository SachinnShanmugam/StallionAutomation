#!/usr/bin/env bash
# =================================================================
#  launch_sitl_only.sh — TERMINAL 2
#  Starts ArduPlane SITL connected to Gazebo via JSON FDM
#  Run this AFTER Gazebo is already running (launch_gazebo_only.sh)
# =================================================================
set -e

STALLION_DIR="/mnt/c/Users/SACHIN/Stallion"
ARDUPILOT_DIR="${HOME}/ardupilot"
PARAM_FILE="${STALLION_DIR}/params/stallion_vtol_sitl.parm"

# ─── Verify ArduPilot source exists ────────────────────────────
if [ ! -d "${ARDUPILOT_DIR}" ]; then
    echo "!!! ERROR: ArduPilot source not found at ${ARDUPILOT_DIR}"
    echo "    Clone with: git clone https://github.com/ArduPilot/ardupilot.git"
    exit 1
fi

if [ ! -f "${PARAM_FILE}" ]; then
    echo "!!! ERROR: Parameter file not found at ${PARAM_FILE}"
    exit 1
fi

# Delete stale param cache to force fresh load
rm -f "${ARDUPILOT_DIR}/mav.parm" ./mav.parm 2>/dev/null

# Windows host IP for Mission Planner UDP connection
WINDOWS_HOST=$(ip route show default | awk '/default/ {print $3}' | head -1)

echo ""
echo "============================================="
echo " TERMINAL 2: ArduPlane SITL (JSON → Gazebo)"
echo "============================================="
echo " Location  : Chennai (13.0827 N, 80.2707 E)"
echo " Frame     : JSON (physics from Gazebo)"
echo " Params    : ${PARAM_FILE}"
echo " UDP out   : ${WINDOWS_HOST}:14550 (Mission Planner)"
echo "============================================="
echo ""
echo ">>> IMPORTANT: Make sure Gazebo is already running!"
echo ">>> Connect Mission Planner to UDP ${WINDOWS_HOST}:14550"
echo ""
echo ">>> After MAVProxy starts, test hover with:"
echo ">>>   mode QHOVER"
echo ">>>   arm throttle"
echo ">>>   rc 3 1600"
echo ""

cd "${ARDUPILOT_DIR}"

# -f JSON tells SITL to use Gazebo's JSON FDM physics backend
# -w wipes eeprom (add -w flag below on FIRST RUN to force param reload)
python3 Tools/autotest/sim_vehicle.py \
    -v ArduPlane \
    -f JSON \
    -w \
    --custom-location="13.0827,80.2707,10,90" \
    --add-param-file="${PARAM_FILE}" \
    --out=udpout:${WINDOWS_HOST}:14550 \
    --out=udpout:127.0.0.1:14550 \
    --console \
    --map

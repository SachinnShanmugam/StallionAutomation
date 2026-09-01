#!/usr/bin/env bash
# =================================================================
#  launch_sitl_drone2.sh — TERMINAL 3
#  Starts ArduPlane SITL for Drone 2 (SYSID=2, Follower)
#  --instance 1  → uses ports 5770 (TCP), 5771 (UDP)
#
#  Run this AFTER Gazebo is already running.
#  Run in a separate terminal from Drone 1 (launch_sitl_only.sh).
# =================================================================

STALLION_DIR="/mnt/c/Users/SACHIN/Stallion"
ARDUPILOT_DIR="${HOME}/ardupilot"
PARAM_FILE="${STALLION_DIR}/params/stallion_drone2_sitl.parm"

if [ ! -d "${ARDUPILOT_DIR}" ]; then
    echo "!!! ERROR: ArduPilot source not found at ${ARDUPILOT_DIR}"
    exit 1
fi

if [ ! -f "${PARAM_FILE}" ]; then
    echo "!!! ERROR: Drone 2 parameter file not found at ${PARAM_FILE}"
    exit 1
fi

# Offset home position by ~2.5 metres east so Drone 2 starts beside Drone 1
HOME_LAT=13.0827
HOME_LON=80.27072    # 2.5m east of runway center line

echo ""
echo "============================================="
echo " TERMINAL 3: Drone 2 SITL (SYSID=2, Follower)"
echo "============================================="
echo " Instance  : 1  (ports 5770/5771)"
echo " Location  : ${HOME_LAT}, ${HOME_LON}  (10m east of Drone 1)"
echo " Params    : ${PARAM_FILE}"
echo "============================================="
echo ""

cd "${ARDUPILOT_DIR}"

# Windows host IP for Mission Planner connection (Drone 2 uses port 14560)
WINDOWS_HOST=$(ip route show default | awk '/default/ {print $3}' | head -1)

echo ">>> Connect Mission Planner (2nd window) → UDP ${WINDOWS_HOST}:14560"
echo ""

python3 Tools/autotest/sim_vehicle.py \
    -v ArduPlane \
    -f JSON \
    -w \
    -I 1 \
    --custom-location="${HOME_LAT},${HOME_LON},10,90" \
    --add-param-file="${PARAM_FILE}" \
    --sysid 2 \
    --out=udpout:${WINDOWS_HOST}:14560 \
    --out=udpout:127.0.0.1:14560 \
    --console \
    --map

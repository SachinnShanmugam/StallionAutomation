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

# Offset home position by ~10 metres east so Drone 2 starts beside Drone 1
# Chennai base: 13.0827, 80.2707  →  +0.0001° lon ≈ +10m
HOME_LAT=13.0827
HOME_LON=80.2708    # 10m east of Drone 1

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

python3 Tools/autotest/sim_vehicle.py \
    -v ArduPlane \
    -f JSON \
    -w \
    -I 1 \
    --custom-location="${HOME_LAT},${HOME_LON},10,90" \
    --add-param-file="${PARAM_FILE}" \
    --no-mavproxy \
    --no-rebuild \
    --sysid 2

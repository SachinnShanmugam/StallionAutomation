#!/usr/bin/env bash
# =============================================================
#  start_sitl_stallion.sh
#  Launch Flightory Stallion VTOL (Tilt-Tricopter) SITL in Ubuntu
#  Location: Chennai (Anna Nagar / Koyambedu)
#  Firmware: ArduPlane (Matek H743-Wing configuration)
# =============================================================

ARDUPILOT_DIR="${HOME}/ardupilot"
PARAM_FILE="/mnt/c/Users/SACHIN/Stallion/params/stallion_vtol_sitl.parm"
WAYPOINT_FILE="/mnt/c/Users/SACHIN/Stallion/missions/chennai_loop_01.waypoints"

HOME_LAT="13.0827"
HOME_LON="80.2707"
HOME_ALT="10"
HOME_HDG="90"

echo "========================================================"
echo " Flightory Stallion VTOL SITL — Chennai Autonomous Mission"
echo " Airframe : Tilt-Tricopter (2 Front Tilt + 1 Rear Fixed)"
echo " Target FC: Matek H743-Wing (ArduPlane)"
echo " Home     : Chennai (${HOME_LAT} N, ${HOME_LON} E)"
echo " UDP Out  : udpout:127.0.0.1:14550 (Mission Planner)"
echo "========================================================"

cd "${ARDUPILOT_DIR}"

python3 Tools/autotest/sim_vehicle.py \
    -v ArduPlane \
    -f quadplane-tilttri \
    --custom-location="${HOME_LAT},${HOME_LON},${HOME_ALT},${HOME_HDG}" \
    --add-param-file="${PARAM_FILE}" \
    --out=udpout:127.0.0.1:14550 \
    --console \
    --map

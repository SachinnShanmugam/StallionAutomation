#!/usr/bin/env bash
# =============================================================
# Flightory Stallion VTOL - SITL Launch Script
# Frame:        quadplane-tilttri (Tilt Tricopter)
# Firmware:     ArduPlane (sim_vehicle.py)
# Home:         Chennai, Tamil Nadu, India
#               Lat: 13.0827  Lon: 80.2707  Alt: 6m (AMSL)
# =============================================================
#
# PREREQUISITES:
#   1. ArduPilot source cloned:
#      git clone https://github.com/ArduPilot/ardupilot.git
#      cd ardupilot && git submodule update --init --recursive
#   2. Python deps:
#      Tools/environment_install/install-prereqs-ubuntu.sh -y
#   3. (Windows) Use WSL2 / Ubuntu 20.04+
#
# USAGE:
#   chmod +x launch_sitl.sh
#   ./launch_sitl.sh
# =============================================================

# --- Configuration ---
ARDUPILOT_DIR="${HOME}/ardupilot"   # Change if your clone is elsewhere
PARAM_FILE="$(dirname "$0")/params/stallion_vtol_sitl.parm"
SITL_SPEEDUP=1

# Chennai coordinates (Anna Nagar / Koyambedu area)
HOME_LAT=13.0827
HOME_LON=80.2707
HOME_ALT=6

echo "========================================================"
echo " Flightory Stallion VTOL - SITL"
echo " Firmware : ArduPlane (stable, MatekH743 target)"
echo " Frame    : quadplane-tilttri"
echo " Home     : Chennai, India ($HOME_LAT, $HOME_LON)"
echo "========================================================"
echo ""

# Verify ArduPilot directory
if [ ! -d "$ARDUPILOT_DIR" ]; then
    echo "[ERROR] ArduPilot source not found at: $ARDUPILOT_DIR"
    echo "  Clone it with: git clone https://github.com/ArduPilot/ardupilot.git"
    exit 1
fi

cd "$ARDUPILOT_DIR"

# Launch sim_vehicle with:
#   -v ArduPlane       -> ArduPlane vehicle
#   -f quadplane-tilttri -> Tilt-tricopter physics model (matches Stallion 2-front-tilt + 1-rear)
#   --home             -> Set home to Chennai
#   --add-param-file   -> Load our Stallion params
#   --console          -> MAVProxy text console
#   --map              -> OpenStreetMap ground-control view
#   -w (FIRST RUN ONLY, uncomment to wipe params) 
#
# NOTE: Remove -w after first launch to preserve tuned parameters.

Tools/autotest/sim_vehicle.py \
    -v ArduPlane \
    -f quadplane-tilttri \
    --home="${HOME_LAT},${HOME_LON},${HOME_ALT},90" \
    --add-param-file="${PARAM_FILE}" \
    --console \
    --map \
    --speedup=${SITL_SPEEDUP}

# =============================================================
# AFTER LAUNCH - MAVProxy Mission Commands
# =============================================================
# Once MAVProxy is running, paste these commands in the console:
#
# 1. Load Chennai waypoints:
#    wp load /path/to/Stallion/missions/chennai_loop_01.waypoints
#
# 2. Arm and set AUTO mode:
#    mode QHOVER
#    arm throttle
#    mode AUTO
#
# 3. Monitor in map window or with:
#    status
#    wp list
# =============================================================

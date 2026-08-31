#!/usr/bin/env bash
# =============================================================
#  start_gazebo_stallion.sh — All-in-One Launcher
#  Starts Gazebo in background, then ArduPlane SITL in foreground
#  Use this for a single-terminal experience
# =============================================================
set -e

STALLION_DIR="/mnt/c/Users/SACHIN/Stallion"
ARDUPILOT_DIR="${HOME}/ardupilot"
ARDUPILOT_GZ_BUILD="${HOME}/ardupilot_gazebo/build"

echo "========================================================="
echo " Flightory Stallion VTOL — Gazebo + SITL Launcher"
echo "========================================================="

# ─── Kill stale processes ─────────────────────────────────────
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f arduplane 2>/dev/null || true
pkill -9 -f mavproxy 2>/dev/null || true
sleep 1

# Delete stale parameter cache
rm -f "${ARDUPILOT_DIR}/mav.parm" ./mav.parm 2>/dev/null
echo ">>> Cleared stale processes and param cache"

# ─── Verify prerequisites ────────────────────────────────────
if [ ! -f "${ARDUPILOT_GZ_BUILD}/libArduPilotPlugin.so" ]; then
    echo "!!! ERROR: libArduPilotPlugin.so not found"
    echo "    Build: cd ~/ardupilot_gazebo && mkdir -p build && cd build && cmake .. && make -j\$(nproc)"
    exit 1
fi

if [ ! -d "${ARDUPILOT_DIR}" ]; then
    echo "!!! ERROR: ArduPilot not found at ${ARDUPILOT_DIR}"
    exit 1
fi

# ─── Gazebo Environment ──────────────────────────────────────
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ARDUPILOT_GZ_BUILD}:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export LD_LIBRARY_PATH="${ARDUPILOT_GZ_BUILD}:${LD_LIBRARY_PATH}"
export GZ_SIM_RESOURCE_PATH="${STALLION_DIR}/gazebo/models:${STALLION_DIR}/gazebo/worlds:${HOME}/ardupilot_gazebo/models:${GZ_SIM_RESOURCE_PATH}"

# ─── WSLg Display ────────────────────────────────────────────
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
export PULSE_SERVER=unix:/mnt/wslg/PulseServer
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=4.5
export MESA_GLSL_VERSION_OVERRIDE=450

WORLD_FILE="${STALLION_DIR}/gazebo/worlds/stallion_runway.sdf"
PARAM_FILE="${STALLION_DIR}/params/stallion_vtol_sitl.parm"
WINDOWS_HOST=$(ip route show default | awk '/default/ {print $3}' | head -1)

# ─── Step 1: Start Gazebo ────────────────────────────────────
echo ""
echo ">>> Step 1: Starting Gazebo 3D window..."
gz sim -v 3 -r "${WORLD_FILE}" &
GZ_PID=$!
echo "    Gazebo PID = $GZ_PID"

echo ">>> Waiting 8 seconds for Gazebo to initialize..."
sleep 8

if ! kill -0 $GZ_PID 2>/dev/null; then
    echo "!!! ERROR: Gazebo failed to start."
    echo "    Check: is WSLg working? Try running 'glxinfo' first."
    exit 1
fi
echo ">>> Gazebo running OK (PID $GZ_PID)"

# ─── Step 2: Start ArduPlane SITL ────────────────────────────
echo ""
echo ">>> Step 2: Starting ArduPlane SITL (JSON → Gazebo)..."
echo ">>> Mission Planner UDP: ${WINDOWS_HOST}:14550"
echo ""

cd "${ARDUPILOT_DIR}"

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

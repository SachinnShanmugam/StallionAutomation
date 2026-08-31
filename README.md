# Flightory Stallion VTOL — Gazebo + ArduPilot SITL Simulation

## Quick Start (2-Terminal Method — Recommended)

### Terminal 1: Start Gazebo
```bash
cd ~/
bash /mnt/c/Users/SACHIN/Stallion/scripts/launch_gazebo_only.sh
```
Wait until the Gazebo 3D window appears with the Stallion model on the ground.

### Terminal 2: Start SITL
```bash
cd ~/
bash /mnt/c/Users/SACHIN/Stallion/scripts/launch_sitl_only.sh
```
Wait for MAVProxy to connect. You should see `GPS fix` and heartbeat messages.

### Fly!
In the MAVProxy console (Terminal 2):
```
mode QHOVER
arm throttle
rc 3 1600
```
The drone should lift off and hover in the Gazebo window.

To land:
```
rc 3 1000
mode QLAND
```

---

## Quick Start (Single-Terminal Method)
```bash
bash /mnt/c/Users/SACHIN/Stallion/scripts/start_gazebo_stallion.sh
```

---

## Connect Mission Planner (Windows)
1. Open Mission Planner on Windows
2. Select **UDP** connection type
3. Click **Connect**
4. Enter the UDP port: **14550**
5. Done — you should see telemetry from the Stallion VTOL

---

## Prerequisites

### 1. ArduPilot Source
```bash
cd ~/
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

### 2. Gazebo Harmonic
```bash
sudo apt install gz-harmonic
```

### 3. ArduPilot Gazebo Plugin
```bash
cd ~/
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

Verify: `ls ~/ardupilot_gazebo/build/libArduPilotPlugin.so` should exist.

---

## Autonomous Mission
After SITL is running:
```
wp load /mnt/c/Users/SACHIN/Stallion/missions/chennai_loop_01.waypoints
mode QHOVER
arm throttle
mode AUTO
```

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Gazebo window doesn't open | Run `echo $DISPLAY` — should show `:0`. Install WSLg. |
| "libArduPilotPlugin.so not found" | Build ardupilot_gazebo (see Prerequisites step 3) |
| SITL says "waiting for JSON" | Gazebo isn't running or world didn't load correctly |
| Drone falls through ground | Verify model.sdf has `<collision>` on base_link |
| Drone doesn't fly / zero thrust | Verify LiftDrag plugins have `<joint_name>` tags |
| Mission Planner won't connect | Check Windows firewall allows UDP port 14550 |

---

## File Structure
```
Stallion/
├── gazebo/
│   ├── models/stallion_vtol/
│   │   ├── model.config          # Gazebo model metadata
│   │   ├── model.sdf             # Full drone physics model
│   │   └── meshes/               # STL mesh files (from CAD)
│   └── worlds/
│       └── stallion_runway.sdf   # World with ground plane + Chennai coords
├── scripts/
│   ├── launch_gazebo_only.sh     # Terminal 1: Gazebo only
│   ├── launch_sitl_only.sh       # Terminal 2: SITL only (connects to Gazebo)
│   └── start_gazebo_stallion.sh  # All-in-one launcher
├── params/
│   └── stallion_vtol_sitl.parm   # ArduPlane parameters for Stallion VTOL
└── missions/
    └── chennai_loop_01.waypoints # Test mission around Chennai
```

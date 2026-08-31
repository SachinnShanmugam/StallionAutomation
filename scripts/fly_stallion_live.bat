@echo off
REM =========================================================================
REM Stallion VTOL - Live 3D Gazebo Flight Demonstration (Windows 1-Click)
REM =========================================================================
echo =========================================================================
echo  Launching Live Gazebo 3D GUI and Executing Flight Takeoff...
echo =========================================================================

wsl -d Ubuntu-22.04 bash -c "export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/drones/ardupilot_gazebo/build; export GZ_SIM_RESOURCE_PATH=/mnt/c/Users/SACHIN/Stallion/gazebo/models; export MESA_GL_VERSION_OVERRIDE=4.5; python3 /mnt/c/Users/SACHIN/Stallion/scripts/fly_stallion_live.py"

pause

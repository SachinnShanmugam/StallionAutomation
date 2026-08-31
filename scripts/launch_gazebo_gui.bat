@echo off
REM =========================================================================
REM Stallion VTOL - Launch Gazebo 3D GUI (Windows 1-Click)
REM =========================================================================
echo =========================================================================
echo  Launching Full 3D Gazebo GUI with Stallion VTOL on Runway...
echo =========================================================================

wsl -d Ubuntu-22.04 bash -c "export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/drones/ardupilot_gazebo/build; export GZ_SIM_RESOURCE_PATH=/mnt/c/Users/SACHIN/Stallion/gazebo/models; gz sim -r /mnt/c/Users/SACHIN/Stallion/gazebo/worlds/stallion_runway.sdf"

pause

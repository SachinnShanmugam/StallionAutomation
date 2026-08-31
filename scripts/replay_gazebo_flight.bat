@echo off
REM =========================================================================
REM Stallion VTOL - Native Gazebo 3D Flight Replay Launcher (Windows)
REM =========================================================================
echo =========================================================================
echo  Launching Native Gazebo 3D Flight Replay GUI...
echo =========================================================================

wsl -d Ubuntu-22.04 -u root bash -c "mkdir -p /home/runner/work/StallionAutomation && ln -sfn /mnt/c/Users/SACHIN/Stallion /home/runner/work/StallionAutomation/StallionAutomation && chmod -R 777 /home/runner"
wsl -d Ubuntu-22.04 bash -c "export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/drones/ardupilot_gazebo/build; export GZ_SIM_RESOURCE_PATH=/mnt/c/Users/SACHIN/Stallion/gazebo/models:/home/runner/work/StallionAutomation/StallionAutomation/gazebo/models; export MESA_GL_VERSION_OVERRIDE=4.5; gz sim -r --playback /mnt/c/Users/SACHIN/Stallion/logs/gazebo_recordings"

pause

@echo off
REM =========================================================================
REM Stallion VTOL - Live Gazebo to DroneCAN Serialization Runner (Windows)
REM =========================================================================
echo =========================================================================
echo  Launching Live Gazebo to DroneCAN GPS Real-Time Serialization Suite...
echo =========================================================================

cd /d "%~dp0\.."
python "%~dp0test_live_gazebo_dronecan.py" 15

pause

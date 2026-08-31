@echo off
REM =========================================================================
REM Stallion VTOL - Simulated DroneCAN GPS Node Launcher (Windows)
REM =========================================================================
echo =========================================================================
echo  Launching Simulated DroneCAN GPS Node (uavcan.equipment.gnss.Fix2)...
echo =========================================================================

wsl -d Ubuntu-22.04 bash -c "python3 /mnt/c/Users/SACHIN/Stallion/scripts/dronecan_gps_node.py fixed 5"

pause

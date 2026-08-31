@echo off
REM =========================================================================
REM Stallion VTOL - Live QLOITER Mission & CAN Bus Telemetry Stream Launcher
REM =========================================================================
echo =========================================================================
echo  Launching Live QLOITER Takeoff with Real-Time CAN Bus Streaming...
echo =========================================================================

wsl -d Ubuntu-22.04 python3 /mnt/c/Users/SACHIN/Stallion/scripts/test_live_can_flight.py

pause

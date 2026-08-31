@echo off
REM =========================================================================
REM Stallion VTOL - Live Parallel Telemetry & DroneCAN HUD Launcher
REM =========================================================================
echo =========================================================================
echo  Launching Live Flight Telemetry and DroneCAN HUD...
echo  (Keep this open while flying SITL in your other terminal!)
echo =========================================================================

cd /d "%~dp0\.."
python "%~dp0live_flight_telemetry_hud.py"

pause

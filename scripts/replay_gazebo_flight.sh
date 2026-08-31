#!/usr/bin/env bash
# =========================================================================
# Stallion VTOL - Native Gazebo 3D Flight Replay Launcher (Linux / WSL)
# =========================================================================
echo "========================================================================="
echo " Launching Native Gazebo 3D Flight Replay GUI..."
echo "========================================================================="

export GZ_SIM_RESOURCE_PATH=/mnt/c/Users/SACHIN/Stallion/gazebo/models
gz sim --playback /mnt/c/Users/SACHIN/Stallion/logs/gazebo_recordings

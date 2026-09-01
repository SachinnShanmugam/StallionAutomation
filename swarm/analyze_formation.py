#!/usr/bin/env python3
"""
Swarm V1 Formation Tracking Accuracy Analyzer.

Reads logs/formation_metrics.csv and outputs objective statistical proof:
- Mean / Min / Max / Std Dev Formation Distance
- Tracking Error (RMSE, Max)
- Altitude Matching Error
- Settling Time
"""

import os
import sys
import math
import numpy as np

CSV_PATH = "logs/formation_metrics.csv"

def analyze():
    if not os.path.exists(CSV_PATH):
        print(f"[ERROR] Telemetry log file not found: {CSV_PATH}")
        print("[INFO] Run a formation flight session first to generate logs.")
        return

    data = []
    with open(CSV_PATH, "r") as f:
        lines = f.readlines()

    if len(lines) < 2:
        print("[WARN] Log file is empty or contains no data points.")
        return

    header = lines[0].strip().split(",")
    for line in lines[1:]:
        parts = line.strip().split(",")
        if len(parts) == len(header):
            try:
                data.append([float(x) for x in parts])
            except ValueError:
                pass

    if not data:
        print("[WARN] No valid numerical data parsed from log.")
        return

    data = np.array(data)

    # Columns:
    # 0: time, 1: leader_lat, 2: leader_lon, 3: leader_alt, 4: leader_hdg, 5: leader_vx, 6: leader_vy,
    # 7: follower_lat, 8: follower_lon, 9: follower_alt, 10: follower_hdg,
    # 11: target_lat, 12: target_lon, 13: target_alt, 14: actual_dist, 15: desired_dist, 16: dist_error, 17: alt_error

    timestamps   = data[:, 0] - data[0, 0]
    actual_dist  = data[:, 14]
    desired_dist = data[:, 15]
    dist_error   = data[:, 16]
    alt_error    = data[:, 17]

    # Filter out initial launch transient (first 10 seconds while climbing to formation height)
    steady_idx = np.where(timestamps >= 10.0)[0]
    if len(steady_idx) == 0:
        steady_idx = np.arange(len(timestamps))

    st_dist   = actual_dist[steady_idx]
    st_err    = dist_error[steady_idx]
    st_alt    = alt_error[steady_idx]

    mean_dist = np.mean(st_dist)
    std_dist  = np.std(st_dist)
    min_dist  = np.min(st_dist)
    max_dist  = np.max(st_dist)

    mean_err  = np.mean(np.abs(st_err))
    rmse_err  = np.sqrt(np.mean(st_err ** 2))
    max_err   = np.max(np.abs(st_err))

    mean_alt_err = np.mean(np.abs(st_alt))
    max_alt_err  = np.max(np.abs(st_alt))
    rmse_alt     = np.sqrt(np.mean(st_alt ** 2))

    # Calculate Settling Time (time until dist_error stays within +- 2.0m of 20m)
    settling_time = "N/A"
    for i in range(len(timestamps)):
        if np.all(np.abs(dist_error[i:]) <= 2.5):
            settling_time = f"{timestamps[i]:.1f} s"
            break

    print("\n" + "=" * 60)
    print("      SWARM V1 FORMATION TRACKING EMPIRICAL REPORT")
    print("=" * 60)
    print(f"Total Flight Samples Recorded : {len(data)} samples ({timestamps[-1]:.1f} seconds)")
    print(f"Steady-State Samples Evaluated: {len(steady_idx)} samples")
    print("-" * 60)
    print("1. FORMATION DISTANCE (Target: 20.0 m rear offset)")
    print(f"   • Mean Distance  : {mean_dist:.2f} m")
    print(f"   • Std Deviation  : ±{std_dist:.2f} m")
    print(f"   • Min / Max Dist : {min_dist:.2f} m / {max_dist:.2f} m")
    print("-" * 60)
    print("2. POSITION TRACKING ERROR (|Actual - 20.0m|)")
    print(f"   • Mean Error     : {mean_err:.2f} m")
    print(f"   • RMSE Error     : {rmse_err:.2f} m")
    print(f"   • Max Peak Error : {max_err:.2f} m")
    print("-" * 60)
    print("3. ALTITUDE TRACKING ERROR (|Follower Alt - Leader Alt|)")
    print(f"   • Mean Alt Error : {mean_alt_err:.2f} m")
    print(f"   • RMSE Alt Error : {rmse_alt:.2f} m")
    print(f"   • Max Peak Error : {max_alt_err:.2f} m")
    print("-" * 60)
    print(f"4. FORMATION SETTLING TIME : {settling_time}")
    print("=" * 60 + "\n")

if __name__ == "__main__":
    analyze()

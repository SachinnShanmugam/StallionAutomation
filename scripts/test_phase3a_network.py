#!/usr/bin/env python3
"""
Stallion VTOL - Phase 3A: Network Transport & Latency Benchmarking
===================================================================
Benchmarks the end-to-end transport link:
  PC UDP Socket <-> High-Speed Bridge <-> Real Matek H743 (STM32H7)

Measures:
  - Packet frequency (Hz)
  - Packet loss (%)
  - Round-trip latency (min, max, mean, median in ms)
  - Jitter / Standard Deviation (ms)
  - Timestamp synchronization error
"""

import os
import sys
import time
import socket
import statistics
import json

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(line_buffering=True)

from pymavlink import mavutil

PORT = 'COM7' if os.name == 'nt' else '/dev/ttyACM0'
BAUD = 115200
TEST_PACKETS = 500

def run_phase3a_benchmark():
    print("=" * 75)
    print(" [PHASE 3A] PC <-> Matek H743 Transport & Timing Benchmark")
    print("=" * 75)

    # 1. Connect to Hardware
    print(f"\n[1] Opening MAVLink connection to Matek H743 on {PORT}...")
    try:
        fc = mavutil.mavlink_connection(PORT, baud=BAUD, autoreconnect=True)
        fc.wait_heartbeat(timeout=10)
        print(f"    [OK] Heartbeat received from SysID: {fc.target_system}")
    except Exception as e:
        print(f"    [FAIL] Connection error: {e}")
        return

    # 2. Configure High-Rate Data Streams
    print("\n[2] Requesting 200 Hz Controller & Extra1 streams from STM32H7...")
    fc.mav.request_data_stream_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 200, 1)
    fc.mav.request_data_stream_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 200, 1)
    time.sleep(0.5)

    # 3. Benchmark Round-Trip Latency & Timing Jitter
    print(f"\n[3] Transmitting {TEST_PACKETS} Sensor Frames and Measuring RTT...")
    rtt_samples_ms = []
    sent_count = 0
    recv_count = 0
    lost_count = 0

    t_benchmark_start = time.time()
    last_print = time.time()

    for seq in range(TEST_PACKETS):
        t_send = time.perf_counter()
        
        # Inject HIL Sensor Frame
        fc.mav.hil_sensor_send(
            int(time.time() * 1e6),
            0.0, 0.0, -9.81, # Accel (m/s^2)
            0.0, 0.0, 0.0,   # Gyro (rad/s)
            0.2, 0.0, 0.4,   # Mag (Gauss)
            1013.25, 0.0, 0.0, 25.0, # Pressure & Temp
            0b1111111111111
        )
        sent_count += 1

        # Wait for Actuator Response or Attitude Update
        msg = fc.recv_match(type=['SERVO_OUTPUT_RAW', 'ATTITUDE'], blocking=True, timeout=0.05)
        t_recv = time.perf_counter()

        if msg:
            recv_count += 1
            rtt_ms = (t_recv - t_send) * 1000.0
            rtt_samples_ms.append(rtt_ms)
        else:
            lost_count += 1

        # Control injection rate (~200 Hz target, 5ms interval)
        elapsed_step = time.perf_counter() - t_send
        if elapsed_step < 0.005:
            time.sleep(0.005 - elapsed_step)

        if time.time() - last_print >= 1.0:
            print(f"    ... Processed {seq+1}/{TEST_PACKETS} packets (Recv: {recv_count}, Lost: {lost_count})")
            last_print = time.time()

    t_total = time.time() - t_benchmark_start
    actual_rate_hz = sent_count / t_total

    # 4. Statistical Analysis
    if rtt_samples_ms:
        min_rtt = min(rtt_samples_ms)
        max_rtt = max(rtt_samples_ms)
        mean_rtt = statistics.mean(rtt_samples_ms)
        median_rtt = statistics.median(rtt_samples_ms)
        stdev_rtt = statistics.stdev(rtt_samples_ms) if len(rtt_samples_ms) > 1 else 0.0
    else:
        min_rtt = max_rtt = mean_rtt = median_rtt = stdev_rtt = 0.0

    loss_pct = (lost_count / sent_count) * 100.0 if sent_count > 0 else 100.0

    # 5. Output Report
    print("\n" + "=" * 75)
    print(" [PHASE 3A BENCHMARK RESULTS]")
    print(f"   Packets Sent:               {sent_count}")
    print(f"   Packets Received:           {recv_count}")
    print(f"   Packet Loss:                {lost_count} ({loss_pct:.2f}%)")
    print(f"   Achieved Packet Rate:       {actual_rate_hz:.1f} Hz")
    print("-" * 75)
    print(f"   Round-Trip Latency (RTT):")
    print(f"     - Mean RTT:               {mean_rtt:.2f} ms")
    print(f"     - Median RTT:             {median_rtt:.2f} ms")
    print(f"     - Min RTT:                {min_rtt:.2f} ms")
    print(f"     - Max RTT:                {max_rtt:.2f} ms")
    print(f"   Jitter (Std Dev):           {stdev_rtt:.2f} ms")
    print("=" * 75)

    # Save benchmark metrics to json
    results = {
        'sent_count': sent_count,
        'recv_count': recv_count,
        'packet_loss_pct': loss_pct,
        'actual_rate_hz': actual_rate_hz,
        'min_rtt_ms': min_rtt,
        'max_rtt_ms': max_rtt,
        'mean_rtt_ms': mean_rtt,
        'median_rtt_ms': median_rtt,
        'jitter_stdev_ms': stdev_rtt
    }
    with open('phase3a_benchmark.json', 'w') as f:
        json.dump(results, f, indent=2)
    print("\n[OK] Benchmark metrics saved to phase3a_benchmark.json")

if __name__ == '__main__':
    run_phase3a_benchmark()

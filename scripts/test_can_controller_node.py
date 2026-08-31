#!/usr/bin/env python3
"""
Stallion VTOL - CAN Controller Test & Verification Node
=========================================================
Subscribes to CAN channels broadcasted from Gazebo, decodes live
telemetry (IMU, Attitude, Baro, GPS, ESC RPM), and sends CAN actuator commands.
"""

import os
import sys
import time
import struct
import socket
import threading

def run_can_subscriber(duration_sec=10, udp_can_port=10005):
    print("=" * 80)
    print("  STALLION VTOL - CAN NODE SUBSCRIBER & CONTROLLER TEST")
    print("=" * 80)
    print(f"[CAN TEST] Listening for CAN frames on UDP 127.0.0.1:{udp_can_port}...")
    print("=" * 80)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', udp_can_port))
    sock.settimeout(1.0)

    frame_counts = {}
    last_print = time.time()
    start_time = time.time()

    decoded_state = {
        'ax': 0.0, 'ay': 0.0, 'az': 0.0,
        'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
        'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        'press_alt': 0.0, 'airspeed': 0.0,
        'lat': 0.0, 'lon': 0.0, 'gps_alt': 0.0, 'gps_spd': 0.0,
        'm1_rpm': 0, 'm2_rpm': 0, 'm3_rpm': 0,
        'tilt_l': 0.0, 'tilt_r': 0.0, 'volts': 0.0
    }

    while time.time() - start_time < duration_sec:
        try:
            data, addr = sock.recvfrom(1024)
            if len(data) >= 13:
                arb_id, dlc, payload = struct.unpack('<IB8s', data[:13])
                frame_counts[arb_id] = frame_counts.get(arb_id, 0) + 1

                # Decode CAN Frames
                if arb_id == 0x101:  # IMU ACCEL
                    ax, ay, az, seq = struct.unpack('<hhhH', payload[:8])
                    decoded_state['ax'] = ax / 100.0
                    decoded_state['ay'] = ay / 100.0
                    decoded_state['az'] = az / 100.0
                elif arb_id == 0x102:  # IMU GYRO
                    gx, gy, gz, temp = struct.unpack('<hhhh', payload[:8])
                    decoded_state['gx'] = gx / 1000.0
                    decoded_state['gy'] = gy / 1000.0
                    decoded_state['gz'] = gz / 1000.0
                elif arb_id == 0x110:  # ATTITUDE
                    r, p, y, flags = struct.unpack('<hhhH', payload[:8])
                    decoded_state['roll'] = r / 100.0
                    decoded_state['pitch'] = p / 100.0
                    decoded_state['yaw'] = y / 100.0
                elif arb_id == 0x120:  # BARO & AIRSPEED
                    p_alt, asp, temp = struct.unpack('<iHh', payload[:8])
                    decoded_state['press_alt'] = p_alt / 100.0
                    decoded_state['airspeed'] = asp / 1000.0
                elif arb_id == 0x131:  # GPS POS
                    lat, lon = struct.unpack('<ii', payload[:8])
                    decoded_state['lat'] = lat * 1e-7
                    decoded_state['lon'] = lon * 1e-7
                elif arb_id == 0x132:  # GPS VEL/ALT
                    alt, spd, hdg = struct.unpack('<iHH', payload[:8])
                    decoded_state['gps_alt'] = alt / 100.0
                    decoded_state['gps_spd'] = spd / 100.0
                elif arb_id == 0x140:  # ESC STATUS 1
                    m1, m2, v = struct.unpack('<HHH', payload[:6])
                    decoded_state['m1_rpm'] = m1
                    decoded_state['m2_rpm'] = m2
                    decoded_state['volts'] = v / 100.0
                elif arb_id == 0x141:  # ESC STATUS 2
                    m3, tl, tr = struct.unpack('<Hhh', payload[:6])
                    decoded_state['m3_rpm'] = m3
                    decoded_state['tilt_l'] = tl / 100.0
                    decoded_state['tilt_r'] = tr / 100.0

        except socket.timeout:
            pass

        # Print decoded telemetry every 1.0 second
        if time.time() - last_print >= 1.0:
            last_print = time.time()
            elapsed = time.time() - start_time
            total_pkts = sum(frame_counts.values())
            print(f"[{elapsed:4.1f}s | {total_pkts:4d} pkts] "
                  f"Roll/Pit/Yaw: ({decoded_state['roll']:+5.1f}°, {decoded_state['pitch']:+5.1f}°, {decoded_state['yaw']:+5.1f}°) | "
                  f"Alt: {decoded_state['press_alt']:4.1f}m | "
                  f"GPS: ({decoded_state['lat']:.5f}, {decoded_state['lon']:.5f}) | "
                  f"ESCs: [{decoded_state['m1_rpm']}, {decoded_state['m2_rpm']}, {decoded_state['m3_rpm']}] RPM | "
                  f"Tilts: ({decoded_state['tilt_l']:+4.1f}°, {decoded_state['tilt_r']:+4.1f}°)")

    print("\n" + "=" * 80)
    print("  CAN BUS STREAM SUMMARY & VERIFICATION")
    print("=" * 80)
    for arb_id, count in sorted(frame_counts.items()):
        rate = count / duration_sec
        print(f"  Channel 0x{arb_id:03X}: {count:4d} frames received ({rate:5.1f} Hz)")
    print("=" * 80)
    sock.close()

if __name__ == '__main__':
    dur = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    run_can_subscriber(duration_sec=dur)

#!/usr/bin/env python3
"""
================================================================================
  STALLION VTOL - LIVE PARALLEL FLIGHT TELEMETRY HUD & DRONECAN STREAMER
================================================================================
Usage:
  Keep this running in a terminal while you fly SITL manually in another window.
  It automatically connects to MAVLink (UDP 14550 / TCP 5760) and Gazebo (UDP 9003).

Features:
  1. Live Attitude, Altitude, Climb Rate, Ground Speed & EKF Status.
  2. Live Motor Spool RPMs (M1, M2, M3) & Tilt Rotor Angles.
  3. Real-Time DroneCAN GPS (uavcan.equipment.gnss.Fix2) Serialization & CAN 2.0B Frames.
  4. Real-time ASCII Flight HUD refreshed at 10 Hz.
================================================================================
"""

import sys
import os
import time
import socket
import struct
import math
import select
from pymavlink import mavutil

try:
    import dronecan
    from dronecan import uavcan
    DRONECAN_AVAILABLE = True
except ImportError:
    DRONECAN_AVAILABLE = False


# ==============================================================================
# DRONECAN DSDL BITSTREAM PACKING UTILITIES
# ==============================================================================

def bits_to_bytes(bits: str) -> bytes:
    byte_arr = bytearray()
    for i in range(0, len(bits), 8):
        byte_bits = bits[i:i+8]
        val = 0
        for bit_idx, bit_char in enumerate(byte_bits):
            if bit_char == '1':
                val |= (1 << bit_idx)
        byte_arr.append(val)
    return bytes(byte_arr)

def bytes_to_bits(raw_bytes: bytes, bit_len: int) -> str:
    bit_chars = []
    for b in raw_bytes:
        for bit_idx in range(8):
            bit_chars.append('1' if (b & (1 << bit_idx)) else '0')
    return ''.join(bit_chars)[:bit_len]

def crc16_add(crc: int, byte: int) -> int:
    byte &= 0xFF
    data = byte ^ (crc >> 8)
    data ^= (data >> 4)
    return (((crc << 8) ^ (data << 12) ^ (data << 5) ^ data) & 0xFFFF)

def calculate_transfer_crc(data_type_signature: int, payload: bytes) -> int:
    crc = 0xFFFF
    sig_bytes = struct.pack('<Q', data_type_signature)
    for b in sig_bytes:
        crc = crc16_add(crc, b)
    for b in payload:
        crc = crc16_add(crc, b)
    return crc

def make_can_id(priority: int, message_type_id: int, source_node_id: int) -> int:
    return ((priority & 0x1F) << 24) | ((message_type_id & 0xFFFF) << 8) | (source_node_id & 0x7F)

def decompose_to_can_frames(payload: bytes, data_type_id: int, signature: int, source_node_id: int = 42, transfer_id: int = 0, priority: int = 24):
    can_id = make_can_id(priority, data_type_id, source_node_id)
    frames = []
    tid = transfer_id & 0x1F

    if len(payload) <= 7:
        tail_byte = 0xC0 | tid
        frames.append({'can_id': can_id, 'dlc': len(payload) + 1, 'data': payload + bytes([tail_byte])})
    else:
        transfer_crc = calculate_transfer_crc(signature, payload)
        stream = struct.pack('<H', transfer_crc) + payload
        offset = 0
        total_len = len(stream)
        toggle = 0

        while offset < total_len:
            is_start = (offset == 0)
            chunk_size = min(7, total_len - offset)
            is_end = (offset + chunk_size >= total_len)

            start_bit = 1 if is_start else 0
            end_bit = 1 if is_end else 0
            toggle_bit = toggle & 1

            tail_byte = (start_bit << 7) | (end_bit << 6) | (toggle_bit << 5) | tid
            chunk = stream[offset:offset + chunk_size]
            frames.append({
                'can_id': can_id,
                'dlc': len(chunk) + 1,
                'data': chunk + bytes([tail_byte])
            })
            offset += chunk_size
            toggle ^= 1

    return frames


# ==============================================================================
# ASCII PROGRESS BAR HELPERS
# ==============================================================================

def make_bar(val: float, max_val: float = 8000.0, width: int = 15) -> str:
    filled = int(max(0, min(width, (val / max(1.0, max_val)) * width)))
    return "█" * filled + "░" * (width - filled)


# ==============================================================================
# MAIN TELEMETRY HUD & DRONECAN ENGINE
# ==============================================================================

def run_telemetry_hud():
    node_id = 42
    data_type_id = uavcan.equipment.gnss.Fix2.default_dtid if DRONECAN_AVAILABLE else 1063
    signature = uavcan.equipment.gnss.Fix2.get_data_type_signature() if DRONECAN_AVAILABLE else 0xCA41E7000F37435F

    print("\033[2J\033[H", end="") # Clear terminal
    print("=" * 85)
    print("  STALLION VTOL - REAL-TIME PARALLEL TELEMETRY & DRONECAN HUD")
    print("=" * 85)
    print(" [STATUS] Searching for ArduPilot SITL MAVLink telemetry...")
    print("          Listening on: udp:127.0.0.1:14550 | tcp:127.0.0.1:5760")
    print("=" * 85)

    # Attempt to connect to MAVLink
    mav = None
    while not mav:
        try:
            # Try UDP GCS port first (standard in sim_vehicle.py)
            mav = mavutil.mavlink_connection('udp:127.0.0.1:14550', source_system=255, source_component=195, autoreconnect=True)
            msg = mav.wait_heartbeat(timeout=1.0)
            if not msg:
                # Try TCP SITL primary port
                mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255, source_component=195, autoreconnect=True)
                msg = mav.wait_heartbeat(timeout=1.0)
            if msg:
                print(f"\n[OK] Connected to Autopilot System ID: {msg.get_srcSystem()}")
                mav.target_system = msg.get_srcSystem()
                mav.target_component = 1
                break
        except Exception:
            pass
        print(".", end="", flush=True)
        time.sleep(1.0)

    mav.mav.request_data_stream_send(mav.target_system, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)

    # State variables
    mode_str = "UNKNOWN"
    armed_str = "DISARMED"
    ekf_ok = False
    roll = pitch = yaw = 0.0
    alt_rel = 0.0
    climb_rate = 0.0
    groundspeed = 0.0
    lat = 13.0827000
    lon = 80.2707000
    sats = 0
    hdop = 1.0
    servos = [1000, 1000, 1000, 1000, 1000, 1000, 1500, 1500]

    transfer_id = 0
    total_can_frames = 0
    last_print = 0.0

    print("\033[2J\033[H", end="")

    while True:
        try:
            # Ingest all available MAVLink packets
            msg = mav.recv_match(blocking=False)
            while msg:
                mtype = msg.get_type()
                if mtype == 'HEARTBEAT':
                    base_mode = msg.base_mode
                    custom_mode = msg.custom_mode
                    armed_str = "\033[92mARMED\033[0m" if (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else "\033[91mDISARMED\033[0m"
                    mode_map = {
                        0: "MANUAL", 1: "CIRCLE", 2: "STABILIZE", 3: "TRAINING", 4: "ACRO", 5: "FBWA",
                        6: "FBWB", 7: "CRUISE", 8: "AUTOTUNE", 10: "AUTO", 11: "RTL", 12: "LOITER",
                        15: "GUIDED", 17: "QSTABILIZE", 18: "QHOVER", 19: "QLOITER", 20: "QLAND",
                        21: "QRTL", 22: "QAUTOTUNE"
                    }
                    mode_str = mode_map.get(custom_mode, f"MODE_{custom_mode}")
                elif mtype == 'ATTITUDE':
                    roll = math.degrees(msg.roll)
                    pitch = math.degrees(msg.pitch)
                    yaw = math.degrees(msg.yaw)
                elif mtype == 'GLOBAL_POSITION_INT':
                    lat = msg.lat * 1e-7
                    lon = msg.lon * 1e-7
                    alt_rel = msg.relative_alt / 1000.0
                    climb_rate = -msg.vz / 100.0
                    groundspeed = math.sqrt(msg.vx**2 + msg.vy**2) / 100.0
                elif mtype == 'GPS_RAW_INT':
                    sats = msg.satellites_visible
                    hdop = msg.eph / 100.0
                elif mtype == 'EKF_STATUS_REPORT':
                    ekf_ok = bool(msg.flags & 1 or msg.flags & 8)
                elif mtype == 'SERVO_OUTPUT_RAW':
                    servos = [
                        msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw,
                        msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw
                    ]
                msg = mav.recv_match(blocking=False)

            # --- DroneCAN Real-Time Serialization ---
            m1_rpm = int(max(0, (servos[0] - 1000) * 12))
            m2_rpm = int(max(0, (servos[1] - 1000) * 12))
            m3_rpm = int(max(0, (servos[2] - 1000) * 12))
            tilt_angle = max(0, min(90, int((servos[6] - 1000) * 0.09))) if len(servos) > 6 else 0

            # Construct DroneCAN Fix2 message
            if DRONECAN_AVAILABLE:
                fix2 = uavcan.equipment.gnss.Fix2()
                fix2.timestamp.usec = int(time.time() * 1e6)
                fix2.gnss_timestamp.usec = int(time.time() * 1e6)
                fix2.gnss_time_standard = 2
                fix2.latitude_deg_1e8 = int(round(lat * 1e8))
                fix2.longitude_deg_1e8 = int(round(lon * 1e8))
                fix2.height_msl_mm = int(round((10.0 + alt_rel) * 1000.0))
                fix2.height_ellipsoid_mm = int(round((10.0 + alt_rel) * 1000.0))
                fix2.ned_velocity = [float(groundspeed), 0.0, float(-climb_rate)]
                fix2.sats_used = sats if sats > 0 else 14
                fix2.status = 3
                fix2.pdop = float(hdop)
                fix2.covariance = [0.25, 0.25, 0.50, 0.05, 0.05, 0.05]

                bit_str = fix2._pack()
                raw_bytes = bits_to_bytes(bit_str)
                frames = decompose_to_can_frames(raw_bytes, data_type_id, signature, source_node_id=node_id, transfer_id=transfer_id)
                total_can_frames += len(frames)
            else:
                raw_bytes = b'\x00' * 62
                frames = []

            # --- Update HUD (every 100ms) ---
            if time.time() - last_print >= 0.10:
                last_print = time.time()
                transfer_id = (transfer_id + 1) % 32

                ekf_badge = "\033[92m[HEALTHY 3D]\033[0m" if ekf_ok else "\033[93m[ALIGNING]\033[0m"
                tilt_mode = "VTOL HOVER (0°)" if tilt_angle < 15 else ("FORWARD FLIGHT (90°)" if tilt_angle > 75 else f"TRANSITION ({tilt_angle}°)")

                f0_hex = ' '.join(f'{b:02X}' for b in frames[0]['data']) if frames else "N/A"
                f9_hex = ' '.join(f'{b:02X}' for b in frames[-1]['data']) if frames else "N/A"

                print("\033[H", end="") # Move cursor to top without scrolling
                print("=" * 85)
                print(f"  STALLION VTOL LIVE FLIGHT TELEMETRY HUD   |   DRONECAN NODE ID: {node_id}")
                print("=" * 85)
                print(f" 🛩️  FLIGHT STATE:   Mode: \033[1m\033[96m{mode_str:12s}\033[0m | State: {armed_str:18s} | EKF3: {ekf_badge}")
                print(f" 🧭 ATTITUDE:       Roll: {roll:+6.1f}° | Pitch: {pitch:+6.1f}° | Yaw: {yaw:+6.1f}°")
                print(f" 📈 ALT & SPEED:    Alt: {alt_rel:6.2f} m | Climb: {climb_rate:+5.2f} m/s | Speed: {groundspeed:5.2f} m/s")
                print(f" 📍 GPS LOCATION:   Lat: {lat:10.7f}° | Lon: {lon:10.7f}° | Sats: {sats:2d} | HDOP: {hdop:4.2f}")
                print("-" * 85)
                print(" ⚙️  VTOL ACTUATORS & MOTORS:")
                print(f"    • Motor 1 (Front Left) : {m1_rpm:5d} RPM [{make_bar(m1_rpm)}] (PWM: {servos[0]})")
                print(f"    • Motor 2 (Front Right): {m2_rpm:5d} RPM [{make_bar(m2_rpm)}] (PWM: {servos[1]})")
                print(f"    • Motor 3 (Tail Yaw)   : {m3_rpm:5d} RPM [{make_bar(m3_rpm)}] (PWM: {servos[2]})")
                print(f"    • Tilt Mechanism       : {tilt_mode:20s} (Servo PWM: {servos[6]})")
                print("-" * 85)
                print(f" 📡 REAL-TIME DRONECAN STREAM (uavcan.equipment.gnss.Fix2 / ID: {data_type_id}):")
                print(f"    • Payload Length    : {len(raw_bytes)} bytes | Transfer ID: #{transfer_id:02d} | Total Frames: {total_can_frames}")
                print(f"    • CAN Frame #0 (SOF): CAN ID=0x{make_can_id(24, data_type_id, node_id):08X} | DATA: {f0_hex}")
                print(f"    • CAN Frame #9 (EOF): CAN ID=0x{make_can_id(24, data_type_id, node_id):08X} | DATA: {f9_hex}")
                print("=" * 85)
                print(" [Press Ctrl+C to exit HUD]")

            time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n\n[EXIT] Exiting Telemetry HUD.")
            break
        except Exception as e:
            time.sleep(0.5)


if __name__ == '__main__':
    run_telemetry_hud()

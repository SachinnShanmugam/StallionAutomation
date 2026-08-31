#!/usr/bin/env python3
"""
Stallion VTOL - Gazebo to CAN Bus Telemetry Bridge & Actuator Node
===================================================================
Extracts high-fidelity physical state & sensor telemetry from Gazebo
and broadcasts across standardized CAN channels (CAN 2.0B / CAN-over-IP).

CAN Frame Architecture:
  0x101: CAN_IMU_ACCEL       - [Ax, Ay, Az] (int16, 0.01 m/s^2) + Seq (uint16)
  0x102: CAN_IMU_GYRO        - [Gx, Gy, Gz] (int16, 0.001 rad/s) + Temp (int16)
  0x110: CAN_ATTITUDE_EULER  - [Roll, Pitch, Yaw] (int16, 0.01 deg) + Flags (uint16)
  0x120: CAN_BARO_AIRSPEED   - PressAlt (int32, cm) + Airspeed (uint16, mm/s) + Temp (int16, 0.1C)
  0x131: CAN_GPS_POS         - Lat, Lon (int32, 1e-7 deg)
  0x132: CAN_GPS_VEL_ALT     - Alt (int32, cm) + Spd (uint16, cm/s) + Hdg (uint16, 0.1 deg)
  0x140: CAN_ESC_STATUS_1    - Motor1 RPM (uint16), Motor2 RPM (uint16), Volts (uint16, 0.01V)
  0x141: CAN_ESC_STATUS_2    - Motor3 RPM (uint16), Tilt_L (int16, 0.01deg), Tilt_R (int16, 0.01deg)
  0x200: CAN_ACTUATOR_CMD    - [Rx] Motor1..3 (uint16 PWM) + Servos (uint16 PWM)
"""

import os
import sys
import time
import json
import math
import struct
import socket
import threading

try:
    import can
    HAS_PYTHON_CAN = True
except ImportError:
    HAS_PYTHON_CAN = False

class GazeboCANBridgeNode:
    def __init__(self, gazebo_port=9003, can_interface='virtual', channel='vcan0', udp_can_port=10005):
        self.gazebo_port = gazebo_port
        self.can_interface = can_interface
        self.channel = channel
        self.udp_can_port = udp_can_port
        self.running = False
        self.seq = 0

        # Sensor State Cache
        self.state = {
            'timestamp': 0.0,
            'accel': [0.0, 0.0, -9.81],
            'gyro': [0.0, 0.0, 0.0],
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'lat': 13.0827,
            'lon': 80.2707,
            'alt': 10.0,
            'groundspeed': 0.0,
            'heading': 90.0,
            'airspeed': 0.0,
            'motors_rpm': [0, 0, 0],
            'tilts_deg': [0.0, 0.0]
        }

        # Setup CAN Bus
        self.bus = None
        self.udp_can_sock = None
        self._init_can_backend()

    def _init_can_backend(self):
        """Initialize CAN backend (SocketCAN, Virtual, or UDP Broadcast)"""
        # Always open UDP CAN-over-IP socket for cross-platform/Windows accessibility
        self.udp_can_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_can_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_can_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        if HAS_PYTHON_CAN and self.can_interface != 'udp':
            try:
                self.bus = can.interface.Bus(channel=self.channel, interface=self.can_interface)
                print(f"[CAN] Connected to python-can interface '{self.can_interface}' on '{self.channel}'")
            except Exception as e:
                print(f"[CAN-WARN] python-can init failed ({e}). Defaulting to UDP CAN-over-IP on port {self.udp_can_port}")
                self.bus = None

    def send_can_frame(self, arbitration_id, data):
        """Send a CAN 2.0B frame across hardware/socketcan and UDP IP bridge"""
        # 1. Send via python-can (if hardware or socketcan available)
        if self.bus:
            try:
                msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
                self.bus.send(msg)
            except Exception:
                pass

        # 2. Broadcast via UDP CAN-over-IP (Frame Format: [uint32 ID, uint8 DLC, uint8 Data[8]])
        payload = struct.pack('<IB8s', arbitration_id, len(data), bytes(data).ljust(8, b'\x00'))
        try:
            self.udp_can_sock.sendto(payload, ('127.0.0.1', self.udp_can_port))
        except Exception:
            pass

    def encode_and_publish_telemetry(self):
        """Encodes all cached state into CAN channels and publishes"""
        self.seq = (self.seq + 1) & 0xFFFF

        # --- 0x101: IMU Accelerometer ---
        ax = int(max(-32767, min(32767, self.state['accel'][0] * 100)))
        ay = int(max(-32767, min(32767, self.state['accel'][1] * 100)))
        az = int(max(-32767, min(32767, self.state['accel'][2] * 100)))
        imu_accel_data = struct.pack('<hhhH', ax, ay, az, self.seq)
        self.send_can_frame(0x101, imu_accel_data)

        # --- 0x102: IMU Gyroscope ---
        gx = int(max(-32767, min(32767, self.state['gyro'][0] * 1000)))
        gy = int(max(-32767, min(32767, self.state['gyro'][1] * 1000)))
        gz = int(max(-32767, min(32767, self.state['gyro'][2] * 1000)))
        temp_c = int(25.0 * 10)  # 25.0 C
        imu_gyro_data = struct.pack('<hhhh', gx, gy, gz, temp_c)
        self.send_can_frame(0x102, imu_gyro_data)

        # --- 0x110: Attitude Euler Angles ---
        roll_val = int(max(-32767, min(32767, self.state['roll'] * 100)))
        pitch_val = int(max(-32767, min(32767, self.state['pitch'] * 100)))
        yaw_val = int(max(-32767, min(32767, self.state['yaw'] * 100)))
        flags = 0x0001  # EKF Healthy
        att_data = struct.pack('<hhhH', roll_val, pitch_val, yaw_val, flags)
        self.send_can_frame(0x110, att_data)

        # --- 0x120: Baro & Airspeed ---
        press_alt_cm = int(self.state['alt'] * 100)
        airspeed_mms = int(max(0, self.state['airspeed'] * 1000))
        baro_data = struct.pack('<iHh', press_alt_cm, airspeed_mms, temp_c)
        self.send_can_frame(0x120, baro_data)

        # --- 0x131: GPS Position (Lat/Lon) ---
        lat_i7 = int(self.state['lat'] * 1e7)
        lon_i7 = int(self.state['lon'] * 1e7)
        gps_pos_data = struct.pack('<ii', lat_i7, lon_i7)
        self.send_can_frame(0x131, gps_pos_data)

        # --- 0x132: GPS Velocity & Altitude ---
        alt_cm = int(self.state['alt'] * 100)
        spd_cms = int(self.state['groundspeed'] * 100)
        hdg_d10 = int(self.state['heading'] * 10)
        gps_vel_data = struct.pack('<iHH', alt_cm, spd_cms, hdg_d10)
        self.send_can_frame(0x132, gps_vel_data)

        # --- 0x140: ESC Status 1 (Front Motors) ---
        rpm1 = int(self.state['motors_rpm'][0])
        rpm2 = int(self.state['motors_rpm'][1])
        volts = int(24.2 * 100)  # 24.2V (6S LiPo)
        esc1_data = struct.pack('<HHH', rpm1, rpm2, volts)
        self.send_can_frame(0x140, esc1_data)

        # --- 0x141: ESC Status 2 (Rear Motor + Tilts) ---
        rpm3 = int(self.state['motors_rpm'][2])
        tilt_l = int(self.state['tilts_deg'][0] * 100)
        tilt_r = int(self.state['tilts_deg'][1] * 100)
        esc2_data = struct.pack('<Hhh', rpm3, tilt_l, tilt_r)
        self.send_can_frame(0x141, esc2_data)

    def start(self):
        """Start listening to Gazebo physics & streaming CAN frames"""
        self.running = True
        print("=" * 80)
        print("  STALLION VTOL - GAZEBO TO CAN BUS TELEMETRY BRIDGE NODE")
        print("=" * 80)
        print(f"[BRIDGE] Ingest Port: UDP {self.gazebo_port} (Gazebo Physics)")
        print(f"[BRIDGE] CAN Channels: 0x101, 0x102, 0x110, 0x120, 0x131, 0x132, 0x140, 0x141")
        print(f"[BRIDGE] CAN-over-IP Broadcast: UDP 127.0.0.1:{self.udp_can_port}")
        print("=" * 80)

        # 1. Listen for Gazebo JSON UDP Telemetry
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', self.gazebo_port))
        sock.settimeout(0.5)

        publish_timer = time.time()

        while self.running:
            try:
                data, _ = sock.recvfrom(4096)
                msg = json.loads(data.decode('utf-8'))
                
                # Ingest state
                self.state['timestamp'] = msg.get('timestamp', time.time())
                self.state['accel'] = [msg.get('imu', {}).get('accel_body', {}).get(k, 0.0) for k in ['x', 'y', 'z']]
                self.state['gyro'] = [msg.get('imu', {}).get('gyro', {}).get(k, 0.0) for k in ['x', 'y', 'z']]
                self.state['roll'] = msg.get('attitude', {}).get('roll', 0.0)
                self.state['pitch'] = msg.get('attitude', {}).get('pitch', 0.0)
                self.state['yaw'] = msg.get('attitude', {}).get('yaw', 0.0)
                self.state['alt'] = msg.get('position', {}).get('z', 10.0)
                self.state['lat'] = msg.get('gps', {}).get('lat', 13.0827)
                self.state['lon'] = msg.get('gps', {}).get('lon', 80.2707)

            except socket.timeout:
                pass
            except Exception:
                pass

            # Publish CAN Frames at 50 Hz (every 20ms)
            if time.time() - publish_timer >= 0.02:
                self.encode_and_publish_telemetry()
                publish_timer = time.time()

    def stop(self):
        self.running = False
        if self.bus:
            self.bus.shutdown()
        if self.udp_can_sock:
            self.udp_can_sock.close()

if __name__ == '__main__':
    node = GazeboCANBridgeNode()
    try:
        node.start()
    except KeyboardInterrupt:
        node.stop()
        print("\n[CAN BRIDGE] Node stopped gracefully.")

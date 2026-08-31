# Stallion VTOL - Gazebo to CAN Bus Interface & Telemetry Node
================================================================

This document details the CAN Bus channel mapping, packet architecture, and node implementation for extracting live physics telemetry from Gazebo and streaming to external hardware flight controllers (STM32, Matek H743, Pixhawk, Raspberry Pi) via standard CAN 2.0B / CAN FD / DroneCAN.

---

## 1. CAN Channel Architecture & Message IDs

| CAN ID | Name | DLC | Frequency | Description | Bit Layout & Units |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`0x101`** | `CAN_IMU_ACCEL` | 8 | 50 Hz | Body Accelerations & Seq | `int16 ax, ay, az` (0.01 m/s²), `uint16 seq` |
| **`0x102`** | `CAN_IMU_GYRO` | 8 | 50 Hz | Angular Rates & Temp | `int16 gx, gy, gz` (0.001 rad/s), `int16 temp` (0.1 °C) |
| **`0x110`** | `CAN_ATTITUDE` | 8 | 50 Hz | Euler Attitude & Status | `int16 roll, pitch, yaw` (0.01°), `uint16 flags` |
| **`0x120`** | `CAN_BARO_AIRSPEED`| 8 | 50 Hz | Pressure Alt & Pitot | `int32 press_alt` (cm), `uint16 airspeed` (mm/s), `int16 temp` (0.1 °C) |
| **`0x131`** | `CAN_GPS_POS` | 8 | 10 Hz | Navigation Coordinates | `int32 lat, lon` (1e-7 deg) |
| **`0x132`** | `CAN_GPS_VEL_ALT` | 8 | 10 Hz | GPS Altitude & Speed | `int32 alt` (cm), `uint16 speed` (cm/s), `uint16 heading` (0.1°) |
| **`0x140`** | `CAN_ESC_STATUS_1`| 6 | 50 Hz | Front Motors & Voltage | `uint16 m1_rpm, m2_rpm`, `uint16 volts` (0.01V) |
| **`0x141`** | `CAN_ESC_STATUS_2`| 6 | 50 Hz | Rear Motor & Tilt Angles | `uint16 m3_rpm`, `int16 tilt_l, tilt_r` (0.01°) |
| **`0x200`** | `CAN_ACTUATOR_CMD`| 8 | 50 Hz | Hardware Control (Rx) | `uint16 m1, m2, m3, servo` (1000–2000 µs PWM) |

---

## 2. Running the Gazebo-to-CAN Node

### A. Start the CAN Bridge Node
```bash
python3 scripts/can_gazebo_bridge.py
```
* Ingests live physics from Gazebo.
* Broadcasts formatted CAN frames over hardware CAN (`socketcan` / `vcan0` / `slcan`) and UDP CAN-over-IP (`127.0.0.1:10005`).

### B. Start the CAN Subscriber / Controller Node
```bash
python3 scripts/test_can_controller_node.py 10
```
* Subscribes to CAN frames.
* Decodes and displays live Roll/Pitch/Yaw, Baro Altitude, GPS coordinates, and ESC RPMs in real-time.

---

## 3. Hardware Transceiver Pinout (STM32 / Matek H743)

```text
       [ Matek H743 / STM32 ]                    [ Physical CAN Bus ]
       ┌─────────────────────┐                  ┌──────────────────┐
       │ CAN1_TX  (PD1)      │ ───► TXD ──┐     │ CAN_H  (Pin 7)   │ ───────► (To Node)
       │ CAN1_RX  (PD0)      │ ◄─── RXD ──┤TJA  │ CAN_L  (Pin 2)   │ ───────► (To Node)
       │ GND                 │ ───────────┤1051 │ 120Ω Term Res    │
       │ +5V                 │ ───────────┤XCVR │ GND              │
       └─────────────────────┘            └───┴─┴──────────────────┘
```

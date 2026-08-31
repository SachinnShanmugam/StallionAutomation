# Stallion VTOL - Hybrid HIL Architecture & DroneCAN Node Integration Guide
================================================================================

This document establishes the verified hardware-in-the-loop (HIL) architecture and DroneCAN sensor node implementation for the Stallion Tilt-Rotor VTOL using the Matek H743 flight controller and Gazebo Harmonic simulation.

---

## 1. Verified Hybrid HIL Architecture

```text
                             GAZEBO HARMONIC
                                    │
                 ┌──────────────────┴──────────────────┐
                 ↓                                     ↓
           High-Rate IMU                       Lower-Rate Sensors
         (Gyro + Accel Physics)                 (GPS / Mag / Baro)
                 │                                     │
                 ↓                                     ↓
         Ethernet / SoH Path                      DroneCAN Path
     (JSON FDM @ 400 Hz Lockstep)          (uavcan.equipment.gnss.Fix2)
                 │                                     │
                 ↓                                     ↓
         Matek H743 HAL                          Matek CAN Driver
     (AP_HAL_ChibiOS SimOnHardware)            (AP_GPS_DroneCAN Driver)
                 │                                     │
                 └──────────────────┬──────────────────┘
                                    ↓
                                  EKF3
                                    ↓
                          ArduPilot Control Loops
```

---

## 2. Architectural Boundary & Driver Reality

> **Important Driver Note:**  
> Stock ArduPilot does **not** provide a generic DroneCAN `RawIMU` (`#1003`) $\to$ `AP_InertialSensor` driver for using DroneCAN as the primary flight control IMU.  
> Primary rate-loop IMUs in ArduPilot expect local high-speed SPI/DMA sampling or `SimOnHardware` HAL injection.  
> 
> Therefore, we strictly separate the HIL transport into:
> * **High-Rate IMU (Gyros & Accels):** Handled via **`SimOnHardware` / Ethernet** (Phase 2).
> * **Lower-Rate Sensors (GPS, Compass, Baro):** Handled via **DroneCAN Peripheral Drivers** (Phase 3).

---

## 3. Supported & Verified DroneCAN Message Matrix

Inspected directly against the local ArduPilot source code (`/home/drones/ardupilot`):

| Sensor Type | DroneCAN Message Expected | DSDL Message ID | Signature | ArduPilot Driver | Status |
| :--- | :--- | :---: | :---: | :--- | :---: |
| **GPS / GNSS** | `uavcan.equipment.gnss.Fix2` | `1063` (`0x0427`) | `0xCA41E7000F37435F` | `AP_GPS_DroneCAN` | ✅ **VERIFIED** |
| **Compass / Mag** | `uavcan.equipment.ahrs.MagneticFieldStrength2` | `1002` (`0x03EA`) | `0x47B0FB5819777E44` | `AP_Compass_DroneCAN` | ✅ Supported |
| **Barometer** | `uavcan.equipment.air_data.StaticPressure` | `1028` (`0x0404`) | `0xCE8CEBE24B022206` | `AP_Baro_DroneCAN` | ✅ Supported |
| **Airspeed (Pitot)** | `uavcan.equipment.air_data.TrueAirspeed` | `1020` (`0x03FC`) | `0x32130CE2F67448D6` | `AP_Airspeed_DroneCAN` | ✅ Supported |
| **ESC RPM / Status** | `uavcan.equipment.esc.Status` | `1034` (`0x040A`) | `0xA9A662369B566542` | `AP_ESC_DroneCAN` | ✅ Supported |

---

## 4. DroneCAN GPS Node Implementation (`scripts/dronecan_gps_node.py`)

* **Node ID:** `42`
* **Message Type:** `uavcan.equipment.gnss.Fix2` (62 Bytes binary payload)
* **CAN Framing:** 10 Multi-Frame CAN 2.0B Extended Frames (`CAN ID: 0x1804272A` with CCITT-CRC16 and protocol tail bytes)
* **Verification:** Built-in lossless roundtrip decode verification ($\Delta Lat = 0.000000000^\circ$).

### Running Locally (Without CAN Hardware):

```bash
# 1. Run Fixed GPS Test (10 Hz):
python scripts/dronecan_gps_node.py fixed 5

# 2. Run Live Gazebo Stream Mode:
python scripts/dronecan_gps_node.py gazebo 10

# 3. Run Parallel Telemetry & DroneCAN HUD:
python scripts/live_flight_telemetry_hud.py
```

---

## 5. Physical CAN Hardware Connection (For Future Testing)

When physical CAN transceivers (e.g. CANable, Candlelight, or Matek CAN adapter) arrive:

```text
       [ Matek H743 Flight Controller ]              [ USB-CAN Transceiver / Node ]
       ┌──────────────────────────────┐              ┌────────────────────────────┐
       │ CAN1_TX  (PD1)               │ ───► TXD ──┐ │ CAN_H  (Pin 7)             │ ───────►
       │ CAN1_RX  (PD0)               │ ◄─── RXD ──┤ │ CAN_L  (Pin 2)             │ ───────►
       │ GND                          │ ───────────┤ │ 120Ω Terminating Resistor  │
       │ +5V                          │ ───────────┤ │ GND                        │
       └──────────────────────────────┘            └─┴────────────────────────────┘
```

### Matek H743 ArduPilot Parameters for DroneCAN GPS:
```text
CAN_P1_DRIVER  = 1        # Enable CAN 1 Driver
CAN_D1_PROTOCOL = 1       # DroneCAN Protocol
GPS_TYPE       = 9        # DroneCAN GPS
GPS_CAN_NODEID = 42       # Override to Node ID 42 (or 0 for auto-discovery)
```

---

## 6. Implementation Roadmap

1. ✅ **Simulated DroneCAN GPS Node:** Built and verified with authentic DSDL bitstream serialization.
2. ✅ **Gazebo GPS Ingestion:** Dynamic coordinate updates stream through the node without hardware.
3. ✅ **Parallel Telemetry HUD:** Passive listener displays live flight telemetry and 29-bit CAN frames at 10 Hz.
4. ✅ **Ethernet / SimOnHardware IMU Path:** High-rate physics path preserved for H743 lockstep control.
5. ⏳ **Physical CAN Hardware Test:** Connect physical CAN1/CAN2 to H743 when adapter arrives.

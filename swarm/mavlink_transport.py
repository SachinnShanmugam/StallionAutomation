"""
swarm/mavlink_transport.py — MAVLink Drone Connection for Stallion Swarm V1
============================================================================
Wraps pymavlink for one ArduPlane SITL instance.
The swarm layer uses this to:
  - Identify the drone's SYSID (heartbeat)
  - Issue navigation commands (GOTO waypoint, QLAND)
  - Monitor flight status (position, mode)

ArduPilot retains full responsibility for:
  EKF, attitude control, navigation, failsafes, landing.

The swarm layer only issues high-level MAVLink commands.
"""

from __future__ import annotations

import time
import threading
from typing import Optional, Tuple
from pymavlink import mavutil


class MAVLinkDrone:
    """
    MAVLink connection wrapper for one ArduPlane SITL instance.

    Usage:
        drone = MAVLinkDrone("tcp:127.0.0.1:5760", expected_sysid=1)
        drone.connect(timeout=30)
        drone.arm()
        drone.goto(lat, lon, alt)
        drone.land()
    """

    def __init__(self, connection_string: str, expected_sysid: int):
        self.connection_string = connection_string
        self.expected_sysid    = expected_sysid
        self._mav: Optional[mavutil.mavlink_connection] = None
        self._lock = threading.Lock()

        # Telemetry cache (updated by background listener)
        self._lat: float  = 0.0
        self._lon: float  = 0.0
        self._alt: float  = 0.0
        self._mode: str   = "UNKNOWN"
        self._armed: bool = False
        self._connected   = False
        self._last_hb: float = 0.0

        self._listener_thread: Optional[threading.Thread] = None
        self._rc_thread: Optional[threading.Thread] = None
        self._running = False
        self._rc_throttle: int = 1000
        self._rc_active: bool = False

    def connect(self, timeout: float = 30.0) -> bool:
        """
        Open MAVLink connection and wait for a heartbeat from expected_sysid.
        Returns True on success.
        """
        print(f"[MAV] Connecting to SYSID={self.expected_sysid} "
              f"at {self.connection_string} ...")

        self._mav = mavutil.mavlink_connection(
            self.connection_string,
            source_system=255,
            source_component=190,
            autoreconnect=True,
        )

        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if msg and msg.get_srcSystem() == self.expected_sysid:
                if msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                    self._mav.target_system    = self.expected_sysid
                    self._mav.target_component = 1
                    self._connected = True
                    self._last_hb   = time.time()
                    print(f"[MAV] Connected to SYSID={self.expected_sysid} ✓")

                    # Configure telemetry stream and QuadPlane parameters
                    self._mav.mav.request_data_stream_send(
                        self.expected_sysid, 1,
                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
                    )
                    self._mav.param_set_send('RC_OVERRIDE_TIME', 60.0)
                    self._mav.param_set_send('ARMING_CHECK', 0.0)
                    self._mav.param_set_send('Q_ENABLE', 1.0)
                    self._mav.param_set_send('Q_FRAME_CLASS', 7.0)

                    # Start background listener & RC streamer threads
                    self._running = True
                    self._rc_active = True
                    self._listener_thread = threading.Thread(
                        target=self._telemetry_loop, daemon=True
                    )
                    self._rc_thread = threading.Thread(
                        target=self._rc_streamer_loop, daemon=True
                    )
                    self._listener_thread.start()
                    self._rc_thread.start()
                    return True
            time.sleep(0.2)

        print(f"[MAV] TIMEOUT waiting for SYSID={self.expected_sysid}")
        return False

    @property
    def is_connected(self) -> bool:
        if not self._connected:
            return False
        # Stale heartbeat = lost link
        if time.time() - self._last_hb > 5.0:
            return False
        return True

    @property
    def position(self) -> Tuple[float, float, float]:
        """Returns (lat_deg, lon_deg, alt_m_amsl)."""
        return (self._lat, self._lon, self._alt)

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def armed(self) -> bool:
        return self._armed

    def seconds_since_heartbeat(self) -> float:
        return time.time() - self._last_hb

    def set_throttle(self, throttle_pwm: int) -> None:
        """Update continuous 50 Hz throttle output (1000 = idle, 1500 = hover, 1680 = climb)."""
        self._rc_throttle = int(max(1000, min(2000, throttle_pwm)))

    # ── MAVLink Commands ─────────────────────────────────────────────────────

    def set_mode(self, mode_name: str) -> bool:
        """Set flight mode by name (e.g. 'QLOITER', 'QHOVER', 'QLAND', 'GUIDED')."""
        if not self._mav:
            return False
        mode_id = self._mav.mode_mapping().get(mode_name)
        if mode_id is None:
            print(f"[MAV] Unknown mode: {mode_name}")
            return False
        with self._lock:
            self._mav.mav.set_mode_send(
                self.expected_sysid,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
        print(f"[MAV] Mode set → {mode_name}")
        return True

    def arm(self, force: bool = True) -> bool:
        """Arm the vehicle."""
        if not self._mav:
            return False
        with self._lock:
            self._mav.mav.command_long_send(
                self.expected_sysid, 1,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,                  # arm=1
                21196 if force else 0,  # force magic number
                0, 0, 0, 0, 0,
            )
        print(f"[MAV] ARM command sent to SYSID={self.expected_sysid}")
        return True

    def takeoff(self, alt: float = 15.0) -> bool:
        """Command smooth VTOL climb to target altitude in meters."""
        if not self._mav:
            return False
        
        print(f"[MAV] Smooth VTOL Takeoff to {alt:.1f}m (SYSID={self.expected_sysid})...")
        self.set_mode("QHOVER")
        time.sleep(0.5)
        self.arm(force=True)
        time.sleep(1.0)

        # Climb at 1680 µs for 6 seconds
        self.set_throttle(1680)
        time.sleep(6.0)

        # Level off to steady 1500 µs (Hover hold)
        self.set_throttle(1500)
        self.set_mode("QLOITER")
        print(f"[MAV] Target altitude reached. Holding steady in QLOITER mode.")
        return True

    def goto(self, lat: float, lon: float, alt: float) -> bool:
        """
        Send SET_POSITION_TARGET_GLOBAL_INT in GUIDED mode.
        ArduPilot navigates to the coordinate autonomously.
        """
        if not self._mav:
            return False

        # Switch to GUIDED mode
        self.set_mode("GUIDED")
        time.sleep(0.5)

        with self._lock:
            self._mav.mav.mission_item_int_send(
                self.expected_sysid, 1,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                2, 1,               # current=2 (guided), autocontinue=1
                0, 0, 0, 0,
                int(lat * 1e7),
                int(lon * 1e7),
                alt,
            )
        print(f"[MAV] GOTO → lat={lat:.7f} lon={lon:.7f} alt={alt:.1f}m "
              f"(SYSID={self.expected_sysid})")
        return True

    def land(self) -> bool:
        """Command QLAND mode for vertical landing."""
        if not self._mav:
            return False
        self.set_throttle(1000)
        result = self.set_mode("QLAND")
        print(f"[MAV] QLAND initiated (SYSID={self.expected_sysid})")
        return result

    def wait_for_mode(self, mode_name: str, timeout: float = 30.0) -> bool:
        """Block until the drone reports the expected flight mode."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._mode == mode_name:
                return True
            time.sleep(0.5)
        return False

    def wait_near(
        self,
        lat: float,
        lon: float,
        radius_m: float = 15.0,
        timeout: float = 120.0,
    ) -> bool:
        """Block until drone is within radius_m of the target coordinate."""
        import math
        t0 = time.time()
        while time.time() - t0 < timeout:
            dlat = math.radians(self._lat - lat)
            dlon = math.radians(self._lon - lon)
            a = (math.sin(dlat / 2) ** 2 +
                 math.cos(math.radians(lat)) *
                 math.cos(math.radians(self._lat)) *
                 math.sin(dlon / 2) ** 2)
            dist = 6371000 * 2 * math.asin(math.sqrt(a))
            if dist <= radius_m:
                return True
            time.sleep(1.0)
        return False

    def wait_landed(self, timeout: float = 60.0) -> bool:
        """Block until drone altitude drops below 1m (on-ground)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._alt < 1.0:
                return True
            time.sleep(1.0)
        return False

    def close(self) -> None:
        self._running = False
        if self._mav:
            self._mav.close()

    # ── Background telemetry listener ────────────────────────────────────────

    def _telemetry_loop(self) -> None:
        while self._running:
            try:
                msg = self._mav.recv_match(
                    type=["HEARTBEAT", "GLOBAL_POSITION_INT", "HEARTBEAT"],
                    blocking=True, timeout=1.0
                )
                if not msg:
                    continue

                mtype = msg.get_type()
                src   = msg.get_srcSystem()

                if src != self.expected_sysid:
                    continue

                if mtype == "HEARTBEAT":
                    self._last_hb = time.time()
                    mode_id = msg.custom_mode
                    mode_map_rev = {v: k for k, v in self._mav.mode_mapping().items()}
                    self._mode  = mode_map_rev.get(mode_id, str(mode_id))
                    self._armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

                elif mtype == "GLOBAL_POSITION_INT":
                    self._lat = msg.lat / 1e7
                    self._lon = msg.lon / 1e7
                    self._alt = msg.relative_alt / 1000.0

            except Exception:
                if not self._running:
                    break
                time.sleep(0.1)

    def _rc_streamer_loop(self) -> None:
        """Continuously streams 50 Hz RC channel overrides to maintain stable flight/hover."""
        while self._running and self._rc_active:
            try:
                if self._mav and self._connected:
                    with self._lock:
                        self._mav.mav.rc_channels_override_send(
                            self.expected_sysid, 1,
                            1500, 1500, self._rc_throttle, 1500,
                            65535, 65535, 65535, 65535
                        )
            except Exception:
                pass
            time.sleep(0.02) # 50 Hz exact

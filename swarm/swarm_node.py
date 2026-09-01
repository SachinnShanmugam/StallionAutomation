"""
swarm/swarm_node.py — SwarmNode Base Class for Stallion Swarm V1
================================================================
Provides the heartbeat loop and message dispatch infrastructure
shared by both LeaderNode and FollowerNode.

Subclasses override on_message() to handle incoming SwarmMessages.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from swarm.config import get_config
from swarm.messages import SwarmMessage, MessageType, make_heartbeat
from swarm.transport import BaseTransport
from swarm.mavlink_transport import MAVLinkDrone


class SwarmNode:
    """
    Base class for all swarm nodes.

    Responsibilities:
        - Connect to own MAVLink SITL instance
        - Send periodic heartbeats over the transport
        - Run a message pump loop (recv → dispatch to on_message)
        - Track peer heartbeat timestamps (for > peers command)
    """

    def __init__(self, own_sysid: int, transport: BaseTransport, drone: MAVLinkDrone):
        self.own_sysid  = own_sysid
        self.transport  = transport
        self.drone      = drone
        self.cfg        = get_config()
        self._running   = False

        # Peer heartbeat tracking: {sysid: last_heartbeat_time}
        self._peers: dict[int, float] = {}

        self._hb_thread:   Optional[threading.Thread] = None
        self._pump_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Start heartbeat and message pump background threads."""
        self._running = True
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True, name=f"hb-{self.own_sysid}"
        )
        self._pump_thread = threading.Thread(
            target=self._message_pump, daemon=True, name=f"pump-{self.own_sysid}"
        )
        self._hb_thread.start()
        self._pump_thread.start()

    def stop(self) -> None:
        """Stop all background threads."""
        self._running = False
        self.transport.close()
        self.drone.close()

    # ── Override in subclasses ────────────────────────────────────────────────

    def on_message(self, msg: SwarmMessage) -> None:
        """Called for every received SwarmMessage. Override in Leader/Follower."""
        pass

    # ── Peer & transport status (for CLI commands) ───────────────────────────

    def peers_summary(self) -> str:
        now = time.time()
        if not self._peers:
            return "  No peers discovered yet."
        lines = []
        for sysid, last in self._peers.items():
            age = now - last
            status = "OK" if age < 5.0 else "STALE"
            lines.append(f"  SYSID {sysid}: last heartbeat {age:.1f}s ago  [{status}]")
        return "\n".join(lines)

    def transport_summary(self) -> str:
        stats = self.transport.get_stats()
        return (
            f"  TX packets   : {stats.get('tx_count', 0)}\n"
            f"  RX packets   : {stats.get('rx_count', 0)}\n"
            f"  Dropped      : {stats.get('drop_count', 0)}\n"
            f"  Avg latency  : {stats.get('avg_latency', 0.0):.1f} ms\n"
            f"  Effective loss: {stats.get('loss_pct', 0.0):.1f}%"
        )

    def drone_summary(self) -> str:
        lat, lon, alt = self.drone.position
        return (
            f"  SYSID        : {self.own_sysid}\n"
            f"  Connection   : {'OK' if self.drone.is_connected else 'LOST'}\n"
            f"  Mode         : {self.drone.mode}\n"
            f"  Armed        : {self.drone.armed}\n"
            f"  Position     : {lat:.7f}, {lon:.7f} @ {alt:.1f}m AGL\n"
            f"  Last HB      : {self.drone.seconds_since_heartbeat():.1f}s ago"
        )

    # ── Background threads ───────────────────────────────────────────────────

    def _heartbeat_loop(self) -> None:
        interval = self.cfg.swarm.heartbeat_interval_seconds
        while self._running:
            hb = make_heartbeat(self.own_sysid)
            self.transport.send(hb)
            time.sleep(interval)

    def _message_pump(self) -> None:
        while self._running:
            msg = self.transport.recv(timeout=0.1)
            if msg is None:
                continue

            # Track peer heartbeats
            if msg.message_type == MessageType.HEARTBEAT:
                self._peers[msg.sender_id] = time.time()
                continue  # Don't pass heartbeats to on_message

            # Dispatch to subclass
            try:
                self.on_message(msg)
            except Exception as e:
                print(f"[NODE ERR] on_message exception: {e}")

"""
swarm/leader.py — Leader Node for Stallion Swarm V1
=====================================================
Connects to Drone 1 (SYSID=1), maintains heartbeat, and exposes an
interactive readline CLI for manual threat/task invocation.

IMPORTANT: The Leader does NOT automatically generate any tasks.
Tasks are ONLY created when you type the 'threat' command.

Usage:
    python -m swarm.leader
    # or from the swarm/ directory:
    python leader.py

CLI Commands:
    threat --target LAT,LON --alt ALT [--priority HIGH|MEDIUM|LOW] [--expiry SECONDS]
    status          — Show drone + swarm status
    peers           — Show connected peers and last heartbeat
    tasks           — Show all task records
    transport       — Show LoRa transport statistics
    stats           — Alias for transport
    set loss N      — Change packet loss % (0–100)
    set latency N   — Change latency in ms
    set jitter N    — Change jitter in ms
    help            — Show this help
    quit / exit     — Shutdown leader
"""

from __future__ import annotations

import sys
import threading
import time
import readline  # noqa: F401 — enables arrow-key history in input()

from swarm.config import get_config
from swarm.lora_transport import SimulatedLoRaTransport
from swarm.mavlink_transport import MAVLinkDrone
from swarm.messages import (
    MessageType, Priority, TaskAction,
    make_task_ack,
)
from swarm.swarm_node import SwarmNode
from swarm.task_manager import TaskManager


# ── Leader SYSID and peer ────────────────────────────────────────────────────
LEADER_SYSID   = 1
FOLLOWER_SYSID = 2   # V1 only — extend peer list for N-drone


class LeaderNode(SwarmNode):
    """
    Leader drone node (SYSID=1).

    Handles:
        - Manual threat → TASK → LORA TX → wait ACK → receive TASK_COMPLETE
        - Interactive CLI for all debug and control commands
    """

    def __init__(self):
        cfg  = get_config()

        # MAVLink connection to Drone 1
        mav_cfg = cfg.mavlink.get("drone1")
        conn    = mav_cfg.connection if mav_cfg else "tcp:127.0.0.1:5762"
        drone   = MAVLinkDrone(conn, expected_sysid=LEADER_SYSID)

        # Simulated LoRa transport (Leader sends to Follower's port)
        transport = SimulatedLoRaTransport(
            own_sysid=LEADER_SYSID,
            peer_sysids=[FOLLOWER_SYSID],
            cfg=cfg.lora,
        )

        super().__init__(LEADER_SYSID, transport, drone)
        self.task_mgr = TaskManager(LEADER_SYSID, cfg.swarm)

    def start_and_connect(self, timeout: float = 30.0) -> bool:
        """Connect MAVLink and start background threads."""
        print(f"\n[LEADER] Connecting to Drone 1 SITL ...")
        if not self.drone.connect(timeout=timeout):
            print("[LEADER] ERROR: Could not connect to Drone 1 SITL. Is it running?")
            print("[LEADER]   → Start with: bash scripts/launch_sitl_only.sh (--instance 0)")
            return False

        print(f"[LEADER] Connected SYSID={LEADER_SYSID} ✓")
        super().start()
        return True

    # ── Incoming message handler ─────────────────────────────────────────────

    def on_message(self, msg) -> None:
        mtype = msg.message_type

        if mtype == MessageType.TASK_ACK:
            task_id    = msg.payload.get("task_id", "?")
            ack_status = msg.payload.get("status", "?")
            reason     = msg.payload.get("reason", "")
            self.task_mgr.mark_ack(task_id, ack_status)
            print(f"\n[LEADER] TASK_ACK received for {task_id}: {ack_status}"
                  + (f" — {reason}" if reason else ""))
            if ack_status == "ACCEPTED":
                self.task_mgr.mark_executing(task_id)

        elif mtype == MessageType.TASK_COMPLETE:
            task_id = msg.payload.get("task_id", "?")
            self.task_mgr.mark_complete(task_id)
            print(f"\n[LEADER] ✓ TASK_COMPLETE received for {task_id}")

        else:
            print(f"\n[LEADER] Received unexpected message type: {mtype.value}")

    # ── Manual task creation (called from CLI) ───────────────────────────────

    def issue_task(
        self,
        latitude:  float,
        longitude: float,
        altitude:  float,
        priority:  Priority = Priority.HIGH,
        expiry_s:  float    = 120.0,
    ) -> None:
        """Create and send one TASK message to the Follower (SYSID=2)."""
        task = self.task_mgr.create_task(
            target_id=FOLLOWER_SYSID,
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
            action=TaskAction.INVESTIGATE_AND_LAND,
            priority=priority,
            expiry_seconds=expiry_s,
        )

        print(f"\n[LEADER] Creating TASK {task.task_id}")
        print(f"[LEADER] Target     = {latitude:.7f}, {longitude:.7f}")
        print(f"[LEADER] Altitude   = {altitude:.1f}m")
        print(f"[LEADER] Priority   = {priority.value}")
        print(f"[LEADER] Action     = {task.action.value}")
        print(f"[LEADER] Expiry     = {expiry_s:.0f}s from now")

        msg = task.wrap_in_swarm_message(LEADER_SYSID)
        ok  = self.transport.send(msg)
        if ok:
            self.task_mgr.mark_sent(task.task_id)
            print(f"[LEADER] → TASK {task.task_id} dispatched via LoRa transport")
        else:
            print(f"[LEADER] ✗ TASK {task.task_id} was DROPPED by transport "
                  f"(simulated packet loss)")

    # ── Interactive CLI ──────────────────────────────────────────────────────

    def run_cli(self) -> None:
        """
        Block on interactive readline CLI.
        This is the only way tasks are created — no automatic threats.
        """
        print("\n" + "=" * 60)
        print(" STALLION SWARM V1 — LEADER CLI (SYSID=1)")
        print("=" * 60)
        print(" Type 'help' for available commands.")
        print(" Type 'threat --target LAT,LON --alt ALT' to issue a task.")
        print("=" * 60 + "\n")

        while True:
            try:
                raw = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n[LEADER] Shutting down...")
                self.stop()
                break

            if not raw:
                continue

            parts = raw.split()
            cmd   = parts[0].lower()

            # ── threat ──────────────────────────────────────────────────────
            if cmd == "threat":
                self._cmd_threat(parts[1:])

            # ── status ──────────────────────────────────────────────────────
            elif cmd == "status":
                print("\n── Leader Drone ──")
                print(self.drone_summary())
                print("\n── LoRa Transport ──")
                print(self.transport_summary())

            # ── peers ────────────────────────────────────────────────────────
            elif cmd == "peers":
                print("\n── Swarm Peers ──")
                print(self.peers_summary())

            # ── tasks ────────────────────────────────────────────────────────
            elif cmd == "tasks":
                print("\n── Task Registry ──")
                print(self.task_mgr.summary_table())

            # ── transport / stats ────────────────────────────────────────────
            elif cmd in ("transport", "stats"):
                print("\n── LoRa Transport Statistics ──")
                print(self.transport_summary())

            # ── set ──────────────────────────────────────────────────────────
            elif cmd == "set" and len(parts) >= 3:
                self._cmd_set(parts[1], parts[2])

            # ── help ─────────────────────────────────────────────────────────
            elif cmd == "help":
                print(__doc__)

            # ── quit ─────────────────────────────────────────────────────────
            elif cmd in ("quit", "exit"):
                print("[LEADER] Shutting down...")
                self.stop()
                break

            else:
                print(f"  Unknown command: '{raw}'. Type 'help' for options.")

    # ── CLI sub-command parsers ──────────────────────────────────────────────

    def _cmd_threat(self, args: list) -> None:
        """Parse: threat --target LAT,LON --alt ALT [--priority P] [--expiry S]"""
        lat = lon = alt = None
        priority  = Priority.HIGH
        expiry_s  = get_config().swarm.task_expiry_seconds

        i = 0
        while i < len(args):
            a = args[i].lower()
            if a == "--target" and i + 1 < len(args):
                try:
                    parts = args[i + 1].split(",")
                    lat, lon = float(parts[0]), float(parts[1])
                except Exception:
                    print("  [ERR] --target must be: LAT,LON  (e.g. 11.0168,76.9558)")
                    return
                i += 2
            elif a == "--alt" and i + 1 < len(args):
                try:
                    alt = float(args[i + 1])
                except ValueError:
                    print("  [ERR] --alt must be a number (metres)")
                    return
                i += 2
            elif a == "--priority" and i + 1 < len(args):
                try:
                    priority = Priority(args[i + 1].upper())
                except ValueError:
                    print(f"  [ERR] --priority must be one of: {[p.value for p in Priority]}")
                    return
                i += 2
            elif a == "--expiry" and i + 1 < len(args):
                try:
                    expiry_s = float(args[i + 1])
                except ValueError:
                    print("  [ERR] --expiry must be seconds (number)")
                    return
                i += 2
            else:
                i += 1

        if lat is None or lon is None:
            # Interactive fallback
            try:
                lat = float(input("  Target latitude  : "))
                lon = float(input("  Target longitude : "))
            except (ValueError, EOFError):
                print("  [ERR] Invalid coordinates entered.")
                return

        if alt is None:
            try:
                alt = float(input("  Target altitude (m): "))
            except (ValueError, EOFError):
                print("  [ERR] Invalid altitude entered.")
                return

        self.issue_task(lat, lon, alt, priority, expiry_s)

    def _cmd_set(self, param: str, value: str) -> None:
        """Parse: set loss|latency|jitter VALUE"""
        try:
            val = float(value)
        except ValueError:
            print(f"  [ERR] Value must be a number, got: {value}")
            return

        param = param.lower()
        if param == "loss":
            self.transport.set_loss(val)
        elif param == "latency":
            self.transport.set_latency(val)
        elif param == "jitter":
            self.transport.set_jitter(val)
        else:
            print(f"  [ERR] Unknown parameter '{param}'. Use: loss | latency | jitter")


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    leader = LeaderNode()

    if not leader.start_and_connect(timeout=30):
        sys.exit(1)

    # Give heartbeat a moment to broadcast before CLI starts
    time.sleep(1.0)

    leader.run_cli()


if __name__ == "__main__":
    main()

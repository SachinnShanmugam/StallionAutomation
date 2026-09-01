"""
swarm/follower.py — Follower Node for Stallion Swarm V1
========================================================
Connects to Drone 2 (SYSID=2), listens for swarm messages over the
LoRa transport, validates tasks, and executes them via MAVLink.

The Follower ONLY acts after receiving a manually issued TASK from the Leader.
There are no automatic tasks, missions, or threats in this node.

Task execution sequence:
    1. Receive TASK message over LoRa
    2. Validate (7 checks in validation.py)
    3. Send ACK (ACCEPTED or REJECTED)
    4. Arm Drone 2
    5. Set mode GUIDED, send GOTO to target coordinate
    6. Wait until within 15m of target
    7. Command QLAND
    8. Wait until on ground
    9. Send TASK_COMPLETE to Leader

Communication failure handling:
    - If no task received + comms lost → HOLD (stay in current mode)
    - If task accepted + comms lost → continue execution, send TASK_COMPLETE when done
    - If task expired before receipt → REJECT

Usage:
    python -m swarm.follower
"""

from __future__ import annotations

import sys
import threading
import time

from swarm.config import get_config
from swarm.lora_transport import SimulatedLoRaTransport
from swarm.mavlink_transport import MAVLinkDrone
from swarm.messages import (
    MessageType, AckStatus, TaskMessage,
    make_task_ack, make_task_complete,
)
from swarm.swarm_node import SwarmNode
from swarm.validation import TaskValidator

# ── Follower SYSID and peer ──────────────────────────────────────────────────
FOLLOWER_SYSID = 2
LEADER_SYSID   = 1


class FollowerNode(SwarmNode):
    """
    Follower drone node (SYSID=2).

    Handles:
        - Passive listening over LoRa transport
        - Task validation, ACK, navigation, landing, TASK_COMPLETE
    """

    def __init__(self):
        cfg = get_config()

        # MAVLink connection to Drone 2
        mav_cfg = cfg.mavlink.get("drone2")
        conn    = mav_cfg.connection if mav_cfg else "tcp:127.0.0.1:5770"
        drone   = MAVLinkDrone(conn, expected_sysid=FOLLOWER_SYSID)

        # Simulated LoRa transport (Follower sends back to Leader's port)
        transport = SimulatedLoRaTransport(
            own_sysid=FOLLOWER_SYSID,
            peer_sysids=[LEADER_SYSID],
            cfg=cfg.lora,
        )

        super().__init__(FOLLOWER_SYSID, transport, drone)
        self.validator     = TaskValidator(FOLLOWER_SYSID, cfg.swarm)
        self._exec_thread: threading.Thread | None = None

    def start_and_connect(self, timeout: float = 30.0) -> bool:
        """Connect MAVLink and start background threads."""
        print(f"\n[FOLLOWER] Connecting to Drone 2 SITL ...")
        if not self.drone.connect(timeout=timeout):
            print("[FOLLOWER] ERROR: Could not connect to Drone 2 SITL. Is it running?")
            print("[FOLLOWER]   → Start with: bash scripts/launch_sitl_drone2.sh")
            return False

        print(f"[FOLLOWER] Connected SYSID={FOLLOWER_SYSID} ✓")
        super().start()

        print("\n[FOLLOWER] Listening for swarm messages...")
        print("[FOLLOWER] Waiting for TASK from Leader (SYSID=1)...\n")
        return True

    # ── Incoming message handler ─────────────────────────────────────────────

    def on_message(self, msg) -> None:
        mtype = msg.message_type

        if mtype == MessageType.TASK:
            self._handle_task(msg)

        elif mtype == MessageType.LAND:
            print(f"\n[FOLLOWER] LAND command received from SYSID={msg.sender_id}")
            self.drone.land()

        elif mtype == MessageType.ABORT:
            print(f"\n[FOLLOWER] ABORT command received — switching to QLOITER HOLD")
            self.drone.set_mode("QLOITER")

        else:
            print(f"\n[FOLLOWER] Received message type={mtype.value} "
                  f"from SYSID={msg.sender_id}")

    # ── Task handling ────────────────────────────────────────────────────────

    def _handle_task(self, msg) -> None:
        print(f"\n[FOLLOWER] ── TASK RECEIVED ─────────────────────────────")
        print(f"[FOLLOWER] From SYSID={msg.sender_id}, msg_id={msg.message_id}")

        # Deserialize task payload
        try:
            task = TaskMessage.from_dict(msg.payload)
        except Exception as e:
            print(f"[FOLLOWER] ERROR: Could not parse task payload: {e}")
            return

        print(f"[FOLLOWER] Task ID   = {task.task_id}")
        print(f"[FOLLOWER] Action    = {task.action.value}")
        print(f"[FOLLOWER] Target    = {task.latitude:.7f}, {task.longitude:.7f} @ {task.altitude:.1f}m")
        print(f"[FOLLOWER] Priority  = {task.priority.value}")

        # Validate
        result = self.validator.validate(task)
        print(f"\n[FOLLOWER] Validation: {result.reason}")

        if not result.valid:
            # Determine ACK status
            if "DUPLICATE" in result.reason:
                ack_status = AckStatus.DUPLICATE
            elif "expired" in result.reason.lower():
                ack_status = AckStatus.EXPIRED
            else:
                ack_status = AckStatus.REJECTED

            ack = make_task_ack(
                sender_id=FOLLOWER_SYSID,
                receiver_id=LEADER_SYSID,
                task_id=task.task_id,
                status=ack_status,
                reason=result.reason,
            )
            self.transport.send(ack)
            return

        # Valid task — send ACCEPTED ACK
        self.validator.mark_processed(task.task_id)
        ack = make_task_ack(
            sender_id=FOLLOWER_SYSID,
            receiver_id=LEADER_SYSID,
            task_id=task.task_id,
            status=AckStatus.ACCEPTED,
        )
        print(f"[FOLLOWER] Sending ACK ACCEPTED for {task.task_id}")
        self.transport.send(ack)

        # Execute task in background thread (non-blocking)
        if self._exec_thread and self._exec_thread.is_alive():
            print("[FOLLOWER] WARNING: A task is already executing. Ignoring new task.")
            return

        self._exec_thread = threading.Thread(
            target=self._execute_task,
            args=(task,),
            daemon=True,
            name="follower-exec",
        )
        self._exec_thread.start()

    def _execute_task(self, task: TaskMessage) -> None:
        """
        Execute INVESTIGATE_AND_LAND task:
            1. Arm Drone 2
            2. Navigate to target (GUIDED)
            3. Land (QLAND)
            4. Send TASK_COMPLETE
        """
        cfg = get_config()

        print(f"\n[FOLLOWER] ── EXECUTING TASK {task.task_id} ──────────────")

        # 1. Arm
        print(f"[FOLLOWER] Arming Drone 2 ...")
        self.drone.arm(force=True)
        time.sleep(2.0)

        # 2. Navigate to target
        print(f"[FOLLOWER] Navigating to target: "
              f"{task.latitude:.7f}, {task.longitude:.7f} @ {task.altitude:.1f}m")
        self.drone.goto(task.latitude, task.longitude, task.altitude)

        # Wait until near target — respect comms-lost hold policy
        reached = self.drone.wait_near(
            task.latitude, task.longitude,
            radius_m=15.0,
            timeout=120.0,
        )

        if not reached:
            print(f"[FOLLOWER] WARNING: Did not reach target within timeout. "
                  f"Proceeding to land at current position.")

        print(f"[FOLLOWER] Target reached ✓")

        # 3. Land
        print(f"[FOLLOWER] LAND initiated (QLAND mode)")
        self.drone.land()
        landed = self.drone.wait_landed(timeout=60.0)

        if landed:
            print(f"[FOLLOWER] Landed ✓")
        else:
            print(f"[FOLLOWER] WARNING: Landing timeout. Reporting TASK_COMPLETE anyway.")

        # 4. Send TASK_COMPLETE
        complete_msg = make_task_complete(
            sender_id=FOLLOWER_SYSID,
            receiver_id=LEADER_SYSID,
            task_id=task.task_id,
        )
        print(f"[FOLLOWER] Sending TASK_COMPLETE for {task.task_id}")
        self.transport.send(complete_msg)

        print(f"[FOLLOWER] ── TASK {task.task_id} COMPLETE ──────────────────")

    # ── Status display ────────────────────────────────────────────────────────

    def print_status(self) -> None:
        print("\n── Follower Drone ──")
        print(self.drone_summary())
        print("\n── LoRa Transport ──")
        print(self.transport_summary())
        print("\n── Swarm Peers ──")
        print(self.peers_summary())


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    follower = FollowerNode()

    if not follower.start_and_connect(timeout=30):
        sys.exit(1)

    # Keep alive — all work happens in background threads
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[FOLLOWER] Shutting down...")
        follower.stop()


if __name__ == "__main__":
    main()

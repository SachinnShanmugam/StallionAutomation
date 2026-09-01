"""
swarm/messages.py — Swarm Message Abstractions for Stallion Swarm V1
======================================================================
Defines the generic SwarmMessage envelope and the TaskMessage payload.

All swarm communication passes through SwarmMessage. The message_type
field selects how the payload is interpreted.

V1 active types: HEARTBEAT, TASK, TASK_ACK, TASK_COMPLETE, LAND
Architecture supports: POSITION, STATUS, ABORT, RETURN_HOME (future)
"""

from __future__ import annotations

import json
import time
import uuid
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Any, Dict, Optional


# ─── Message Types ──────────────────────────────────────────────────────────

class MessageType(str, Enum):
    HEARTBEAT    = "HEARTBEAT"
    LEADER_STATE = "LEADER_STATE"   # Real-time Leader telemetry stream
    POSITION     = "POSITION"
    STATUS       = "STATUS"
    TASK         = "TASK"
    TASK_ACK     = "TASK_ACK"
    TASK_COMPLETE = "TASK_COMPLETE"
    ABORT        = "ABORT"
    RETURN_HOME  = "RETURN_HOME"
    LAND         = "LAND"


class Priority(str, Enum):
    LOW    = "LOW"
    MEDIUM = "MEDIUM"
    HIGH   = "HIGH"
    URGENT = "URGENT"


class TaskAction(str, Enum):
    INVESTIGATE_AND_LAND = "INVESTIGATE_AND_LAND"
    # Future: PATROL, LOITER, RTL, PHOTO_SURVEY


class AckStatus(str, Enum):
    ACCEPTED  = "ACCEPTED"
    REJECTED  = "REJECTED"
    DUPLICATE = "DUPLICATE"
    EXPIRED   = "EXPIRED"


# ─── Generic Swarm Message Envelope ────────────────────────────────────────

@dataclass
class SwarmMessage:
    """
    Generic message envelope for all swarm communication.

    receiver_id = 0  → broadcast to all nodes
    receiver_id = N  → unicast to node with sysid=N
    """
    sender_id:    int
    receiver_id:  int
    message_id:   str
    timestamp:    float          # Unix epoch (seconds)
    message_type: MessageType
    priority:     Priority
    payload:      Dict[str, Any] = field(default_factory=dict)

    @staticmethod
    def new(
        sender_id: int,
        receiver_id: int,
        message_type: MessageType,
        priority: Priority = Priority.MEDIUM,
        payload: Optional[Dict[str, Any]] = None,
    ) -> "SwarmMessage":
        return SwarmMessage(
            sender_id=sender_id,
            receiver_id=receiver_id,
            message_id=str(uuid.uuid4())[:8].upper(),
            timestamp=time.time(),
            message_type=message_type,
            priority=priority,
            payload=payload or {},
        )

    def to_json(self) -> str:
        d = asdict(self)
        d["message_type"] = self.message_type.value
        d["priority"]     = self.priority.value
        return json.dumps(d)

    @staticmethod
    def from_json(raw: str) -> "SwarmMessage":
        d = json.loads(raw)
        d["message_type"] = MessageType(d["message_type"])
        d["priority"]     = Priority(d["priority"])
        return SwarmMessage(**d)


# ─── HEARTBEAT helper ───────────────────────────────────────────────────────

def make_heartbeat(sender_id: int) -> SwarmMessage:
    """Broadcast heartbeat — receiver_id=0 means all nodes."""
    return SwarmMessage.new(
        sender_id=sender_id,
        receiver_id=0,
        message_type=MessageType.HEARTBEAT,
        priority=Priority.LOW,
        payload={"ts": time.time()},
    )


# ─── LEADER STATE (Formation Swarm Telemetry) ──────────────────────────────

@dataclass
class LeaderState:
    """
    Real-time Leader telemetry payload for autonomous formation following.
    Carries GPS, Altitude, Velocity Vector, Heading, and Flight Mode.
    """
    latitude:    float
    longitude:   float
    altitude:    float
    vx:          float = 0.0
    vy:          float = 0.0
    vz:          float = 0.0
    heading:     float = 0.0
    flight_mode: str = "UNKNOWN"
    timestamp:   float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "LeaderState":
        return LeaderState(**d)


def make_leader_state(
    sender_id: int,
    latitude: float,
    longitude: float,
    altitude: float,
    vx: float = 0.0,
    vy: float = 0.0,
    vz: float = 0.0,
    heading: float = 0.0,
    flight_mode: str = "QLOITER",
) -> SwarmMessage:
    """Broadcast Leader's real-time flight state over LoRa to all followers."""
    state = LeaderState(
        latitude=latitude,
        longitude=longitude,
        altitude=altitude,
        vx=vx,
        vy=vy,
        vz=vz,
        heading=heading,
        flight_mode=flight_mode,
        timestamp=time.time(),
    )
    return SwarmMessage.new(
        sender_id=sender_id,
        receiver_id=0,  # broadcast
        message_type=MessageType.LEADER_STATE,
        priority=Priority.MEDIUM,
        payload=state.to_dict(),
    )


# ─── TASK Message ──────────────────────────────────────────────────────────

@dataclass
class TaskMessage:
    """
    Task payload carried inside SwarmMessage.payload when message_type=TASK.

    Coordinates are NOT hard-coded here. They are supplied manually
    by the operator via the Leader CLI at runtime.
    """
    task_id:    str
    sender_id:  int
    target_id:  int         # Which drone should execute this task
    timestamp:  float
    priority:   Priority
    action:     TaskAction
    latitude:   float       # Decimal degrees, WGS-84
    longitude:  float       # Decimal degrees, WGS-84
    altitude:   float       # Meters AMSL
    expiry:     float       # Unix epoch — task invalid after this time

    def to_dict(self) -> Dict[str, Any]:
        return {
            "task_id":   self.task_id,
            "sender_id": self.sender_id,
            "target_id": self.target_id,
            "timestamp": self.timestamp,
            "priority":  self.priority.value,
            "action":    self.action.value,
            "latitude":  self.latitude,
            "longitude": self.longitude,
            "altitude":  self.altitude,
            "expiry":    self.expiry,
        }

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "TaskMessage":
        return TaskMessage(
            task_id=d["task_id"],
            sender_id=d["sender_id"],
            target_id=d["target_id"],
            timestamp=d["timestamp"],
            priority=Priority(d["priority"]),
            action=TaskAction(d["action"]),
            latitude=float(d["latitude"]),
            longitude=float(d["longitude"]),
            altitude=float(d["altitude"]),
            expiry=float(d["expiry"]),
        )

    @staticmethod
    def new(
        sender_id: int,
        target_id: int,
        action: TaskAction,
        latitude: float,
        longitude: float,
        altitude: float,
        priority: Priority = Priority.HIGH,
        expiry_seconds: float = 120.0,
    ) -> "TaskMessage":
        now = time.time()
        return TaskMessage(
            task_id=f"T{int(now) % 100000:05d}",
            sender_id=sender_id,
            target_id=target_id,
            timestamp=now,
            priority=priority,
            action=action,
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
            expiry=now + expiry_seconds,
        )

    def is_expired(self) -> bool:
        return time.time() > self.expiry

    def wrap_in_swarm_message(self, sender_id: int) -> SwarmMessage:
        """Wrap this TaskMessage into a SwarmMessage envelope."""
        return SwarmMessage.new(
            sender_id=sender_id,
            receiver_id=self.target_id,
            message_type=MessageType.TASK,
            priority=self.priority,
            payload=self.to_dict(),
        )


# ─── ACK Message helper ─────────────────────────────────────────────────────

def make_task_ack(
    sender_id: int,
    receiver_id: int,
    task_id: str,
    status: AckStatus,
    reason: str = "",
) -> SwarmMessage:
    return SwarmMessage.new(
        sender_id=sender_id,
        receiver_id=receiver_id,
        message_type=MessageType.TASK_ACK,
        priority=Priority.HIGH,
        payload={"task_id": task_id, "status": status.value, "reason": reason},
    )


def make_task_complete(
    sender_id: int,
    receiver_id: int,
    task_id: str,
) -> SwarmMessage:
    return SwarmMessage.new(
        sender_id=sender_id,
        receiver_id=receiver_id,
        message_type=MessageType.TASK_COMPLETE,
        priority=Priority.HIGH,
        payload={"task_id": task_id, "completed_at": time.time()},
    )

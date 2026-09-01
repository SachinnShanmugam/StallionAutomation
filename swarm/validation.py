"""
swarm/validation.py — Task Validator for Stallion Swarm V1
===========================================================
The Follower calls validate_task() before executing any received task.

Validation checks (all must pass):
    1. sender_id is in authorized_leaders list
    2. target_id matches our own sysid (or is a broadcast task)
    3. task_id has not been processed before (duplicate guard)
    4. Coordinates are within valid WGS-84 bounds
    5. Altitude is within safe operational bounds
    6. Timestamp is not from the future (clock skew check)
    7. Task has not expired (expiry field check)
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Set

from swarm.messages import TaskMessage
from swarm.config import SwarmConfig


@dataclass
class ValidationResult:
    valid:  bool
    reason: str


class TaskValidator:
    """
    Stateful validator that tracks processed task IDs to prevent duplicates.

    Instantiate once per follower node and reuse across all received tasks.
    """

    # Safety bounds for coordinates
    MIN_LAT = -90.0
    MAX_LAT = +90.0
    MIN_LON = -180.0
    MAX_LON = +180.0
    MIN_ALT = 0.0
    MAX_ALT = 500.0   # metres — adjust for operational ceiling

    # Maximum allowed clock skew between sender and receiver
    MAX_FUTURE_SKEW_SEC = 10.0

    def __init__(self, own_sysid: int, cfg: SwarmConfig):
        self.own_sysid   = own_sysid
        self.cfg         = cfg
        self._processed: Set[str] = set()

    def validate(self, task: TaskMessage) -> ValidationResult:
        """
        Run all validation checks on a received TaskMessage.
        Returns ValidationResult(valid=True) if all checks pass.
        """

        # 1. Authorized sender
        if task.sender_id not in self.cfg.authorized_leaders:
            return ValidationResult(
                False,
                f"REJECTED: sender_id={task.sender_id} not in authorized_leaders "
                f"{self.cfg.authorized_leaders}"
            )

        # 2. Target ID valid (unicast must match us, or broadcast=0)
        if task.target_id not in (self.own_sysid, 0):
            return ValidationResult(
                False,
                f"REJECTED: task target_id={task.target_id} does not match "
                f"our sysid={self.own_sysid}"
            )

        # 3. Duplicate check
        if task.task_id in self._processed:
            return ValidationResult(
                False,
                f"DUPLICATE: task_id={task.task_id} already processed"
            )

        # 4. Coordinate bounds
        if not (self.MIN_LAT <= task.latitude <= self.MAX_LAT):
            return ValidationResult(
                False,
                f"REJECTED: latitude={task.latitude} out of bounds "
                f"[{self.MIN_LAT}, {self.MAX_LAT}]"
            )
        if not (self.MIN_LON <= task.longitude <= self.MAX_LON):
            return ValidationResult(
                False,
                f"REJECTED: longitude={task.longitude} out of bounds "
                f"[{self.MIN_LON}, {self.MAX_LON}]"
            )

        # 5. Altitude bounds
        if not (self.MIN_ALT <= task.altitude <= self.MAX_ALT):
            return ValidationResult(
                False,
                f"REJECTED: altitude={task.altitude}m out of safe bounds "
                f"[{self.MIN_ALT}, {self.MAX_ALT}]"
            )

        # 6. Timestamp not from the future (clock skew)
        now = time.time()
        if task.timestamp > now + self.MAX_FUTURE_SKEW_SEC:
            return ValidationResult(
                False,
                f"REJECTED: task timestamp is {task.timestamp - now:.1f}s in the "
                f"future (clock skew limit={self.MAX_FUTURE_SKEW_SEC}s)"
            )

        # 7. Expiry check
        if task.is_expired():
            age = now - task.expiry
            return ValidationResult(
                False,
                f"REJECTED: task_id={task.task_id} expired {age:.1f}s ago"
            )

        return ValidationResult(True, "VALID")

    def mark_processed(self, task_id: str) -> None:
        """Record a task_id as processed so duplicates are rejected."""
        self._processed.add(task_id)

    def is_processed(self, task_id: str) -> bool:
        return task_id in self._processed

"""
swarm/task_manager.py — Task Lifecycle Manager for Stallion Swarm V1
=====================================================================
Tracks all tasks created by the Leader and their current state.

Task lifecycle:
    CREATED → SENT → ACK_PENDING → ACKNOWLEDGED → EXECUTING → COMPLETE
                                                            ↘ FAILED
                                                            ↘ EXPIRED
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional

from swarm.messages import TaskMessage, Priority, TaskAction
from swarm.config import SwarmConfig


class TaskStatus(str, Enum):
    CREATED      = "CREATED"
    SENT         = "SENT"
    ACK_PENDING  = "ACK_PENDING"
    ACKNOWLEDGED = "ACKNOWLEDGED"
    EXECUTING    = "EXECUTING"
    COMPLETE     = "COMPLETE"
    FAILED       = "FAILED"
    EXPIRED      = "EXPIRED"


@dataclass
class TaskRecord:
    task:       TaskMessage
    status:     TaskStatus = TaskStatus.CREATED
    sent_at:    Optional[float] = None
    acked_at:   Optional[float] = None
    done_at:    Optional[float] = None
    ack_status: str = ""
    retries:    int = 0


class TaskManager:
    """
    Creates, tracks, and expires swarm tasks for the Leader node.

    The Leader creates tasks via create_task() and passes the
    SwarmMessage to the transport for delivery. The task_manager
    then tracks ACK and TASK_COMPLETE responses.
    """

    def __init__(self, own_sysid: int, cfg: SwarmConfig):
        self.own_sysid = own_sysid
        self.cfg       = cfg
        self._tasks: Dict[str, TaskRecord] = {}

    def create_task(
        self,
        target_id: int,
        latitude:  float,
        longitude: float,
        altitude:  float,
        action:    TaskAction = TaskAction.INVESTIGATE_AND_LAND,
        priority:  Priority   = Priority.HIGH,
        expiry_seconds: float = 120.0,
    ) -> TaskMessage:
        """
        Create a new TaskMessage. Does NOT send it — that's the Leader's job.
        Coordinates are supplied by the operator at runtime, never hard-coded.
        """
        task = TaskMessage.new(
            sender_id=self.own_sysid,
            target_id=target_id,
            action=action,
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
            priority=priority,
            expiry_seconds=expiry_seconds,
        )
        record = TaskRecord(task=task)
        self._tasks[task.task_id] = record
        return task

    def mark_sent(self, task_id: str) -> None:
        rec = self._tasks.get(task_id)
        if rec:
            rec.status  = TaskStatus.SENT
            rec.sent_at = time.time()

    def mark_ack(self, task_id: str, ack_status: str) -> None:
        rec = self._tasks.get(task_id)
        if rec:
            rec.status     = TaskStatus.ACKNOWLEDGED
            rec.acked_at   = time.time()
            rec.ack_status = ack_status

    def mark_executing(self, task_id: str) -> None:
        rec = self._tasks.get(task_id)
        if rec:
            rec.status = TaskStatus.EXECUTING

    def mark_complete(self, task_id: str) -> None:
        rec = self._tasks.get(task_id)
        if rec:
            rec.status  = TaskStatus.COMPLETE
            rec.done_at = time.time()

    def mark_failed(self, task_id: str) -> None:
        rec = self._tasks.get(task_id)
        if rec:
            rec.status = TaskStatus.FAILED

    def get(self, task_id: str) -> Optional[TaskRecord]:
        return self._tasks.get(task_id)

    def all_tasks(self) -> List[TaskRecord]:
        return list(self._tasks.values())

    def expire_stale(self) -> List[str]:
        """Check for expired tasks and update their status. Returns list of expired IDs."""
        expired = []
        for tid, rec in self._tasks.items():
            if rec.task.is_expired() and rec.status not in (
                TaskStatus.COMPLETE, TaskStatus.FAILED, TaskStatus.EXPIRED
            ):
                rec.status = TaskStatus.EXPIRED
                expired.append(tid)
        return expired

    def is_ack_timeout(self, task_id: str) -> bool:
        """True if a sent task has not received an ACK within ack_timeout_seconds."""
        rec = self._tasks.get(task_id)
        if not rec or rec.sent_at is None:
            return False
        if rec.status != TaskStatus.SENT:
            return False
        return (time.time() - rec.sent_at) > self.cfg.ack_timeout_seconds

    def summary_table(self) -> str:
        """Return a formatted text table of all tasks for the > tasks command."""
        if not self._tasks:
            return "  No tasks created yet."
        lines = [f"  {'TASK_ID':<12} {'STATUS':<14} {'TARGET':<8} {'ACTION':<22} {'PRIORITY':<8}"]
        lines.append("  " + "-" * 70)
        for rec in self._tasks.values():
            t = rec.task
            lines.append(
                f"  {t.task_id:<12} {rec.status.value:<14} {t.target_id:<8} "
                f"{t.action.value:<22} {t.priority.value:<8}"
            )
        return "\n".join(lines)

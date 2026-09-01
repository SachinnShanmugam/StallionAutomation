"""
swarm/transport.py — Abstract Transport Interface for Stallion Swarm V1
========================================================================
The swarm application layer communicates exclusively through BaseTransport.
The concrete transport (simulated LoRa, real LoRa, mesh radio) is injected
at startup. Swapping transports requires zero changes to leader/follower logic.

Implementing a real LoRa transport:
    1. Create a class that inherits BaseTransport
    2. Implement send() and recv()
    3. Pass it to LeaderNode / FollowerNode instead of SimulatedLoRaTransport

The interface is intentionally minimal and synchronous-recv friendly so it
can wrap both UDP sockets and serial ports without abstraction leakage.
"""

from __future__ import annotations

import abc
from typing import Optional

from swarm.messages import SwarmMessage


class BaseTransport(abc.ABC):
    """
    Abstract transport interface.

    All swarm messages pass through send() and recv().
    Implementations must be thread-safe for send().
    recv() is called from the node's message-pump loop.
    """

    @abc.abstractmethod
    def send(self, msg: SwarmMessage) -> bool:
        """
        Serialize and transmit a SwarmMessage.

        Returns True if the message was accepted for transmission.
        Does NOT guarantee delivery (LoRa is fire-and-forget).
        """

    @abc.abstractmethod
    def recv(self, timeout: float = 0.1) -> Optional[SwarmMessage]:
        """
        Receive and deserialize one SwarmMessage.

        Returns None if no message arrives within timeout seconds.
        Non-blocking when timeout=0.
        """

    @abc.abstractmethod
    def get_stats(self) -> dict:
        """
        Return transport diagnostics.

        Must include at minimum:
            tx_count    : int   — packets transmitted
            rx_count    : int   — packets received
            drop_count  : int   — packets deliberately dropped (loss simulation)
            avg_latency : float — average one-way delivery latency (ms)
        """

    @abc.abstractmethod
    def close(self) -> None:
        """Release transport resources (close sockets, serial port, etc.)."""

    # ── Optional hot-parameter update (implemented by SimulatedLoRa) ─────────

    def set_loss(self, percent: float) -> None:
        """Override packet loss percent at runtime. No-op if not supported."""

    def set_latency(self, ms: float) -> None:
        """Override latency (ms) at runtime. No-op if not supported."""

    def set_jitter(self, ms: float) -> None:
        """Override jitter (ms) at runtime. No-op if not supported."""

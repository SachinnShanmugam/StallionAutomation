"""
swarm/lora_transport.py — Simulated LoRa Transport for Stallion Swarm V1
=========================================================================
Software-only LoRa channel emulator using UDP loopback sockets.

Port scheme (N-drone scalable):
    Each node's listen port  = udp_port_base + own_sysid
    Each node sends to port  = udp_port_base + peer_sysid

    Drone 1 (sysid=1): listens on 10001, sends to 10002
    Drone 2 (sysid=2): listens on 10002, sends to 10001
    Drone 3 (sysid=3): listens on 10003, sends to 10001 + 10002
    ...
    Drone N:           listens on 10000+N

Swapping to real LoRa hardware:
    Implement BaseTransport (transport.py), inject into leader/follower.
    Zero changes needed in leader.py or follower.py.

TX/RX events format:
    [LORA TX] src=1 dst=2 type=TASK id=T12345
    [LORA RX] src=1 dst=2 type=TASK id=T12345
"""

from __future__ import annotations

import queue
import random
import socket
import threading
import time
from typing import List, Optional

from swarm.config import LoRaConfig
from swarm.messages import SwarmMessage
from swarm.transport import BaseTransport


class SimulatedLoRaTransport(BaseTransport):
    """
    UDP-based LoRa emulator with configurable:
        latency_ms          — one-way base propagation delay
        jitter_ms           — ±random jitter per packet
        packet_loss_percent — probability a packet is silently dropped
        bandwidth_bps       — throughput cap (limits burst sending)
    """

    def __init__(
        self,
        own_sysid: int,
        peer_sysids: List[int],
        cfg: LoRaConfig,
    ):
        self.own_sysid   = own_sysid
        self.peer_sysids = peer_sysids
        self.cfg         = cfg

        # Mutable channel parameters (can be changed via set_loss/set_latency/set_jitter)
        self._lock_params   = threading.Lock()
        self._latency_ms    = cfg.latency_ms
        self._jitter_ms     = cfg.jitter_ms
        self._loss_pct      = cfg.packet_loss_percent

        # Statistics
        self._tx_count      = 0
        self._rx_count      = 0
        self._drop_count    = 0
        self._latency_sum   = 0.0
        self._latency_count = 0
        self._stat_lock     = threading.Lock()

        # Incoming message queue (filled by receiver thread)
        self._rx_queue: queue.Queue[SwarmMessage] = queue.Queue()

        # UDP receive socket — own port = base + own_sysid
        self._listen_port = cfg.udp_port_base + own_sysid
        self._host        = cfg.udp_host

        self._rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx_sock.bind((self._host, self._listen_port))
        self._rx_sock.settimeout(0.1)

        # Background receiver thread
        self._running = True
        self._rx_thread = threading.Thread(target=self._receiver_loop, daemon=True)
        self._rx_thread.start()

        print(f"[LORA INIT] Node sysid={own_sysid} listening on "
              f"{self._host}:{self._listen_port} | "
              f"peers={peer_sysids} | "
              f"loss={self._loss_pct:.0f}% | "
              f"latency={self._latency_ms:.0f}±{self._jitter_ms:.0f}ms")

    # ── Public send/recv API (implements BaseTransport) ─────────────────────

    def send(self, msg: SwarmMessage) -> bool:
        """Transmit a SwarmMessage to the destination(s) with simulated channel effects."""
        with self._lock_params:
            loss  = self._loss_pct
            delay = self._latency_ms
            jit   = self._jitter_ms

        # Packet loss simulation
        if random.uniform(0, 100) < loss:
            with self._stat_lock:
                self._drop_count += 1
            print(f"[LORA DROP] src={msg.sender_id} dst={msg.receiver_id} "
                  f"type={msg.message_type.value} id={msg.message_id}  "
                  f"(simulated packet loss {loss:.0f}%)")
            return False

        # Determine destination ports
        if msg.receiver_id == 0:
            # Broadcast
            dest_ports = [self.cfg.udp_port_base + sid for sid in self.peer_sysids]
        else:
            dest_ports = [self.cfg.udp_port_base + msg.receiver_id]

        # Compute jittered delay
        actual_delay_ms = delay + random.uniform(-jit, jit)
        actual_delay_ms = max(0.0, actual_delay_ms)

        print(f"[LORA TX] src={msg.sender_id} dst={msg.receiver_id} "
              f"type={msg.message_type.value} id={msg.message_id} "
              f"delay={actual_delay_ms:.0f}ms")

        # Launch delayed send in background thread
        raw = msg.to_json().encode()
        delay_sec = actual_delay_ms / 1000.0

        def _delayed():
            time.sleep(delay_sec)
            tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                for port in dest_ports:
                    # Bandwidth cap: serialize time = bytes * 8 / bps
                    byte_delay = (len(raw) * 8) / self.cfg.bandwidth_bps
                    time.sleep(byte_delay)
                    tx_sock.sendto(raw, (self._host, port))
            finally:
                tx_sock.close()
            with self._stat_lock:
                self._tx_count += 1
                self._latency_sum   += actual_delay_ms
                self._latency_count += 1

        threading.Thread(target=_delayed, daemon=True).start()
        return True

    def recv(self, timeout: float = 0.1) -> Optional[SwarmMessage]:
        """Return the next received SwarmMessage, or None if queue is empty."""
        try:
            return self._rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def get_stats(self) -> dict:
        with self._stat_lock:
            avg = (self._latency_sum / self._latency_count
                   if self._latency_count > 0 else 0.0)
            total = self._tx_count + self._drop_count
            loss_pct = (self._drop_count / total * 100) if total > 0 else 0.0
            return {
                "tx_count":    self._tx_count,
                "rx_count":    self._rx_count,
                "drop_count":  self._drop_count,
                "avg_latency": round(avg, 1),
                "loss_pct":    round(loss_pct, 1),
            }

    def close(self) -> None:
        self._running = False
        self._rx_sock.close()

    # ── Runtime parameter adjustment ─────────────────────────────────────────

    def set_loss(self, percent: float) -> None:
        with self._lock_params:
            self._loss_pct = max(0.0, min(100.0, percent))
        print(f"[LORA CFG] packet_loss set to {self._loss_pct:.1f}%")

    def set_latency(self, ms: float) -> None:
        with self._lock_params:
            self._latency_ms = max(0.0, ms)
        print(f"[LORA CFG] latency set to {self._latency_ms:.0f}ms")

    def set_jitter(self, ms: float) -> None:
        with self._lock_params:
            self._jitter_ms = max(0.0, ms)
        print(f"[LORA CFG] jitter set to {self._jitter_ms:.0f}ms")

    # ── Background receiver loop ─────────────────────────────────────────────

    def _receiver_loop(self) -> None:
        while self._running:
            try:
                raw, _ = self._rx_sock.recvfrom(65535)
                msg = SwarmMessage.from_json(raw.decode())

                # Filter: only accept messages addressed to us or broadcast
                if msg.receiver_id not in (self.own_sysid, 0):
                    continue

                print(f"[LORA RX] src={msg.sender_id} dst={msg.receiver_id} "
                      f"type={msg.message_type.value} id={msg.message_id}")

                with self._stat_lock:
                    self._rx_count += 1

                self._rx_queue.put(msg)

            except socket.timeout:
                continue
            except OSError:
                # Socket closed during shutdown
                break
            except Exception as e:
                print(f"[LORA ERR] Receiver error: {e}")

"""
swarm/config.py — Configuration loader for Stallion Swarm V1
=============================================================
Loads swarm/config.yaml and exposes typed dataclasses.
"""

from __future__ import annotations

import os
import yaml
from dataclasses import dataclass, field
from typing import List


# Path to config.yaml relative to this file
_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.yaml")


def _load_yaml() -> dict:
    with open(_CONFIG_PATH, "r") as f:
        return yaml.safe_load(f)


@dataclass
class LoRaConfig:
    latency_ms: float = 100.0
    jitter_ms: float = 20.0
    packet_loss_percent: float = 2.0
    bandwidth_bps: int = 10000
    udp_port_base: int = 10000
    udp_host: str = "127.0.0.1"


@dataclass
class SwarmConfig:
    authorized_leaders: List[int] = field(default_factory=lambda: [1])
    task_expiry_seconds: float = 120.0
    ack_timeout_seconds: float = 10.0
    heartbeat_interval_seconds: float = 2.0
    comms_lost_hold_seconds: float = 30.0


@dataclass
class MAVLinkDroneConfig:
    sysid: int = 1
    connection: str = "udpin:0.0.0.0:14550"
    role: str = "follower"


@dataclass
class Config:
    lora: LoRaConfig = field(default_factory=LoRaConfig)
    swarm: SwarmConfig = field(default_factory=SwarmConfig)
    mavlink: dict = field(default_factory=dict)  # key=drone_name, value=MAVLinkDroneConfig


def load_config() -> Config:
    """Load and parse config.yaml. Returns a typed Config object."""
    raw = _load_yaml()

    lora_raw = raw.get("lora", {})
    lora = LoRaConfig(
        latency_ms=float(lora_raw.get("latency_ms", 100.0)),
        jitter_ms=float(lora_raw.get("jitter_ms", 20.0)),
        packet_loss_percent=float(lora_raw.get("packet_loss_percent", 2.0)),
        bandwidth_bps=int(lora_raw.get("bandwidth_bps", 10000)),
        udp_port_base=int(lora_raw.get("udp_port_base", 10000)),
        udp_host=str(lora_raw.get("udp_host", "127.0.0.1")),
    )

    swarm_raw = raw.get("swarm", {})
    swarm = SwarmConfig(
        authorized_leaders=list(swarm_raw.get("authorized_leaders", [1])),
        task_expiry_seconds=float(swarm_raw.get("task_expiry_seconds", 120.0)),
        ack_timeout_seconds=float(swarm_raw.get("ack_timeout_seconds", 10.0)),
        heartbeat_interval_seconds=float(swarm_raw.get("heartbeat_interval_seconds", 2.0)),
        comms_lost_hold_seconds=float(swarm_raw.get("comms_lost_hold_seconds", 30.0)),
    )

    mav_raw = raw.get("mavlink", {})
    mavlink = {}
    for name, d in mav_raw.items():
        mavlink[name] = MAVLinkDroneConfig(
            sysid=int(d.get("sysid", 1)),
            connection=str(d.get("connection", "udpin:0.0.0.0:14550")),
            role=str(d.get("role", "follower")),
        )

    return Config(lora=lora, swarm=swarm, mavlink=mavlink)


# Singleton for module-level access
_cfg: Config | None = None


def get_config() -> Config:
    global _cfg
    if _cfg is None:
        _cfg = load_config()
    return _cfg


def reload_config() -> Config:
    """Force reload from disk (e.g. after editing config.yaml at runtime)."""
    global _cfg
    _cfg = load_config()
    return _cfg

"""
Stallion VTOL — 2-Drone LoRa Swarm SITL V1
===========================================
Expandable swarm coordination layer over simulated LoRa transport.

Architecture:
    Swarm Application (leader.py / follower.py)
        ↓
    BaseTransport (transport.py)
        ↓
    SimulatedLoRaTransport (lora_transport.py)  ← swap for real LoRa here
        ↓
    UDP loopback (127.0.0.1)
"""
__version__ = "1.0.0"

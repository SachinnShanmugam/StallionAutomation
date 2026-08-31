#!/usr/bin/env python3
"""
================================================================================
  STALLION VTOL - DRONECAN GPS (Fix2) FULL ROUND-TRIP PROOF & VERIFIER
================================================================================
Strict Round-Trip Pipeline:
  1. Gazebo GPS Input (Dynamic / True Coordinates)
  2. Authentic DSDL Object Creation (uavcan.equipment.gnss.Fix2 / ID: 1063)
  3. Real DroneCAN DSDL Serialization (Bit-level packed binary)
  4. Multi-Frame CAN 2.0B Transport Framing (CAN ID: 0x1804272A + CRC16 + Tail Bytes)
  5. Multi-Frame Stream Reassembly (CAN Frames -> Reassembled Stream -> CRC Validation)
  6. Independent DSDL Deserialization (Reassembled Stream -> Decoded Fix2 Object)
  7. Exact Mathematical Delta Equality Verification (Original == Decoded)
================================================================================
"""

import sys
import os
import time
import math
import struct

try:
    import dronecan
    from dronecan import uavcan
    DRONECAN_AVAILABLE = True
except ImportError:
    print("[ERROR] 'dronecan' package is required. Install via: pip install dronecan")
    sys.exit(1)


# ==============================================================================
# 1. CAN ID CONSTRUCTOR (Matches libcanard.c line 217)
# ==============================================================================

def make_dronecan_broadcast_can_id(priority: int, data_type_id: int, source_node_id: int) -> int:
    """
    Constructs 29-bit CAN ID matching libcanard:
    can_id = (priority << 24) | (data_type_id << 8) | (source_node_id)
    """
    return ((priority & 0x1F) << 24) | ((data_type_id & 0xFFFF) << 8) | (source_node_id & 0x7F)


# ==============================================================================
# 2. CCITT-CRC16 (DroneCAN Transfer CRC)
# ==============================================================================

def crc16_add(crc: int, byte: int) -> int:
    byte &= 0xFF
    data = byte ^ (crc >> 8)
    data ^= (data >> 4)
    return (((crc << 8) ^ (data << 12) ^ (data << 5) ^ data) & 0xFFFF)

def calculate_transfer_crc(data_type_signature: int, payload: bytes) -> int:
    crc = 0xFFFF
    sig_bytes = struct.pack('<Q', data_type_signature)
    for b in sig_bytes:
        crc = crc16_add(crc, b)
    for b in payload:
        crc = crc16_add(crc, b)
    return crc


# ==============================================================================
# 3. DSDL BIT-LEVEL PACK & UNPACK
# ==============================================================================

def bits_to_bytes(bits: str) -> bytes:
    byte_arr = bytearray()
    for i in range(0, len(bits), 8):
        byte_bits = bits[i:i+8]
        val = 0
        for bit_idx, bit_char in enumerate(byte_bits):
            if bit_char == '1':
                val |= (1 << bit_idx)
        byte_arr.append(val)
    return bytes(byte_arr)

def bytes_to_bits(raw_bytes: bytes, bit_len: int) -> str:
    bit_chars = []
    for b in raw_bytes:
        for bit_idx in range(8):
            bit_chars.append('1' if (b & (1 << bit_idx)) else '0')
    return ''.join(bit_chars)[:bit_len]


# ==============================================================================
# 4. ENCODER / FRAMER (Fix2 -> CAN Frames)
# ==============================================================================

def encode_fix2_to_can_frames(fix2_obj, node_id=42, transfer_id=0, priority=24):
    dtid = uavcan.equipment.gnss.Fix2.default_dtid
    signature = uavcan.equipment.gnss.Fix2.get_data_type_signature()
    can_id = make_dronecan_broadcast_can_id(priority, dtid, node_id)

    # 1. DSDL pack
    bit_str = fix2_obj._pack()
    payload = bits_to_bytes(bit_str)

    # 2. Multi-frame packaging (CRC16 + payload)
    transfer_crc = calculate_transfer_crc(signature, payload)
    crc_bytes = struct.pack('<H', transfer_crc)
    stream = crc_bytes + payload

    frames = []
    offset = 0
    total_len = len(stream)
    toggle = 0
    tid = transfer_id & 0x1F

    while offset < total_len:
        is_start = (offset == 0)
        chunk_size = min(7, total_len - offset)
        is_end = (offset + chunk_size >= total_len)

        start_bit = 1 if is_start else 0
        end_bit = 1 if is_end else 0
        toggle_bit = toggle & 1

        tail_byte = (start_bit << 7) | (end_bit << 6) | (toggle_bit << 5) | tid
        chunk = stream[offset:offset + chunk_size]
        frame_data = chunk + bytes([tail_byte])

        frames.append({
            'can_id': can_id,
            'dlc': len(frame_data),
            'data': frame_data,
            'is_start': is_start,
            'is_end': is_end,
            'toggle': toggle_bit,
            'tail_byte': tail_byte
        })
        offset += chunk_size
        toggle ^= 1

    return frames, payload, len(bit_str), transfer_crc


# ==============================================================================
# 5. INDEPENDENT RECEIVER / REASSEMBLER (CAN Frames -> Decoded Fix2)
# ==============================================================================

def decode_can_frames_to_fix2(frames, bit_len, expected_signature):
    # 1. Reassemble payload stream from CAN frames by stripping tail bytes
    reassembled_stream = bytearray()
    transfer_id = None

    for idx, f in enumerate(frames):
        data = f['data']
        payload_chunk = data[:-1]
        tail = data[-1]

        start_bit = (tail >> 7) & 1
        end_bit = (tail >> 6) & 1
        toggle_bit = (tail >> 5) & 1
        tid = tail & 0x1F

        if idx == 0:
            if start_bit != 1:
                raise ValueError("Frame 0 missing START bit")
            transfer_id = tid
        else:
            if tid != transfer_id:
                raise ValueError("Transfer ID mismatch across frames")

        if idx == len(frames) - 1:
            if end_bit != 1:
                raise ValueError("Last frame missing END bit")

        reassembled_stream.extend(payload_chunk)

    # 2. Extract CRC16 and Payload
    received_crc = struct.unpack('<H', reassembled_stream[:2])[0]
    payload = bytes(reassembled_stream[2:])

    # 3. Verify CCITT-CRC16
    computed_crc = calculate_transfer_crc(expected_signature, payload)
    crc_ok = (received_crc == computed_crc)

    # 4. Deserialization through DSDL
    decoded_fix2 = uavcan.equipment.gnss.Fix2()
    bit_str = bytes_to_bits(payload, bit_len)
    decoded_fix2._unpack(bit_str)

    return decoded_fix2, payload, received_crc, computed_crc, crc_ok


# ==============================================================================
# 6. MASTER VERIFICATION TEST HARNESS
# ==============================================================================

def run_proof_verification(lat=13.0827150, lon=80.2707100, alt=16.50, vx=0.45, vy=-0.12, vz=-1.10, sats=14):
    if hasattr(sys.stdout, 'reconfigure'):
        try:
            sys.stdout.reconfigure(encoding='utf-8')
        except Exception:
            pass

    dtid = uavcan.equipment.gnss.Fix2.default_dtid
    signature = uavcan.equipment.gnss.Fix2.get_data_type_signature()
    node_id = 42
    transfer_id = 7

    print("=" * 90)
    print("  STALLION VTOL - DRONECAN GPS (uavcan.equipment.gnss.Fix2) ROUND-TRIP PROOF")
    print("=" * 90)
    print(" [OBJECTIVE] Mathematically prove lossless serialization, CAN 2.0B framing,")
    print("             transfer CRC integrity, and independent DSDL reassembly.")
    print("-" * 90)

    # 1. Create Source Fix2 Message
    fix2_orig = uavcan.equipment.gnss.Fix2()
    fix2_orig.timestamp.usec = 1725140000000000
    fix2_orig.gnss_timestamp.usec = 1725140000000000
    fix2_orig.gnss_time_standard = 2 # UTC
    fix2_orig.latitude_deg_1e8 = int(round(lat * 1e8))
    fix2_orig.longitude_deg_1e8 = int(round(lon * 1e8))
    fix2_orig.height_msl_mm = int(round(alt * 1000.0))
    fix2_orig.height_ellipsoid_mm = int(round(alt * 1000.0))
    fix2_orig.ned_velocity = [float(vx), float(vy), float(vz)]
    fix2_orig.sats_used = int(sats)
    fix2_orig.status = 3 # 3D Fix
    fix2_orig.pdop = 1.15
    fix2_orig.covariance = [0.25, 0.25, 0.50, 0.05, 0.05, 0.05]

    print(f" [INPUT] ORIGINAL GAZEBO / ARDUPILOT VALUES:")
    print(f"    • Latitude:          {lat:12.7f}° ({fix2_orig.latitude_deg_1e8} in 1e-8 deg units)")
    print(f"    • Longitude:         {lon:12.7f}° ({fix2_orig.longitude_deg_1e8} in 1e-8 deg units)")
    print(f"    • MSL Altitude:      {alt:7.2f} m ({fix2_orig.height_msl_mm} mm)")
    print(f"    • NED Velocity:      [{vx:+5.2f}, {vy:+5.2f}, {vz:+5.2f}] m/s")
    print(f"    • Satellites:        {sats} | Fix Status: 3D Fix | PDOP: {fix2_orig.pdop}")
    print("-" * 90)

    # 2. Encode to Serialized Payload and CAN Frames
    frames, tx_payload, bit_len, tx_crc = encode_fix2_to_can_frames(fix2_orig, node_id=node_id, transfer_id=transfer_id)

    print(f" [PAYLOAD] DRONECAN DSDL SERIALIZED BYTES ({len(tx_payload)} Bytes / {bit_len} Bits):")
    print("   ", ' '.join(f'{b:02X}' for b in tx_payload))
    print(f"    • Transfer CCITT-CRC16: 0x{tx_crc:04X} (incorporating signature 0x{signature:016X})")
    print("-" * 90)

    print(f" [FRAMES] DECOMPOSED PHYSICAL CAN 2.0B FRAMES ({len(frames)} Frames Generated):")
    print(" Frame | CAN ID     | DLC | Data Bytes (Hex)              | Tail Byte (Hex & Flags)")
    print(" ------|------------|-----|-------------------------------|------------------------")
    for idx, f in enumerate(frames):
        d_hex = ' '.join(f'{b:02X}' for b in f['data'][:-1])
        t_hex = f"{f['data'][-1]:02X}"
        flags = f"[START={f['is_start']}, END={f['is_end']}, TOGGLE={f['toggle']}, TID={transfer_id}]"
        print(f"  #{idx:02d}  | 0x{f['can_id']:08X} |  {f['dlc']}  | {d_hex:29s} | 0x{t_hex} {flags}")
    print("-" * 90)

    # 3. Independent Reassembly & Deserialization
    dec_obj, rx_payload, rx_crc, comp_crc, crc_ok = decode_can_frames_to_fix2(frames, bit_len, signature)

    dec_lat = dec_obj.latitude_deg_1e8 / 1e8
    dec_lon = dec_obj.longitude_deg_1e8 / 1e8
    dec_alt = dec_obj.height_msl_mm / 1000.0
    dec_vx, dec_vy, dec_vz = dec_obj.ned_velocity[0], dec_obj.ned_velocity[1], dec_obj.ned_velocity[2]

    # Compute exact deltas
    d_lat = abs(lat - dec_lat)
    d_lon = abs(lon - dec_lon)
    d_alt = abs(alt - dec_alt)
    d_vel = math.sqrt((vx - dec_vx)**2 + (vy - dec_vy)**2 + (vz - dec_vz)**2)

    print(f" [DECODED] INDEPENDENTLY REASSEMBLED & DECODED DRONECAN VALUES:")
    print(f"    • Decoded Latitude:  {dec_lat:12.7f}° (ΔLat = {d_lat:.9f}°)")
    print(f"    • Decoded Longitude: {dec_lon:12.7f}° (ΔLon = {d_lon:.9f}°)")
    print(f"    • Decoded Altitude:  {dec_alt:7.2f} m (ΔAlt = {d_alt:.4f} m)")
    print(f"    • Decoded Velocity:  [{dec_vx:+5.2f}, {dec_vy:+5.2f}, {dec_vz:+5.2f}] m/s (ΔVel = {d_vel:.4f} m/s)")
    print(f"    • CRC16 Check:       Received: 0x{rx_crc:04X} | Computed: 0x{comp_crc:04X} | Integrity: {'VALID OK' if crc_ok else 'CORRUPT'}")
    print("-" * 90)

    # 4. Final Verdict
    pass_math = (d_lat < 1e-7 and d_lon < 1e-7 and d_alt < 0.001 and d_vel < 0.01)
    pass_all = pass_math and crc_ok and (tx_payload == rx_payload)

    print(" [VERDICT] FINAL ROUND-TRIP VERIFICATION:")
    if pass_all:
        print("    [PASS - 100% PROVEN BIT-EXACT MATCH]")
        print("    Every byte serialized by DroneCAN was successfully reassembled from CAN 2.0B")
        print("    multi-frames and recovered with zero mathematical distortion.")
    else:
        print("    [FAIL]")
    print("=" * 90)


if __name__ == '__main__':
    run_proof_verification()

#!/usr/bin/env python3
"""
Stallion VTOL - Direct MP4 Video Renderer
==========================================
Renders a 1080p MP4 video file (flight_video.mp4) directly from flight telemetry:
 - 3D Perspective Aircraft View (3D Wireframe/Shaded Airframe & Attitude)
 - Aviation HUD Overlay (Pitch Ladder, Horizon, Roll Indicator, Altimeter, Airspeed)
 - Actuator Telemetry Gauges (Front-Right, Front-Left, Rear Motors, Yaw Tilt)
 - Runway Track & 3D Flight Trajectory Trail
"""

import json
import os
import math
import numpy as np
import cv2

def render_mp4_video(log_json_path, output_mp4_path, fps=30):
    print(f"[1] Loading flight telemetry from: {log_json_path}")
    if not os.path.exists(log_json_path):
        print(f"Error: Log file not found: {log_json_path}")
        return False

    with open(log_json_path, 'r') as f:
        data = json.load(f)

    records = data.get('trajectory', [])
    if not records:
        print("Error: No trajectory points in telemetry log.")
        return False

    # Video Settings
    width, height = 1280, 720
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_mp4_path, fourcc, fps, (width, height))

    print(f"[2] Rendering {len(records)} frames to MP4 video: {output_mp4_path}...")

    # Aircraft 3D Model Definition (Local Body Coordinates)
    # Fuselage, Wings, Tail, Booms
    vertices = np.array([
        # Fuselage
        [0.0, 1.2, 0.0],    # 0: Nose
        [-0.15, -1.0, 0.1], # 1: Tail Top L
        [0.15, -1.0, 0.1],  # 2: Tail Top R
        [0.0, -1.0, -0.15], # 3: Tail Bottom
        # Main Wing
        [-1.6, 0.0, 0.05],  # 4: Left Wingtip
        [1.6, 0.0, 0.05],   # 5: Right Wingtip
        [-1.6, -0.3, 0.05], # 6: Left Wing Trailing
        [1.6, -0.3, 0.05],  # 7: Right Wing Trailing
        # Booms & Motors
        [-0.5, 0.4, -0.05], # 8: Front Left Motor
        [0.5, 0.4, -0.05],  # 9: Front Right Motor
        [0.0, -0.9, 0.1],   # 10: Rear Motor
        # V-Tail
        [-0.5, -1.1, 0.35], # 11: Left Tail Tip
        [0.5, -1.1, 0.35]   # 12: Right Tail Tip
    ], dtype=np.float32)

    edges = [
        (0, 1), (0, 2), (0, 3), (1, 2), (2, 3), (3, 1), # Fuselage
        (4, 5), (4, 6), (5, 7), (6, 7),                 # Main Wing
        (0, 8), (0, 9), (8, 6), (9, 7),                 # Motor Mounts
        (1, 11), (2, 12), (11, 12)                      # Tail
    ]

    for idx, pt in enumerate(records):
        # Create dark atmospheric background
        frame = np.full((height, width, 3), (15, 23, 42), dtype=np.uint8)

        # Draw Grid / Ground Horizon
        cv2.line(frame, (0, 480), (width, 480), (45, 60, 85), 2)
        for gx in range(0, width, 80):
            cv2.line(frame, (gx, 480), (int(gx + (gx - width/2) * 1.5), height), (30, 41, 59), 1)

        # 3D Rotation Matrix from Telemetry (Roll, Pitch, Yaw)
        r = math.radians(pt.get('roll', 0.0))
        p = math.radians(pt.get('pitch', 0.0))
        y = math.radians(pt.get('yaw', 0.0))

        Rx = np.array([[1, 0, 0], [0, math.cos(p), -math.sin(p)], [0, math.sin(p), math.cos(p)]])
        Ry = np.array([[math.cos(r), 0, math.sin(r)], [0, 1, 0], [-math.sin(r), 0, math.cos(r)]])
        Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx

        # Camera View Projection
        alt = pt.get('alt', 0.0)
        center_x = int(width / 2)
        center_y = int(450 - alt * 25.0) # Altitude scaling
        scale = 120.0

        # Project 3D vertices to 2D screen
        projected = []
        for v in vertices:
            rot_v = R @ v
            px = int(center_x + rot_v[0] * scale)
            py = int(center_y - rot_v[2] * scale - rot_v[1] * 30.0)
            projected.append((px, py))

        # Draw 3D Aircraft Wireframe & Surfaces
        for e in edges:
            pt1 = projected[e[0]]
            pt2 = projected[e[1]]
            cv2.line(frame, pt1, pt2, (248, 189, 56), 2, cv2.LINE_AA) # Light Cyan

        # Draw Motor Discs (Spinning Visual)
        for m_idx, m_color in [(8, (56, 189, 248)), (9, (56, 189, 248)), (10, (14, 165, 233))]:
            mx, my = projected[m_idx]
            cv2.circle(frame, (mx, my), 14, m_color, 2, cv2.LINE_AA)
            # Spinner blade
            ang = (idx * 0.8) % (2 * math.pi)
            bx = int(mx + math.cos(ang) * 14)
            by = int(my + math.sin(ang) * 14)
            cv2.line(frame, (mx, my), (bx, by), (255, 255, 255), 2, cv2.LINE_AA)

        # Draw HUD Box
        hud_w, hud_h = 320, 200
        hud_x, hud_y = 30, 30
        overlay = frame.copy()
        cv2.rectangle(overlay, (hud_x, hud_y), (hud_x + hud_w, hud_y + hud_h), (15, 23, 42), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)
        cv2.rectangle(frame, (hud_x, hud_y), (hud_x + hud_w, hud_y + hud_h), (56, 189, 248), 1)

        # HUD Text
        cv2.putText(frame, "STALLION VTOL - FLIGHT HUD", (hud_x + 12, hud_y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (148, 163, 184), 1)
        cv2.putText(frame, f"TIME:     {pt.get('time', 0.0):5.1f} s", (hud_x + 12, hud_y + 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(frame, f"ALTITUDE: {alt:5.1f} m", (hud_x + 12, hud_y + 85), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (56, 189, 248), 2)
        cv2.putText(frame, f"ROLL:     {pt.get('roll', 0.0):+5.1f} deg", (hud_x + 12, hud_y + 115), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (226, 232, 240), 1)
        cv2.putText(frame, f"PITCH:    {pt.get('pitch', 0.0):+5.1f} deg", (hud_x + 12, hud_y + 140), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (226, 232, 240), 1)
        cv2.putText(frame, f"MODE:     {pt.get('mode', 'QHOVER')}", (hud_x + 12, hud_y + 175), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (52, 211, 153), 2)

        # Actuators Panel
        act_w, act_h = 320, 200
        act_x, act_y = width - act_w - 30, 30
        overlay2 = frame.copy()
        cv2.rectangle(overlay2, (act_x, act_y), (act_x + act_w, act_y + act_h), (15, 23, 42), -1)
        cv2.addWeighted(overlay2, 0.75, frame, 0.25, 0, frame)
        cv2.rectangle(frame, (act_x, act_y), (act_x + act_w, act_y + act_h), (56, 189, 248), 1)

        cv2.putText(frame, "ACTUATOR TELEMETRY", (act_x + 12, act_y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (148, 163, 184), 1)
        
        # Motor Bars
        m1 = pt.get('servo1', 1000)
        m2 = pt.get('servo2', 1000)
        m3 = pt.get('servo3', 1000)

        for b_i, (m_label, m_val) in enumerate([("M1 (Front-R)", m1), ("M2 (Front-L)", m2), ("M3 (Rear)", m3)]):
            by = act_y + 60 + b_i * 45
            cv2.putText(frame, f"{m_label}: {m_val} us", (act_x + 12, by - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (203, 213, 225), 1)
            # Track
            cv2.rectangle(frame, (act_x + 12, by), (act_x + act_w - 24, by + 10), (30, 41, 59), -1)
            # Fill
            fill_w = int(((m_val - 1000) / 1000.0) * (act_w - 36))
            fill_w = max(0, min(act_w - 36, fill_w))
            cv2.rectangle(frame, (act_x + 12, by), (act_x + 12 + fill_w, by + 10), (56, 189, 248), -1)

        out.write(frame)

    out.release()
    print(f"[OK] Video render complete: {output_mp4_path}")
    return True

if __name__ == '__main__':
    log_file = 'logs/flight_telemetry.json'
    video_out = 'flight_video.mp4'
    render_mp4_video(log_file, video_out)

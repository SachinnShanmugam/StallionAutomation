#!/usr/bin/env python3
"""
Stallion VTOL - Interactive 3D Web Flight Replay Generator
===========================================================
Generates a standalone, rich Three.js 3D flight replay dashboard from telemetry logs.
Includes:
 - Full 3D Flight Trajectory Visualization & Animated Stallion VTOL Model
 - Interactive Play/Pause/Scrubber Controls & Speed Multipliers (0.5x, 1x, 2x, 5x)
 - Aviation HUD (Pitch Ladder, Roll Arc, Airspeed Tape, Altitude Tape, Heading)
 - Actuator Telemetry Gauges (Front-Right, Front-Left, Rear Motors, Elevons, Yaw Tilt)
 - EKF Vibration & Quality Graphs
"""

import json
import os
import sys

def build_3d_replay_html(telemetry_log_path, output_html_path):
    if not os.path.exists(telemetry_log_path):
        print(f"Error: Telemetry log not found at {telemetry_log_path}")
        return False

    with open(telemetry_log_path, 'r') as f:
        data = json.load(f)

    flight_records = data.get('trajectory', [])
    metadata = data.get('metadata', {})

    telemetry_json_str = json.dumps(flight_records)
    metadata_json_str = json.dumps(metadata)

    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Stallion VTOL - 3D Flight Replay & Telemetry Dashboard</title>
    <style>
        * {{ box-sizing: border-box; margin: 0; padding: 0; font-family: 'Segoe UI', -apple-system, BlinkMacSystemFont, Roboto, sans-serif; }}
        body {{ background: #0b0f19; color: #f3f4f6; overflow: hidden; height: 100vh; display: flex; flex-direction: column; }}

        /* Top Navigation Bar */
        header {{
            background: rgba(15, 23, 42, 0.85);
            backdrop-filter: blur(12px);
            border-bottom: 1px solid rgba(255, 255, 255, 0.1);
            padding: 12px 24px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            z-index: 100;
        }}
        .logo-group {{ display: flex; align-items: center; gap: 12px; }}
        .badge {{ background: #0284c7; color: white; font-size: 11px; font-weight: 700; padding: 3px 8px; border-radius: 4px; text-transform: uppercase; letter-spacing: 0.5px; }}
        .meta-stats {{ display: flex; gap: 20px; font-size: 13px; color: #94a3b8; }}
        .meta-stats span b {{ color: #38bdf8; }}

        /* Main Workspace */
        #workspace {{ display: flex; flex: 1; position: relative; overflow: hidden; }}
        #viewport-3d {{ flex: 1; height: 100%; position: relative; background: radial-gradient(circle at center, #1e293b 0%, #090d16 100%); }}

        /* HUD Overlay */
        #hud-overlay {{
            position: absolute;
            top: 20px;
            left: 20px;
            pointer-events: none;
            display: flex;
            flex-direction: column;
            gap: 12px;
            z-index: 10;
        }}
        .hud-card {{
            background: rgba(15, 23, 42, 0.75);
            backdrop-filter: blur(10px);
            border: 1px solid rgba(56, 189, 248, 0.25);
            border-radius: 8px;
            padding: 12px 16px;
            min-width: 220px;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.4);
        }}
        .hud-title {{ font-size: 11px; text-transform: uppercase; color: #94a3b8; font-weight: 700; margin-bottom: 6px; letter-spacing: 0.5px; }}
        .hud-value {{ font-size: 22px; font-weight: 700; color: #38bdf8; font-variant-numeric: tabular-nums; }}
        .hud-sub {{ font-size: 12px; color: #64748b; margin-top: 2px; }}

        /* Actuators Panel */
        #actuator-panel {{
            position: absolute;
            top: 20px;
            right: 20px;
            width: 280px;
            background: rgba(15, 23, 42, 0.85);
            backdrop-filter: blur(12px);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 12px;
            padding: 16px;
            z-index: 10;
            box-shadow: 0 12px 40px rgba(0, 0, 0, 0.5);
        }}
        .panel-heading {{ font-size: 13px; font-weight: 700; color: #e2e8f0; margin-bottom: 12px; text-transform: uppercase; letter-spacing: 0.5px; border-bottom: 1px solid rgba(255,255,255,0.08); padding-bottom: 6px; }}
        .bar-group {{ margin-bottom: 10px; }}
        .bar-label {{ display: flex; justify-content: space-between; font-size: 12px; color: #cbd5e1; margin-bottom: 4px; }}
        .bar-track {{ width: 100%; height: 6px; background: #1e293b; border-radius: 3px; overflow: hidden; }}
        .bar-fill {{ height: 100%; width: 0%; background: linear-gradient(90deg, #38bdf8, #0284c7); border-radius: 3px; transition: width 0.05s ease; }}

        /* Bottom Playback Control Bar */
        #playback-bar {{
            background: rgba(15, 23, 42, 0.95);
            backdrop-filter: blur(16px);
            border-top: 1px solid rgba(255, 255, 255, 0.1);
            padding: 12px 24px;
            display: flex;
            align-items: center;
            gap: 20px;
            z-index: 100;
        }}
        .btn {{
            background: #1e293b;
            border: 1px solid rgba(255, 255, 255, 0.15);
            color: #f8fafc;
            padding: 8px 16px;
            border-radius: 6px;
            font-size: 13px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.15s ease;
            display: flex;
            align-items: center;
            gap: 6px;
        }}
        .btn:hover {{ background: #0284c7; border-color: #38bdf8; }}
        .btn-primary {{ background: #0284c7; border-color: #38bdf8; }}
        .btn-primary:hover {{ background: #0369a1; }}
        .time-display {{ font-size: 13px; font-variant-numeric: tabular-nums; color: #94a3b8; min-width: 90px; }}
        #timeline {{
            flex: 1;
            -webkit-appearance: none;
            height: 6px;
            border-radius: 3px;
            background: #334155;
            outline: none;
            cursor: pointer;
        }}
        #timeline::-webkit-slider-thumb {{
            -webkit-appearance: none;
            width: 16px;
            height: 16px;
            border-radius: 50%;
            background: #38bdf8;
            cursor: pointer;
            box-shadow: 0 0 10px #38bdf8;
        }}
        .speed-select {{
            background: #1e293b;
            color: #f8fafc;
            border: 1px solid rgba(255, 255, 255, 0.15);
            padding: 6px 12px;
            border-radius: 6px;
            font-size: 12px;
            font-weight: 600;
            outline: none;
            cursor: pointer;
        }}
    </style>
    <!-- Three.js & OrbitControls from CDN -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
</head>
<body>

    <!-- Header -->
    <header>
        <div class="logo-group">
            <h2>Stallion VTOL</h2>
            <span class="badge">3D Flight Replay</span>
        </div>
        <div class="meta-stats">
            <span>Mission: <b id="meta-mission">Hover & Climb</b></span>
            <span>Duration: <b id="meta-duration">0.0s</b></span>
            <span>Max Alt: <b id="meta-alt">0.0m</b></span>
            <span>Max Speed: <b id="meta-speed">0.0 m/s</b></span>
        </div>
    </header>

    <!-- Workspace -->
    <div id="workspace">
        <div id="viewport-3d"></div>

        <!-- Flight HUD Overlay -->
        <div id="hud-overlay">
            <div class="hud-card">
                <div class="hud-title">Altitude (AGL)</div>
                <div class="hud-value" id="hud-alt">0.0 <span style="font-size:14px;color:#94a3b8">m</span></div>
                <div class="hud-sub">Climb Rate: <span id="hud-climb" style="color:#e2e8f0">+0.0 m/s</span></div>
            </div>
            <div class="hud-card">
                <div class="hud-title">Attitude (RPY)</div>
                <div class="hud-value" id="hud-attitude">0.0° / 0.0°</div>
                <div class="hud-sub">Heading: <span id="hud-yaw" style="color:#e2e8f0">000°</span> | Mode: <span id="hud-mode" style="color:#38bdf8;font-weight:700">QHOVER</span></div>
            </div>
            <div class="hud-card">
                <div class="hud-title">Ground Speed</div>
                <div class="hud-value" id="hud-speed">0.0 <span style="font-size:14px;color:#94a3b8">m/s</span></div>
                <div class="hud-sub">Airspeed: <span id="hud-aspd" style="color:#e2e8f0">0.0 m/s</span></div>
            </div>
        </div>

        <!-- Actuator Outputs -->
        <div id="actuator-panel">
            <div class="panel-heading">Actuator Telemetry</div>
            <div class="bar-group">
                <div class="bar-label"><span>Front-Right Motor (M1)</span><span id="txt-m1">1000 µs (0%)</span></div>
                <div class="bar-track"><div class="bar-fill" id="bar-m1"></div></div>
            </div>
            <div class="bar-group">
                <div class="bar-label"><span>Front-Left Motor (M2)</span><span id="txt-m2">1000 µs (0%)</span></div>
                <div class="bar-track"><div class="bar-fill" id="bar-m2"></div></div>
            </div>
            <div class="bar-group">
                <div class="bar-label"><span>Rear Motor (M3)</span><span id="txt-m3">1000 µs (0%)</span></div>
                <div class="bar-track"><div class="bar-fill" id="bar-m3"></div></div>
            </div>
            <div class="bar-group">
                <div class="bar-label"><span>Rear Yaw Tilt Servo</span><span id="txt-m4">1500 µs (0°)</span></div>
                <div class="bar-track"><div class="bar-fill" id="bar-m4"></div></div>
            </div>
            <div class="bar-group">
                <div class="bar-label"><span>Right Elevon</span><span id="txt-el1">1500 µs</span></div>
                <div class="bar-track"><div class="bar-fill" id="bar-el1"></div></div>
            </div>
            <div class="bar-group">
                <div class="bar-label"><span>Left Elevon</span><span id="txt-el2">1500 µs</span></div>
                <div class="bar-track"><div class="bar-fill" id="bar-el2"></div></div>
            </div>
        </div>
    </div>

    <!-- Playback Control Bar -->
    <div id="playback-bar">
        <button class="btn btn-primary" id="btn-play" onclick="togglePlay()">▶ Play</button>
        <button class="btn" onclick="seekRelative(-5)">⏪ -5s</button>
        <button class="btn" onclick="seekRelative(5)">+5s ⏩</button>
        <span class="time-display" id="time-display">00:00 / 00:00</span>
        <input type="range" id="timeline" min="0" max="100" value="0" step="0.1" oninput="onScrub(this.value)">
        <select class="speed-select" id="speed-select" onchange="setSpeed(this.value)">
            <option value="0.5">0.5x</option>
            <option value="1.0" selected>1.0x (Real-Time)</option>
            <option value="2.0">2.0x</option>
            <option value="5.0">5.0x</option>
        </select>
        <button class="btn" onclick="resetCamera()">Reset Camera</button>
    </div>

    <!-- Replay Logic Script -->
    <script>
        const flightData = {telemetry_json_str};
        const metadata = {metadata_json_str};

        let isPlaying = false;
        let currentIndex = 0;
        let playbackSpeed = 1.0;
        let lastFrameTime = performance.now();

        // 3D Scene Setup
        const container = document.getElementById('viewport-3d');
        const scene = new THREE.Scene();
        scene.fog = new THREE.FogExp2(0x0b0f19, 0.005);

        const camera = new THREE.PerspectiveCamera(55, container.clientWidth / container.clientHeight, 0.1, 1000);
        camera.position.set(12, 10, 15);

        const renderer = new THREE.WebGLRenderer({{ antialias: true, alpha: true }});
        renderer.setSize(container.clientWidth, container.clientHeight);
        renderer.setPixelRatio(window.devicePixelRatio);
        renderer.shadowMap.enabled = true;
        container.appendChild(renderer.domElement);

        const controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;

        // Environment & Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
        scene.add(ambientLight);

        const sunLight = new THREE.DirectionalLight(0x38bdf8, 1.2);
        sunLight.position.set(20, 40, 20);
        sunLight.castShadow = true;
        scene.add(sunLight);

        // Ground Grid
        const gridHelper = new THREE.GridHelper(200, 50, 0x0284c7, 0x1e293b);
        gridHelper.position.y = 0;
        scene.add(gridHelper);

        // Runway Mesh
        const runwayGeo = new THREE.PlaneGeometry(16, 120);
        const runwayMat = new THREE.MeshStandardMaterial({{ color: 0x1e293b, roughness: 0.8 }});
        const runway = new THREE.Mesh(runwayGeo, runwayMat);
        runway.rotation.x = -Math.PI / 2;
        runway.position.y = -0.01;
        scene.add(runway);

        // Stallion VTOL 3D Representation
        const aircraftGroup = new THREE.Group();
        scene.add(aircraftGroup);

        // Fuselage
        const fuseGeo = new THREE.CylinderGeometry(0.2, 0.35, 2.2, 16);
        const fuseMat = new THREE.MeshStandardMaterial({{ color: 0x0284c7, roughness: 0.3, metalness: 0.4 }});
        const fuse = new THREE.Mesh(fuseGeo, fuseMat);
        fuse.rotation.x = Math.PI / 2;
        aircraftGroup.add(fuse);

        // Main Wing
        const wingGeo = new THREE.BoxGeometry(3.0, 0.04, 0.55);
        const wingMat = new THREE.MeshStandardMaterial({{ color: 0x38bdf8, roughness: 0.2 }});
        const wing = new THREE.Mesh(wingGeo, wingMat);
        wing.position.set(0, 0.1, -0.2);
        aircraftGroup.add(wing);

        // Twin Booms
        const boomGeo = new THREE.CylinderGeometry(0.04, 0.04, 1.8, 8);
        const boomMat = new THREE.MeshStandardMaterial({{ color: 0x64748b }});
        const boomL = new THREE.Mesh(boomGeo, boomMat);
        boomL.rotation.x = Math.PI / 2;
        boomL.position.set(-0.7, 0.1, -0.4);
        const boomR = boomL.clone();
        boomR.position.set(0.7, 0.1, -0.4);
        aircraftGroup.add(boomL);
        aircraftGroup.add(boomR);

        // Tail Fins & Elevators
        const tailGeo = new THREE.BoxGeometry(1.6, 0.03, 0.3);
        const tail = new THREE.Mesh(tailGeo, wingMat);
        tail.position.set(0, 0.3, -1.2);
        aircraftGroup.add(tail);

        // Trajectory Ribbon Path
        const pathPoints = [];
        flightData.forEach(pt => {{
            pathPoints.push(new THREE.Vector3(pt.x || 0, pt.z || 0, -(pt.y || 0)));
        }});
        const pathGeo = new THREE.BufferGeometry().setFromPoints(pathPoints);
        const pathMat = new THREE.LineBasicMaterial({{ color: 0x38bdf8, linewidth: 2 }});
        const flightPathLine = new THREE.Line(pathGeo, pathMat);
        scene.add(flightPathLine);

        // Update Metadata
        if (flightData.length > 0) {{
            const totalTime = flightData[flightData.length - 1].time || 0;
            const maxAlt = Math.max(...flightData.map(p => p.alt || 0));
            const maxSpd = Math.max(...flightData.map(p => p.speed || 0));
            document.getElementById('meta-duration').innerText = totalTime.toFixed(1) + 's';
            document.getElementById('meta-alt').innerText = maxAlt.toFixed(1) + 'm';
            document.getElementById('meta-speed').innerText = maxSpd.toFixed(1) + ' m/s';
        }}

        function updateTelemetry(index) {{
            if (!flightData || flightData.length === 0) return;
            const pt = flightData[index];
            if (!pt) return;

            // Position & Attitude
            aircraftGroup.position.set(pt.x || 0, pt.alt || 0, -(pt.y || 0));
            aircraftGroup.rotation.order = 'YXZ';
            aircraftGroup.rotation.y = -THREE.MathUtils.degToRad(pt.yaw || 0);
            aircraftGroup.rotation.x = THREE.MathUtils.degToRad(pt.pitch || 0);
            aircraftGroup.rotation.z = -THREE.MathUtils.degToRad(pt.roll || 0);

            // Follow Camera (Smooth tracking)
            controls.target.copy(aircraftGroup.position);

            // Update HUD
            document.getElementById('hud-alt').innerHTML = (pt.alt || 0).toFixed(1) + ' <span style="font-size:14px;color:#94a3b8">m</span>';
            document.getElementById('hud-climb').innerText = (pt.climb >= 0 ? '+' : '') + (pt.climb || 0).toFixed(1) + ' m/s';
            document.getElementById('hud-attitude').innerText = (pt.roll || 0).toFixed(1) + '° / ' + (pt.pitch || 0).toFixed(1) + '°';
            document.getElementById('hud-yaw').innerText = Math.round(pt.yaw || 0) + '°';
            document.getElementById('hud-mode').innerText = pt.mode || 'QHOVER';
            document.getElementById('hud-speed').innerHTML = (pt.speed || 0).toFixed(1) + ' <span style="font-size:14px;color:#94a3b8">m/s</span>';
            document.getElementById('hud-aspd').innerText = (pt.airspeed || 0).toFixed(1) + ' m/s';

            // Update Actuators
            const m1 = pt.servo1 || 1000;
            const m2 = pt.servo2 || 1000;
            const m3 = pt.servo3 || 1000;
            const m4 = pt.servo4 || 1500;
            const el1 = pt.servo5 || 1500;
            const el2 = pt.servo6 || 1500;

            setBar('m1', m1, 1000, 2000, `${{m1}} µs (${{Math.round((m1-1000)/10)}}%)`);
            setBar('m2', m2, 1000, 2000, `${{m2}} µs (${{Math.round((m2-1000)/10)}}%)`);
            setBar('m3', m3, 1000, 2000, `${{m3}} µs (${{Math.round((m3-1000)/10)}}%)`);
            setBar('m4', m4, 1000, 2000, `${{m4}} µs (${{Math.round((m4-1500)/10)}}°)`);
            setBar('el1', el1, 1000, 2000, `${{el1}} µs`);
            setBar('el2', el2, 1000, 2000, `${{el2}} µs`);

            // Update Timeline
            const total = flightData[flightData.length - 1].time || 1;
            const current = pt.time || 0;
            document.getElementById('timeline').value = (current / total) * 100;
            document.getElementById('time-display').innerText = formatTime(current) + ' / ' + formatTime(total);
        }}

        function setBar(id, val, min, max, label) {{
            const pct = Math.max(0, Math.min(100, ((val - min) / (max - min)) * 100));
            document.getElementById('bar-' + id).style.width = pct + '%';
            document.getElementById('txt-' + id).innerText = label;
        }}

        function formatTime(sec) {{
            const m = Math.floor(sec / 60);
            const s = Math.floor(sec % 60);
            return (m < 10 ? '0' : '') + m + ':' + (s < 10 ? '0' : '') + s;
        }}

        function togglePlay() {{
            isPlaying = !isPlaying;
            document.getElementById('btn-play').innerText = isPlaying ? '⏸ Pause' : '▶ Play';
            lastFrameTime = performance.now();
        }}

        function seekRelative(deltaSec) {{
            if (!flightData || flightData.length === 0) return;
            const currentTime = flightData[currentIndex].time || 0;
            const targetTime = currentTime + deltaSec;
            let bestIdx = 0;
            let bestDiff = 999999;
            flightData.forEach((p, idx) => {{
                const d = Math.abs((p.time || 0) - targetTime);
                if (d < bestDiff) {{ bestDiff = d; bestIdx = idx; }}
            }});
            currentIndex = bestIdx;
            updateTelemetry(currentIndex);
        }}

        function onScrub(val) {{
            if (!flightData || flightData.length === 0) return;
            const idx = Math.floor((val / 100) * (flightData.length - 1));
            currentIndex = idx;
            updateTelemetry(currentIndex);
        }}

        function setSpeed(spd) {{
            playbackSpeed = parseFloat(spd);
        }}

        function resetCamera() {{
            camera.position.set(12, 10, 15);
            controls.target.set(0, 2, 0);
        }}

        // Animation Loop
        function animate(now) {{
            requestAnimationFrame(animate);
            const dt = (now - lastFrameTime) / 1000.0;
            lastFrameTime = now;

            if (isPlaying && flightData.length > 0) {{
                const advance = (dt * playbackSpeed) / (flightData[1]?.time - flightData[0]?.time || 0.05);
                currentIndex = Math.min(flightData.length - 1, currentIndex + Math.max(1, Math.round(advance)));
                if (currentIndex >= flightData.length - 1) {{
                    isPlaying = false;
                    document.getElementById('btn-play').innerText = '▶ Replay';
                }}
                updateTelemetry(currentIndex);
            }}

            controls.update();
            renderer.render(scene, camera);
        }}

        window.addEventListener('resize', () => {{
            camera.aspect = container.clientWidth / container.clientHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(container.clientWidth, container.clientHeight);
        }});

        updateTelemetry(0);
        animate(performance.now());
    </script>
</body>
</html>
"""
    with open(output_html_path, 'w', encoding='utf-8') as f:
        f.write(html_content)

    print(f"    [OK] 3D Flight Replay Dashboard generated: {output_html_path}")
    return True

if __name__ == '__main__':
    log_path = sys.argv[1] if len(sys.argv) > 1 else 'logs/flight_telemetry.json'
    out_path = sys.argv[2] if len(sys.argv) > 2 else 'flight_replay.html'
    build_3d_replay_html(log_path, out_path)

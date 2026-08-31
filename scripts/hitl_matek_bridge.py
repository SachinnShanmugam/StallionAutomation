import sys, time, json, math, struct, socket, threading, platform, subprocess
from pymavlink import mavutil

if platform.system() == 'Linux':
    DEFAULT_PORT = '/dev/ttyACM0'
    try:
        WINDOWS_HOST = subprocess.check_output("ip route show default", shell=True).decode().split()[2]
    except Exception:
        WINDOWS_HOST = '172.26.16.1'
else:
    DEFAULT_PORT = 'COM7'
    WINDOWS_HOST = '127.0.0.1'

COM_PORT = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
BAUD_RATE = 115200
GZ_PORT = 9002
GCS_PORT = 14550

HOME_LAT = 13.0827
HOME_LON = 80.2707
HOME_ALT = 10.0
DEG_PER_METER = 1.0 / 111319.5

# Ground idle
throttle_val = 1000
is_running = True
last_pwm = [1000] * 16
last_pwm[3] = 1000
last_pwm[4] = 1500

# Loiter position anchor
loiter_target_north = 0.0
loiter_target_east = 0.0
loiter_target_yaw = 0.0
loiter_target_alt = 8.0
loiter_locked = False
is_taking_off = False

def command_thread(fc):
    global throttle_val, is_running, loiter_target_north, loiter_target_east, loiter_target_yaw, loiter_target_alt, loiter_locked, is_taking_off
    print("\n" + "=" * 60)
    print(" READY! Ground Idle (Motors Off)")
    print(" Commands:")
    print("   [qloiter]             - Select QLOITER Mode on Ground")
    print("   [takeoff] / [fly]     - Direct QLOITER Liftoff to 8.0m")
    print("   [auto]                - Execute Autonomous Waypoint Mission")
    print("   [up] / [down]         - Step Altitude (+-2m)")
    print("   [land]                - Precision Vertical Landing (QLAND)")
    print("   [disarm]              - Cut Motors & Disarm")
    print("=" * 60 + "\n")
    while is_running:
        try:
            line = sys.stdin.readline()
            if not line:
                break
            cmd = line.strip().lower()
            if not cmd:
                continue

            if cmd in ['qloiter', 'loiter']:
                fc.set_mode(19)  # QLOITER
                loiter_locked = True
                print("[CMD] >>> QLOITER (GPS Position Hold) SELECTED! Anchored to Current Coordinates. Type [takeoff] or [fly] to lift off! <<<")

            elif cmd in ['takeoff', 'fly', 'hover', 'qhover', '']:
                # If in QLOITER, stay in QLOITER and climb straight up on the spot!
                if fc.flightmode != 'QLOITER':
                    fc.set_mode(19)  # Default to QLOITER for dead-stop vertical liftoff
                fc.mav.command_long_send(
                    fc.target_system, fc.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 21196, 0, 0, 0, 0, 0
                )
                throttle_val = 1580  # Smooth, gentle vertical climb rate
                is_taking_off = True
                loiter_target_alt = 8.0
                print(f"[CMD] >>> QLOITER TAKEOFF! Climbing smoothly and vertically to {loiter_target_alt:.1f}m... <<<")

            elif cmd == 'auto':
                fc.set_mode(10)  # AUTO
                fc.mav.command_long_send(
                    fc.target_system, fc.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 21196, 0, 0, 0, 0, 0
                )
                throttle_val = 1500
                is_taking_off = False
                print("[CMD] >>> Starting Autonomous Waypoint Mission! <<<")

            elif cmd == 'up':
                loiter_target_alt += 5.0
                throttle_val = min(2000, throttle_val + 100)
                print(f"[CMD] Climbing to target: {loiter_target_alt:.1f}m")

            elif cmd == 'down':
                loiter_target_alt = max(3.0, loiter_target_alt - 5.0)
                throttle_val = max(1100, throttle_val - 100)
                print(f"[CMD] Descending to target: {loiter_target_alt:.1f}m")

            elif cmd in ['land', 'qland']:
                fc.set_mode(20)  # QLAND
                throttle_val = 1500
                is_taking_off = False
                print("[CMD] Precision Landing in QLAND (Autonomous Descent & Touchdown)...")

            elif cmd in ['disarm', 'cut', 'stop']:
                throttle_val = 1000
                fc.mav.command_long_send(
                    fc.target_system, fc.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
                is_taking_off = False
                print("[CMD] Disarmed. Motors OFF.")

            elif cmd.startswith('throttle ') or cmd.startswith('thr ') or cmd.startswith('rc 3 '):
                throttle_val = int(cmd.split()[-1])
                print(f"[CMD] Throttle set to {throttle_val} µs")

        except Exception as e:
            print(f"Error: {e}")

def main():
    global throttle_val, last_pwm, is_running, loiter_target_north, loiter_target_east, loiter_target_alt, loiter_locked, is_taking_off
    print("=" * 65)
    print(" Flightory Stallion VTOL — Precision Closed-Loop HITL Bridge")
    print(f" Matek H743: {COM_PORT} | Gazebo: UDP {GZ_PORT} | GCS: {WINDOWS_HOST}:{GCS_PORT}")
    print("=" * 65)
    fc = None
    ports_to_try = [COM_PORT]
    if COM_PORT == '/dev/ttyACM0':
        ports_to_try.append('/dev/ttyACM1')
    elif COM_PORT == '/dev/ttyACM1':
        ports_to_try.append('/dev/ttyACM0')

    for p in ports_to_try:
        try:
            print(f"[1/3] Connecting to {p}...")
            fc = mavutil.mavlink_connection(p, baud=BAUD_RATE)
            fc.wait_heartbeat(timeout=6)
            print(f"      [OK] Connected to Matek H743 on {p}! SysID: {fc.target_system}")
            break
        except Exception as e:
            print(f"      Could not connect on {p}: {e}")
            fc = None

    if fc is None:
        print(f"      [ERROR] Could not connect to Matek H743 on any port.")
        sys.exit(1)

    hitl_overrides = {
        'ARMING_CHECK': 0.0, 'COMPASS_ENABLE': 0.0, 'COMPASS_USE': 0.0,
        'COMPASS_USE2': 0.0, 'COMPASS_USE3': 0.0, 'COMPASS_AUTODEC': 0.0,
        'BATT_MONITOR': 0.0, 'THR_FAILSAFE': 0.0, 'FS_SHORT_ACTN': 0.0, 'FS_LONG_ACTN': 0.0,
        'RC_PROTOCOLS': 0.0, 'LOG_BITMASK': 0.0, 'LOG_BACKEND_TYPE': 0.0, 'LOG_DISARMED': 0.0, 'TERRAIN_ENABLE': 0.0,
        'GPS_TYPE': 14.0, 'GPS_DELAY_MS': 0.0, 'GPS_RATE_MS': 100.0,
        'EK3_ENABLE': 1.0, 'EK2_ENABLE': 0.0, 'AHRS_EKF_TYPE': 3.0,
        'EK3_SRC1_POSXY': 3.0, 'EK3_SRC1_VELXY': 3.0,
        'EK3_SRC1_POSZ': 3.0, 'EK3_SRC1_VELZ': 3.0,
        'EK3_SRC1_YAW': 0.0, 'EK3_SRC2_YAW': 0.0, 'EK3_SRC3_YAW': 0.0,
        'EK3_GSF_USE_MASK': 1.0, 'EK3_CHECK_SCALE': 0.0,
        'Q_ENABLE': 1.0, 'Q_FRAME_CLASS': 7.0, 'Q_FRAME_TYPE': 0.0,
        'Q_TILT_ENABLE': 1.0, 'Q_TILT_MASK': 3.0, 'Q_TILT_TYPE': 0.0,
        'SERVO1_FUNCTION': 33.0, 'SERVO2_FUNCTION': 34.0, 'SERVO3_FUNCTION': 36.0,
        'SERVO4_FUNCTION': 41.0, 'SERVO5_FUNCTION': 39.0, 'SERVO7_FUNCTION': 39.0,
        'SERVO1_MIN': 1000.0, 'SERVO1_MAX': 2000.0,
        'SERVO2_MIN': 1000.0, 'SERVO2_MAX': 2000.0,
        'SERVO3_MIN': 1000.0, 'SERVO3_MAX': 2000.0,
        'SERVO4_MIN': 1000.0, 'SERVO4_MAX': 2000.0,
        'SERVO5_MIN': 1000.0, 'SERVO5_MAX': 2000.0, 'SERVO5_TRIM': 1500.0, 'SERVO5_REVERSED': 0.0,
        'SERVO7_MIN': 1000.0, 'SERVO7_MAX': 2000.0, 'SERVO7_TRIM': 1500.0, 'SERVO7_REVERSED': 0.0,
        'Q_M_YAW_SV_ANGLE': 15.0,
        'Q_TRIM_PITCH': 0.0,
        'Q_M_THST_HOVER': 0.45, 'Q_M_SPIN_ARM': 0.10, 'Q_M_SPIN_MIN': 0.15,
        'Q_A_ANG_RLL_P': 3.5, 'Q_A_ANG_PIT_P': 3.5, 'Q_A_ANG_YAW_P': 1.5,
        'Q_A_RAT_RLL_P': 0.10, 'Q_A_RAT_RLL_I': 0.15, 'Q_A_RAT_RLL_D': 0.001,
        'Q_A_RAT_PIT_P': 0.10, 'Q_A_RAT_PIT_I': 0.15, 'Q_A_RAT_PIT_D': 0.001,
        'Q_A_RAT_YAW_P': 0.08, 'Q_A_RAT_YAW_I': 0.03, 'Q_A_RAT_YAW_D': 0.001,
        'Q_WP_SPD': 2.5, 'Q_WP_SPD_UP': 0.8, 'Q_LOIT_SPEED_MS': 3.0,
        'Q_LAND_SPEED': 0.5, 'Q_LAND_FINAL_ALT': 2.0,
        'ARSPD_ENABLE': 0.0, 'ARSPD_USE': 0.0, 'ARSPD_FBW_MIN': 12.0, 'ARSPD_FBW_MAX': 25.0,
        'Q_TRANSITION_MS': 4000.0, 'Q_TAKEOFF_ALT': 10.0,
        'RC1_MIN': 1000.0, 'RC1_MAX': 2000.0, 'RC1_TRIM': 1500.0, 'RC1_DZ': 30.0,
        'RC2_MIN': 1000.0, 'RC2_MAX': 2000.0, 'RC2_TRIM': 1500.0, 'RC2_DZ': 30.0,
        'RC3_MIN': 1000.0, 'RC3_MAX': 2000.0, 'RC3_TRIM': 1000.0, 'RC3_DZ': 30.0,
        'RC4_MIN': 1000.0, 'RC4_MAX': 2000.0, 'RC4_TRIM': 1500.0, 'RC4_DZ': 30.0,
        'SR0_RAW_CTRL': 50.0, 'SR0_POSITION': 50.0, 'SR0_EXTRA1': 50.0,
    }
    for pname, pval in hitl_overrides.items():
        try:
            fc.param_set_send(pname, float(pval))
        except Exception:
            pass
        time.sleep(0.01)

    try:
        # Force disarm on startup so no residual motor commands run on the ground
        fc.mav.command_long_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
        # Preflight calibrate level to desk
        fc.mav.command_long_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, 2, 0, 0)
        fc.mav.set_gps_global_origin_send(
            fc.target_system,
            int(HOME_LAT * 1e7),
            int(HOME_LON * 1e7),
            int(HOME_ALT * 1000)
        )
        fc.set_mode(19)  # Default to QLOITER on ground
        fc.mav.request_data_stream_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1)
        fc.mav.command_long_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 36, 20000, 0, 0, 0, 0, 0)
    except Exception:
        pass

    print("[2/3] Connecting to Gazebo Physics on UDP 9002...")
    gz_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gz_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    gz_sock.bind(('0.0.0.0', 0))
    gz_sock.setblocking(False)

    frame_count = 1
    handshake = struct.pack('<HHI16H', 18458, 400, frame_count, *([1000] * 16))
    for target_ip in ['127.0.0.1', '172.26.30.4']:
        try:
            gz_sock.sendto(handshake, (target_ip, GZ_PORT))
        except Exception:
            pass

    print(f"[3/3] Telemetry socket ready on {WINDOWS_HOST}:{GCS_PORT}...")
    gcs_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gcs_sock.setblocking(False)

    th = threading.Thread(target=command_thread, args=(fc,), daemon=True)
    th.start()

    start_time = time.time()
    gz_client = None
    packet_count = 0
    last_rc_time = 0.0
    last_gps_time = 0.0
    last_print = time.time()
    gps_epoch_offset = 315964800
    rel_alt = 0.0
    lat = HOME_LAT
    lon = HOME_LON
    amsl_alt = HOME_ALT
    speed = 0.0
    yaw_cdeg = 0
    vn = 0.0
    ve = 0.0
    vd = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    x_north = 0.0
    y_east = 0.0
    dist_h = 0.0
    gz_pwm = [1000] * 16

    while is_running:
        now = time.time()
        boot_time_ms = int((now - start_time) * 1000) % 4294967295

        if packet_count == 0:
            frame_count += 1
            handshake = struct.pack('<HHI16H', 18458, 400, frame_count, *([1000] * 16))
            try:
                gz_sock.sendto(handshake, ('127.0.0.1', GZ_PORT))
            except Exception:
                pass
            time.sleep(0.05)

        # ─── A. Read Gazebo Physics (Authentic NED Frame) ───
        try:
            raw, addr = gz_sock.recvfrom(8192)
            gz_client = addr
            fdm = json.loads(raw.decode('utf-8'))
            packet_count += 1

            pos = fdm.get('position', [0, 0, 0])
            vel = fdm.get('velocity', [0, 0, 0])
            quat = fdm.get('quaternion', [1, 0, 0, 0])
            imu_data = fdm.get('imu', {})
            gyro = imu_data.get('gyro', [0, 0, 0])
            accel_raw = imu_data.get('accel_body', imu_data.get('accel', [0, 0, 9.81]))
            # 100% authentic body-frame IMU (matching ArduPilot SIM_JSON.cpp):
            accel = [float(accel_raw[0]), float(accel_raw[1]), float(accel_raw[2])]

            # Position in NED: pos[0]=North, pos[1]=East, pos[2]=Down (-rel_alt)
            x_north = pos[0]
            y_east = pos[1]
            rel_alt = max(0.0, -pos[2])
            amsl_alt = HOME_ALT + rel_alt

            # Capture initial ground spot & heading anchor
            if not loiter_locked:
                loiter_target_north = x_north
                loiter_target_east = y_east
                loiter_target_yaw = yaw
                loiter_locked = True

            # Automatic center stick once airborne (0.25m) so ArduPilot locks altitude without saturating motors:
            if is_taking_off and rel_alt >= 0.25:
                is_taking_off = False
                throttle_val = 1500  # Center stick -> ArduPilot locks altitude cleanly with 100% control headroom!
                print(f"[CMD] >>> AIRBORNE AT {rel_alt:.1f}m! Throttle stick centered (1500 µs) -> ArduPilot Altitude & Position Hold ACTIVE! <<<")

            lat = HOME_LAT + (x_north * DEG_PER_METER)
            lon = HOME_LON + (y_east * DEG_PER_METER / math.cos(math.radians(HOME_LAT)))

            vn = float(vel[0])
            ve = float(vel[1])
            vd = float(vel[2])
            speed = math.sqrt(vn**2 + ve**2)

            qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
            roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
            sinp = 2.0 * (qw * qy - qz * qx)
            pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            yaw_cdeg = int(math.degrees(yaw) * 100) % 36000

            # ─── Pure 100% ArduPilot HIL: Stream 400 Hz HIL_SENSOR to Matek & Pass Unmodified PWMs to Gazebo ───
            frame_count += 1

            # 1. Calculate Standard Barometric Pressure from Altitude:
            abs_press = float(1013.25 * math.pow(1.0 - (amsl_alt / 44330.0), 5.255))

            # 2. Stream 400 Hz IMU (Accels + Gyros + Baro) directly to Matek STM32:
            try:
                fc.mav.hil_sensor_send(
                    int(now * 1e6),
                    float(accel[0]), float(accel[1]), float(accel[2]),  # 3D Accelerometers (m/s²)
                    float(gyro[0]), float(gyro[1]), float(gyro[2]),    # 3D Gyroscopes (rad/s)
                    0.0, 0.0, 0.0,                                     # Magnetometer (GSF handles compass)
                    abs_press,                                         # Barometric Pressure (hPa)
                    0.0,                                               # Differential Airspeed Pressure
                    float(amsl_alt),                                   # Pressure Altitude (m)
                    20.0,                                              # Temperature (°C)
                    0x1FFF                                             # Bitmask: All sensors valid
                )
            except Exception:
                pass

            # 3. 100% Pure 1:1 Actuator Passthrough (ArduPilot Controls All Motors & Servos Directly!)
            gz_pwm = list(last_pwm)
            # Route active Tricopter Yaw Servo (SERVO7 ch6 or SERVO5 ch4) to Gazebo Channel 4:
            # SDF multiplier is now +0.524 (corrected), so pass through directly 1:1
            raw_yaw = last_pwm[6] if abs(last_pwm[6] - 1500) > abs(last_pwm[4] - 1500) else last_pwm[4]
            gz_pwm[4] = raw_yaw

            out_pkt = struct.pack('<HHI16H', 18458, 400, frame_count, *gz_pwm)
            gz_sock.sendto(out_pkt, gz_client)

            # Send Horizon to Mission Planner
            att = fc.mav.attitude_encode(boot_time_ms, float(roll), float(pitch), float(yaw), float(gyro[0]), float(gyro[1]), float(gyro[2]))
            att_buf = att.pack(fc.mav)
            for target in [(WINDOWS_HOST, GCS_PORT), ('127.0.0.1', GCS_PORT)]:
                try:
                    gcs_sock.sendto(att_buf, target)
                except Exception:
                    pass

        except (BlockingIOError, socket.error):
            pass
        except Exception:
            pass

        # ─── B. Send Clean 10 Hz Precision GPS to Matek & Mission Planner ───
        if now - last_gps_time > 0.10:
            last_gps_time = now
            gps_seconds = int(now - gps_epoch_offset + 18)
            time_week = int(gps_seconds / 604800)
            time_week_ms = int((gps_seconds % 604800) * 1000 + (now % 1.0) * 1000)

            # 10 Hz GPS_INPUT to Matek
            fc.mav.gps_input_send(
                int(now * 1e6),
                0, 0,
                time_week_ms, time_week,
                3, # 3D Fix
                int(lat * 1e7), int(lon * 1e7), float(amsl_alt),
                0.6, 0.6,
                vn, ve, vd,
                0.05, 0.2, 0.2,
                18,
                yaw_cdeg
            )

            # 10 Hz HUD & Global Position to Mission Planner
            vfr = fc.mav.vfr_hud_encode(
                float(speed), float(speed),
                int(yaw_cdeg * 0.01) % 360,
                int(max(0, min(100, (last_pwm[0] - 1000) / 10.0))),
                float(rel_alt), float(-vd)
            )
            vfr_buf = vfr.pack(fc.mav)

            gpos = fc.mav.global_position_int_encode(
                boot_time_ms,
                int(lat * 1e7), int(lon * 1e7),
                int(amsl_alt * 1000), int(rel_alt * 1000),
                int(vn * 100), int(ve * 100), int(vd * 100),
                yaw_cdeg
            )
            gpos_buf = gpos.pack(fc.mav)

            for target in [(WINDOWS_HOST, GCS_PORT), ('127.0.0.1', GCS_PORT)]:
                try:
                    gcs_sock.sendto(vfr_buf, target)
                    gcs_sock.sendto(gpos_buf, target)
                except Exception:
                    pass

        # ─── C. Read Actuators from Matek ───
        msg = fc.recv_msg()
        if msg:
            FILTERED_TYPES = [
                'ATTITUDE', 'VFR_HUD', 'GLOBAL_POSITION_INT', 'ALTITUDE',
                'SCALED_PRESSURE', 'SCALED_PRESSURE2', 'AHRS', 'AHRS2', 'AHRS3',
                'RAW_IMU', 'HIGHRES_IMU'
            ]
            if msg.get_type() not in FILTERED_TYPES:
                try:
                    mbuf = msg.get_msgbuf()
                    gcs_sock.sendto(mbuf, (WINDOWS_HOST, GCS_PORT))
                    gcs_sock.sendto(mbuf, ('127.0.0.1', GCS_PORT))
                except Exception:
                    pass

            if msg.get_type() == 'STATUSTEXT':
                if 'ENOSPC' not in msg.text:
                    print(f"[MATEK] {msg.text}")

            if msg.get_type() == 'SERVO_OUTPUT_RAW':
                last_pwm[0] = msg.servo1_raw
                last_pwm[1] = msg.servo2_raw
                last_pwm[2] = msg.servo3_raw
                last_pwm[3] = msg.servo4_raw
                last_pwm[4] = msg.servo5_raw
                last_pwm[5] = msg.servo6_raw
                last_pwm[6] = msg.servo7_raw
                last_pwm[7] = msg.servo8_raw

        # ─── D. 20 Hz RC Override ───
        if now - last_rc_time > 0.05:
            last_rc_time = now
            fc.mav.rc_channels_override_send(fc.target_system, fc.target_component, 1500, 1500, throttle_val, 1500, 0, 0, 0, 0)

        # ─── E. Forward Mission Planner Uplink ───
        try:
            gcs_data, gcs_src = gcs_sock.recvfrom(4096)
            fc.write(gcs_data)
        except (BlockingIOError, socket.error):
            pass

        if now - last_print > 1.5:
            last_print = now
            dist_h = math.sqrt((x_north - loiter_target_north)**2 + (y_east - loiter_target_east)**2)
            print(f"[STATUS] FR={gz_pwm[0]} FL={gz_pwm[1]} Rear={gz_pwm[2]} YawServo={gz_pwm[4]} | Alt={rel_alt:.1f}m | Dist={dist_h:.1f}m | Yaw={math.degrees(yaw):.0f}° P={math.degrees(pitch):.1f}° R={math.degrees(roll):.1f}° | Speed={speed:.1f}m/s")
        time.sleep(0.001)

if __name__ == '__main__':
    main()

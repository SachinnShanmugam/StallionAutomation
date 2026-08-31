#!/usr/bin/env python3
"""
Autonomous HITL Test & Tuning Harness for Flightory Stallion VTOL
Runs automated flight trials, resets Gazebo world, and evaluates flight stability.
"""

import sys, time, json, math, struct, socket, subprocess, os
from pymavlink import mavutil

GZ_PORT = 9002
GCS_PORT = 14550
HOME_LAT = 13.0827
HOME_LON = 80.2707
HOME_ALT = 10.0
DEG_PER_METER = 1.0 / 111319.5
MAG_NED = [0.280, 0.020, 0.360]

def reset_gazebo_world():
    """Resets Gazebo simulation state to t=0 and unpauses physics."""
    try:
        cmd = "gz service -s /world/stallion_runway/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --req 'reset: {all: true}, pause: false'"
        subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
        time.sleep(0.5)
        return True
    except Exception as e:
        print(f"[WARN] Gazebo reset failed: {e}")
        return False

def run_flight_trial(duration_sec=10.0, target_alt=4.0):
    print(f"\n=======================================================")
    print(f" [AUTONOMOUS TRIAL] Starting {duration_sec}s Flight Test...")
    print(f"=======================================================")
    
    # 1. Reset Gazebo
    reset_gazebo_world()
    
    # 2. Connect to Matek
    fc = None
    for port in ['/dev/ttyACM0', '/dev/ttyACM1', 'COM7']:
        try:
            fc = mavutil.mavlink_connection(port, baud=115200)
            fc.wait_heartbeat(timeout=3)
            print(f"      [OK] Connected to Matek H743 on {port} (SysID: {fc.target_system})")
            break
        except Exception:
            fc = None
            
    if fc is None:
        print("      [ERROR] Cannot connect to Matek H743.")
        return None

    # Setup sockets
    gz_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gz_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    gz_sock.bind(('127.0.0.1', 9003))
    gz_sock.setblocking(False)

    # Apply configuration parameters
    hitl_params = {
        'ARMING_CHECK': 0.0, 'COMPASS_ENABLE': 0.0, 'COMPASS_USE': 0.0,
        'COMPASS_USE2': 0.0, 'COMPASS_USE3': 0.0, 'COMPASS_AUTODEC': 0.0,
        'BATT_MONITOR': 0.0, 'THR_FAILSAFE': 0.0, 'FS_SHORT_ACTN': 0.0, 'FS_LONG_ACTN': 0.0,
        'RC_PROTOCOLS': 0.0, 'LOG_BITMASK': 0.0, 'LOG_BACKEND_TYPE': 0.0, 'LOG_DISARMED': 0.0, 'TERRAIN_ENABLE': 0.0,
        'AHRS_TRIM_X': 0.0, 'AHRS_TRIM_Y': 0.0, 'AHRS_TRIM_Z': 0.0,
        'GPS_TYPE': 14.0, 'GPS_DELAY_MS': 0.0, 'GPS_RATE_MS': 50.0,
        'EK3_ENABLE': 1.0, 'EK2_ENABLE': 0.0, 'AHRS_EKF_TYPE': 3.0,
        'EK3_SRC1_POSXY': 3.0, 'EK3_SRC1_VELXY': 3.0,
        'EK3_SRC1_POSZ': 1.0, 'EK3_SRC1_VELZ': 0.0,
        'EK3_SRC2_POSZ': 0.0,
        'EK3_SRC1_YAW': 2.0, 'EK3_SRC2_YAW': 0.0, 'EK3_SRC3_YAW': 0.0,
        'EK3_GSF_USE_MASK': 0.0, 'EK3_CHECK_SCALE': 0.0,
        'EK3_ALT_M_NSE': 3.0, 'EK3_POSNE_M_NSE': 1.0,
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
        'Q_M_YAW_SV_ANGLE': 12.0,
        'Q_TRIM_PITCH': 0.0,
        'AHRS_TRIM_X': 0.0, 'AHRS_TRIM_Y': 0.0, 'AHRS_TRIM_Z': 0.0,
        'Q_M_SPOOL_TIME': 0.5,
        'Q_M_THST_HOVER': 0.45, 'Q_M_SPIN_ARM': 0.10, 'Q_M_SPIN_MIN': 0.15,
        'Q_A_ANG_RLL_P': 5.0, 'Q_A_ANG_PIT_P': 5.0, 'Q_A_ANG_YAW_P': 3.0,
        'Q_A_RAT_RLL_P': 0.18, 'Q_A_RAT_RLL_I': 0.0, 'Q_A_RAT_RLL_D': 0.006, 'Q_A_RAT_RLL_IMAX': 0.0,
        'Q_A_RAT_PIT_P': 0.18, 'Q_A_RAT_PIT_I': 0.0, 'Q_A_RAT_PIT_D': 0.006, 'Q_A_RAT_PIT_IMAX': 0.0,
        'Q_A_RAT_YAW_P': 0.20, 'Q_A_RAT_YAW_I': 0.0, 'Q_A_RAT_YAW_D': 0.006, 'Q_A_RAT_YAW_IMAX': 0.0,
        'Q_A_ANGLE_MAX': 15.0,
        'Q_P_POSXY_P': 0.30, 'Q_P_VELXY_P': 0.50, 'Q_LOIT_SPEED_MS': 1.5,
        'Q_WP_SPD': 1.5, 'Q_WP_SPD_UP': 0.8,
        'Q_LAND_SPEED': 0.5, 'Q_LAND_FINAL_ALT': 2.0,
        'ARSPD_ENABLE': 0.0, 'ARSPD_USE': 0.0, 'ARSPD_FBW_MIN': 12.0, 'ARSPD_FBW_MAX': 25.0,
        'Q_TRANSITION_MS': 4000.0, 'Q_TAKEOFF_ALT': 10.0,
        'RC1_MIN': 1000.0, 'RC1_MAX': 2000.0, 'RC1_TRIM': 1500.0, 'RC1_DZ': 30.0,
        'RC2_MIN': 1000.0, 'RC2_MAX': 2000.0, 'RC2_TRIM': 1500.0, 'RC2_DZ': 30.0,
        'RC3_MIN': 1000.0, 'RC3_MAX': 2000.0, 'RC3_TRIM': 1000.0, 'RC3_DZ': 30.0,
        'RC4_MIN': 1000.0, 'RC4_MAX': 2000.0, 'RC4_TRIM': 1500.0, 'RC4_DZ': 30.0,
        'SR0_RAW_CTRL': 50.0, 'SR0_POSITION': 50.0, 'SR0_EXTRA1': 50.0,
    }
    for k, v in hitl_params.items():
        try:
            fc.param_set_send(k, float(v))
        except Exception:
            pass
        time.sleep(0.005)

    # Initial calibration
    fc.mav.command_long_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
    fc.mav.set_gps_global_origin_send(fc.target_system, int(HOME_LAT * 1e7), int(HOME_LON * 1e7), int(HOME_ALT * 1000), int(time.time() * 1e6))

    # Handshake with Gazebo
    gz_client = ('127.0.0.1', GZ_PORT)
    for _ in range(5):
        handshake = struct.pack('<HHI16H', 18458, 400, 1, *([1000] * 16))
        gz_sock.sendto(handshake, gz_client)
        time.sleep(0.02)

    # Wait for EKF3 to be ready (up to 5s)
    print("      [INIT] Waiting for EKF3 alignment...")
    t_start = time.time()
    last_hil = 0.0
    last_gps = 0.0
    hil_accel = [0.0, 0.0, 9.81]
    hil_gyro = [0.0, 0.0, 0.0]
    hil_mag = [MAG_NED[0], 0.0, MAG_NED[2]]
    rel_alt = 0.0
    last_pwm = [1000] * 16
    yaw_deg = 0.0
    pitch_deg = 0.0
    roll_deg = 0.0

    while time.time() - t_start < 4.0:
        now = time.time()
        try:
            raw, _ = gz_sock.recvfrom(8192)
            fdm = json.loads(raw.decode('utf-8'))
            imu = fdm.get('imu', {})
            accel_raw = imu.get('accel_body', imu.get('accel', [0,0,9.81]))
            gyro_raw = imu.get('gyro', [0,0,0])
            hil_accel = [float(accel_raw[0]), float(accel_raw[1]), float(accel_raw[2])]
            hil_gyro = [float(gyro_raw[0]), float(gyro_raw[1]), float(gyro_raw[2])]
        except Exception:
            pass

        if now - last_hil >= 0.0025:
            last_hil = now
            fc.mav.hil_sensor_send(int(now*1e6), hil_accel[0], hil_accel[1], hil_accel[2],
                                   hil_gyro[0], hil_gyro[1], hil_gyro[2],
                                   hil_mag[0], hil_mag[1], hil_mag[2],
                                   1013.25, 0.0, HOME_ALT, 20.0, 0x1FFF)

        if now - last_gps >= 0.05:
            last_gps = now
            fc.mav.gps_input_send(int(now*1e6), 0, 0, 1000, 100, 3,
                                  int(HOME_LAT*1e7), int(HOME_LON*1e7), HOME_ALT,
                                  0.5, 0.5, 0.0, 0.0, 0.0, 0.05, 0.2, 0.2, 18, 36000)

        time.sleep(0.001)

    # 3. ARM AND COMMAND TAKEOFF IN QHOVER (Mode 18: Pure vertical attitude climb)
    fc.set_mode(18) # QHOVER
    fc.mav.command_long_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    print("      [CMD] Armed in QHOVER! Clean vertical climb to 1.5m before QLOITER position lock...")
    
    throttle_val = 1650  # Crisp, clean liftoff
    mode_switched = False
    trial_start = time.time()
    last_rc = 0.0
    frame_count = 0
    amsl_alt = HOME_ALT
    vel = [0.0, 0.0, 0.0]

    dist_xy = 0.0
    rel_alt = 0.0
    roll_deg = 0.0
    pitch_deg = 0.0
    yaw_deg = 0.0

    max_alt = 0.0
    max_roll = 0.0
    max_pitch = 0.0
    max_yaw_rate = 0.0
    samples = 0
    stable_samples = 0

    print(f"{'Time':>6} | {'FR':>4} {'FL':>4} {'Rear':>4} {'YawSv':>5} | {'Alt':>5} | {'Dist':>5} | {'Roll':>6} {'Pitch':>6} {'Yaw':>6}")
    print("-" * 75)

    while time.time() - trial_start < duration_sec:
        now = time.time()
        elapsed = now - trial_start

        # Read Gazebo
        try:
            raw, _ = gz_sock.recvfrom(8192)
            fdm = json.loads(raw.decode('utf-8'))
            pos = fdm.get('position', [0,0,0])
            vel = fdm.get('velocity', [0,0,0])
            quat = fdm.get('quaternion', [1,0,0,0])
            imu = fdm.get('imu', {})
            accel_raw = imu.get('accel_body', imu.get('accel', [0,0,9.81]))
            gyro_raw = imu.get('gyro', [0,0,0])
            hil_accel = [float(accel_raw[0]), float(accel_raw[1]), float(accel_raw[2])]
            hil_gyro = [float(gyro_raw[0]), float(gyro_raw[1]), float(gyro_raw[2])]

            rel_alt = max(0.0, -pos[2])
            amsl_alt = HOME_ALT + rel_alt
            max_alt = max(max_alt, rel_alt)
            dist_xy = math.sqrt(pos[0]*pos[0] + pos[1]*pos[1])

            # Transition from QHOVER to QLOITER once airborne (>= 1.5m)
            if not mode_switched and rel_alt >= 1.5:
                fc.set_mode(19) # QLOITER
                mode_switched = True
                print(f"      [CMD] >>> AT {rel_alt:.1f}m: Switched QHOVER -> QLOITER! GPS Position Hold ACTIVE! <<<")

            # Smooth level-off and altitude/position lock at target altitude (5.0m)
            if rel_alt >= (target_alt - 0.5):
                throttle_val = 1500  # Center stick -> Holds 5.0m altitude and locks GPS position!

            qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
            roll  = math.atan2(2.0*(qw*qx+qy*qz), 1.0-2.0*(qx*qx+qy*qy))
            sinp  = 2.0*(qw*qy-qz*qx)
            pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
            yaw   = math.atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz))

            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)

            max_roll = max(max_roll, abs(roll_deg))
            max_pitch = max(max_pitch, abs(pitch_deg))
        except Exception:
            pass

        # Send actuators to Gazebo (Direct, pure 1:1 hardware-in-the-loop passthrough)
        frame_count += 1
        gz_pwm = list(last_pwm)
        gz_pwm[4] = last_pwm[4]

        try:
            out_pkt = struct.pack('<HHI16H', 18458, 400, frame_count, *gz_pwm)
            gz_sock.sendto(out_pkt, ('127.0.0.1', GZ_PORT))
        except Exception:
            pass

        # 400 Hz HIL_SENSOR
        if now - last_hil >= 0.0025:
            last_hil = now
            press = float(1013.25 * math.pow(1.0 - (amsl_alt / 44330.0), 5.255))
            try:
                fc.mav.hil_sensor_send(int(now*1e6), hil_accel[0], hil_accel[1], hil_accel[2],
                                       hil_gyro[0], hil_gyro[1], hil_gyro[2],
                                       MAG_NED[0], MAG_NED[1], MAG_NED[2],
                                       press, 0.0, float(amsl_alt), 20.0, 0x1FFF)
            except Exception:
                pass

        # 20 Hz GPS
        if now - last_gps >= 0.05:
            last_gps = now
            yaw_cdeg = int((yaw_deg % 360) * 100) or 36000
            try:
                fc.mav.gps_input_send(int(now*1e6), 0, 0, 1000, 100, 3,
                                      int(HOME_LAT*1e7), int(HOME_LON*1e7), float(amsl_alt),
                                      0.5, 0.5, float(vel[0]), float(vel[1]), -float(vel[2]),
                                      0.05, 0.2, 0.2, 18, yaw_cdeg)
            except Exception:
                pass

        # Read Matek (drain all incoming MAVLink packets)
        while True:
            msg = fc.recv_msg()
            if not msg:
                break
            if msg.get_type() == 'SERVO_OUTPUT_RAW':
                last_pwm[0] = msg.servo1_raw
                last_pwm[1] = msg.servo2_raw
                last_pwm[2] = msg.servo3_raw
                last_pwm[3] = msg.servo4_raw
                last_pwm[4] = msg.servo5_raw
                last_pwm[5] = msg.servo6_raw
                last_pwm[6] = msg.servo7_raw

        # 20 Hz RC
        if now - last_rc >= 0.05:
            last_rc = now
            fc.mav.rc_channels_override_send(fc.target_system, fc.target_component, 1500, 1500, throttle_val, 1500, 0, 0, 0, 0)

        # Print periodic line every 0.8s
        if samples % 80 == 0:
            print(f"{elapsed:5.1f}s | {last_pwm[0]:4d} {last_pwm[1]:4d} {last_pwm[2]:4d} {last_pwm[4]:5d} | {rel_alt:4.1f}m | {dist_xy:4.1f}m | {roll_deg:5.1f}° {pitch_deg:5.1f}° {yaw_deg:5.1f}°")

        samples += 1
        if rel_alt >= 1.0 and dist_xy < 2.0 and abs(roll_deg) < 20.0 and abs(pitch_deg) < 20.0:
            stable_samples += 1

        time.sleep(0.002)

    # Disarm
    fc.mav.command_long_send(fc.target_system, fc.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
    
    score = (stable_samples / max(1, samples)) * 100.0
    print(f"\n--- TRIAL EVALUATION ---")
    print(f"Max Altitude:    {max_alt:.2f} m")
    print(f"Max Roll Angle:  {max_roll:.1f}°")
    print(f"Max Pitch Angle: {max_pitch:.1f}°")
    print(f"Stability Score: {score:.1f}%")
    
    passed = (max_alt >= 1.5 and max_roll < 25.0 and max_pitch < 25.0)
    print(f"Result:          {'>>> PASSED <<<' if passed else 'FAILED'}")
    return passed

if __name__ == '__main__':
    run_flight_trial(duration_sec=15.0, target_alt=5.0)

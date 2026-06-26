"""
iRobot Create controller (PERSISTENT SPEED FIX + SONAR)
Exports sensor data + applies speed commands from customData continuously
"""

import io
import json
import os
import queue
import struct
import tempfile
import threading
import time
from multiprocessing.shared_memory import SharedMemory
from controller import Robot
from PIL import Image

_SHM_HEADER = 8  # [seq:uint32][length:uint32]
_SHM_SIZE = 512 * 1024 + _SHM_HEADER

MAX_SPEED = 16.0


def _run_camera_encoder(cam_w, cam_h, shm_cam, robot_name, tmp, in_queue):
    seq = 0
    while True:
        try:
            raw = in_queue.get(timeout=1.0)
        except queue.Empty:
            continue

        if raw is None:
            break

        try:
            img = Image.frombytes("RGBA", (cam_w, cam_h), raw, "raw", "BGRA")
            buf = io.BytesIO()
            img.convert("RGB").save(buf, format="JPEG", quality=75)
            jpeg = buf.getvalue()

            if shm_cam is not None:
                length = len(jpeg)
                if length < _SHM_SIZE - _SHM_HEADER:
                    shm_cam.buf[_SHM_HEADER:_SHM_HEADER + length] = jpeg
                    struct.pack_into("<I", shm_cam.buf, 4, length)
                    seq += 1
                    struct.pack_into("<I", shm_cam.buf, 0, seq)
                else:
                    print(f"[{robot_name}] JPEG too large for shm")
            else:
                cam_file = os.path.join(tmp, f"webots_{robot_name}_camera.jpg")
                tmp_file = cam_file + ".tmp"
                with open(tmp_file, "wb") as f:
                    f.write(jpeg)
                os.replace(tmp_file, cam_file)

        except Exception as e:
            print(f"[{robot_name}] camera encoding error: {e}")


def safe_device(robot, name):
    try:
        return robot.getDevice(name)
    except:
        return None


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    robot_name = robot.getName()

    # ─────────────────────────────
    # MOTORS
    # ─────────────────────────────
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")

    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    current_sl = 0.0
    current_sr = 0.0

    # ─────────────────────────────
    # ENCODERS
    # ─────────────────────────────
    left_enc = safe_device(robot, "left wheel sensor")
    right_enc = safe_device(robot, "right wheel sensor")

    if left_enc:
        left_enc.enable(timestep)
    if right_enc:
        right_enc.enable(timestep)

    # ─────────────────────────────
    # BUMPERS
    # ─────────────────────────────
    bumper_left = safe_device(robot, "bumper_left")
    bumper_right = safe_device(robot, "bumper_right")

    if bumper_left:
        bumper_left.enable(timestep)
    if bumper_right:
        bumper_right.enable(timestep)

    # ─────────────────────────────
    # CLIFF
    # ─────────────────────────────
    cliff_names = ["cliff_left", "cliff_front_left", "cliff_front_right", "cliff_right"]
    cliff_sensors = []

    for n in cliff_names:
        s = safe_device(robot, n)
        if s:
            s.enable(timestep)
            cliff_sensors.append(s)

    # ─────────────────────────────
    # CAMERA
    # ─────────────────────────────
    camera = safe_device(robot, "front_camera")
    if camera:
        camera.enable(timestep)

    # ─────────────────────────────
    # SHM CAMERA
    # ─────────────────────────────
    shm_cam = None
    if camera:
        shm_name = f"webots_{robot_name}_camera_shm"
        for _ in range(20):
            try:
                shm_cam = SharedMemory(name=shm_name, create=False)
                break
            except:
                time.sleep(0.1)

    # ─────────────────────────────
    # LIDAR
    # ─────────────────────────────
    lidar = safe_device(robot, "lidar")
    if lidar:
        lidar.enable(timestep)

    # ─────────────────────────────
    # SONAR (NEW)
    # ─────────────────────────────
    infrared_front = safe_device(robot, "front_sensor")
    if infrared_front:
        infrared_front.enable(timestep)

    # ─────────────────────────────
    # GPS / IMU / COMPASS
    # ─────────────────────────────
    gps = safe_device(robot, "gps")
    compass = safe_device(robot, "compass")
    imu = safe_device(robot, "inertial_unit")
    gyro = safe_device(robot, "gyro")
    accel = safe_device(robot, "accelerometer")

    if gps:
        gps.enable(timestep)
    if compass:
        compass.enable(timestep)
    if imu:
        imu.enable(timestep)
    if gyro:
        gyro.enable(timestep)
    if accel:
        accel.enable(timestep)

    # ─────────────────────────────
    # FILE
    # ─────────────────────────────
    tmp = tempfile.gettempdir()
    state_file = os.path.join(tmp, f"webots_{robot_name}_state.json")

    # ─────────────────────────────
    # CAMERA THREAD
    # ─────────────────────────────
    cam_queue = None
    if camera:
        cam_queue = queue.Queue(maxsize=1)

        threading.Thread(
            target=_run_camera_encoder,
            args=(camera.getWidth(), camera.getHeight(), shm_cam, robot_name, tmp, cam_queue),
            daemon=True,
        ).start()

    # ─────────────────────────────
    # MAIN LOOP
    # ─────────────────────────────
    while robot.step(timestep) != -1:

        # COMMANDS
        custom = robot.getCustomData()
        if custom:
            try:
                cmd = json.loads(custom)
                if "speed_left" in cmd and "speed_right" in cmd:
                    current_sl = max(-MAX_SPEED, min(MAX_SPEED, float(cmd["speed_left"])))
                    current_sr = max(-MAX_SPEED, min(MAX_SPEED, float(cmd["speed_right"])))
            except:
                pass

        left_motor.setVelocity(current_sl)
        right_motor.setVelocity(current_sr)

        # ── SONAR READ ─────────
        sonar_front_value = infrared_front.getValue() if infrared_front else None

        # ── STATE ──────────────
        state = {
            "name": robot_name,
            "time": robot.getTime(),

            "wheel_left": left_enc.getValue() if left_enc else None,
            "wheel_right": right_enc.getValue() if right_enc else None,

            "bumper_left": bumper_left.getValue() != 0.0 if bumper_left else None,
            "bumper_right": bumper_right.getValue() != 0.0 if bumper_right else None,

            "cliff": [s.getValue() for s in cliff_sensors] if cliff_sensors else None,

            # NEW SONAR (meters)
            "sonar_front": sonar_front_value,

            "gps": list(gps.getValues()) if gps else None,
            "compass": list(compass.getValues()) if compass else None,
            "imu_rpy": list(imu.getRollPitchYaw()) if imu else None,
            "gyro": list(gyro.getValues()) if gyro else None,
            "accel": list(accel.getValues()) if accel else None,
        }

        tmp_state_file = state_file + ".tmp"
        with open(tmp_state_file, "w") as f:
            json.dump(state, f)
        os.replace(tmp_state_file, state_file)

        # CAMERA PIPELINE
        if camera and cam_queue:
            try:
                raw = camera.getImage()
                try:
                    cam_queue.put_nowait(raw)
                except queue.Full:
                    pass
            except:
                pass

    # CLEANUP
    if shm_cam:
        shm_cam.close()


if __name__ == "__main__":
    main()
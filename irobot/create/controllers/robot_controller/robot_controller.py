"""
iRobot Create controller (PERSISTENT SPEED FIX)
Exports sensor data + applies speed commands from customData continuously
"""

import io
import json
import os
import struct
import tempfile
import time
from multiprocessing.shared_memory import SharedMemory
from controller import Robot
from PIL import Image

_SHM_HEADER = 8  # header: [seq: uint32 LE][length: uint32 LE]
_SHM_SIZE = 512 * 1024 + _SHM_HEADER  # 512 KB payload + header

MAX_SPEED = 16.0


def safe_device(robot, name):
    """Return device or None if not available"""
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

    # 📋 VARIABLES DE VITESSE PERSISTANTES
    # Elles évitent que le robot s'arrête si customData devient vide ou s'efface
    current_sl = 0.0
    current_sr = 0.0

    # ─────────────────────────────
    # WHEEL ENCODERS
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
    # CLIFF SENSORS
    # ─────────────────────────────
    cliff_names = [
        "cliff_left",
        "cliff_front_left",
        "cliff_front_right",
        "cliff_right"
    ]

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
    # SHARED MEMORY (camera IPC)
    # ─────────────────────────────
    shm_cam = None
    shm_seq = 0
    if camera:
        shm_name = f"webots_{robot_name}_camera_shm"
        for _ in range(20):
            try:
                shm_cam = SharedMemory(name=shm_name, create=False)
                break
            except FileNotFoundError:
                time.sleep(0.1)
        if shm_cam is None:
            print(f"[robot_controller:{robot_name}] shm not found, falling back to file")

    # ─────────────────────────────
    # LIDAR
    # ─────────────────────────────
    lidar = safe_device(robot, "lidar")
    if lidar:
        lidar.enable(timestep)

    # ─────────────────────────────
    # GPS / ORIENTATION / IMU
    # ─────────────────────────────
    gps = robot.getDevice("gps")
    compass = robot.getDevice("compass")
    imu = robot.getDevice("inertial_unit")
    gyro = robot.getDevice("gyro")
    accel = robot.getDevice("accelerometer")

    gps.enable(timestep)
    compass.enable(timestep)
    imu.enable(timestep)
    gyro.enable(timestep)
    accel.enable(timestep)

    # ─────────────────────────────
    # OUTPUT FILE
    # ─────────────────────────────
    tmp = tempfile.gettempdir()
    state_file = os.path.join(tmp, f"webots_{robot_name}_state.json")

    # ─────────────────────────────
    # MAIN LOOP
    # ─────────────────────────────
    while robot.step(timestep) != -1:

        # ── APPLY COMMANDS ─────────
        custom = robot.getCustomData()
        if custom:
            try:
                cmd = json.loads(custom)

                # On ne met à jour les vitesses QUE si les clés sont présentes dans le JSON
                if "speed_left" in cmd and "speed_right" in cmd:
                    current_sl = max(-MAX_SPEED, min(MAX_SPEED, float(cmd.get("speed_left", 0))))
                    current_sr = max(-MAX_SPEED, min(MAX_SPEED, float(cmd.get("speed_right", 0))))
            except:
                # Si le JSON est mal formé ou temporairement vide pendant l'écriture,
                # on ne fait rien et on garde la vitesse précédente (pas de coup de frein !)
                pass

        # 🔥 On applique en continu la dernière vitesse connue valide
        left_motor.setVelocity(current_sl)
        right_motor.setImage(current_sr) if hasattr(right_motor, 'setImage') else right_motor.setVelocity(current_sr)

        # ── STATE BUILD ───────────
        state = {
            "name": robot_name,
            "time": robot.getTime(),

            # wheels
            "wheel_left": left_enc.getValue() if left_enc else None,
            "wheel_right": right_enc.getValue() if right_enc else None,

            # bumpers
            "bumper_left": bumper_left.getValue() != 0.0 if bumper_left else None,
            "bumper_right": bumper_right.getValue() != 0.0 if bumper_right else None,

            # cliffs
            "cliff": [s.getValue() for s in cliff_sensors] if cliff_sensors else None,

            # GPS
            "gps": gps.getValues(),

            # orientation
            "compass": compass.getValues(),
            "imu_rpy": imu.getRollPitchYaw(),

            # dynamics
            "gyro": gyro.getValues(),
            "accel": accel.getValues(),
        }

        # ── WRITE JSON ─────────────
        with open(state_file, "w") as f:
            json.dump(state, f)

        # ── WRITE CAMERA ──────────
        if camera:
            try:
                raw = camera.getImage()
                img = Image.frombytes(
                    "RGBA",
                    (camera.getWidth(), camera.getHeight()),
                    raw,
                    "raw",
                    "BGRA",
                )
                buf = io.BytesIO()
                img.convert("RGB").save(buf, format="JPEG", quality=75)
                jpeg = buf.getvalue()

                if shm_cam is not None:
                    length = len(jpeg)
                    if length < _SHM_SIZE - _SHM_HEADER:
                        shm_cam.buf[_SHM_HEADER:_SHM_HEADER + length] = jpeg
                        struct.pack_into("<I", shm_cam.buf, 4, length)
                        shm_seq += 1
                        struct.pack_into("<I", shm_cam.buf, 0, shm_seq)
                    else:
                        print(f"[robot_controller:{robot_name}] JPEG too large for shm ({length} B), skipping frame")
                else:
                    cam_file = os.path.join(tmp, f"webots_{robot_name}_camera.jpg")
                    tmp_file = cam_file + ".tmp"
                    with open(tmp_file, "wb") as f:
                        f.write(jpeg)
                    os.replace(tmp_file, cam_file)
            except (OSError, ValueError, RuntimeError) as e:
                print(f"[robot_controller:{robot_name}] camera encoding error: {e}")

    # ── CLEANUP ────────────────
    if shm_cam is not None:
        shm_cam.close()


if __name__ == "__main__":
    main()
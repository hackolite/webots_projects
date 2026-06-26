"""
iRobot Create controller
Exports sensor data + applies speed commands from customData
"""

import io
import json
import os
import tempfile
from controller import Robot
from PIL import Image


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
    # BUMPERS (optionnels)
    # ─────────────────────────────
    bumper_left = safe_device(robot, "bumper_left")
    bumper_right = safe_device(robot, "bumper_right")

    if bumper_left:
        bumper_left.enable(timestep)
    if bumper_right:
        bumper_right.enable(timestep)

    # ─────────────────────────────
    # CLIFF SENSORS (optionnels)
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
    # LI DAR (IMPORTANT)
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

                sl = max(-MAX_SPEED, min(MAX_SPEED, float(cmd.get("speed_left", 0))))
                sr = max(-MAX_SPEED, min(MAX_SPEED, float(cmd.get("speed_right", 0))))

                left_motor.setVelocity(sl)
                right_motor.setVelocity(sr)

            except:
                pass

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

            # lidar (NEW)
            #"lidar": lidar.getRangeImage() if lidar else None,
        }

        # ── WRITE JSON ─────────────
        with open(state_file, "w") as f:
            json.dump(state, f)

        # ── WRITE CAMERA JPEG ──────
        if camera:
            try:
                raw = camera.getImage()   # BGRA bytes
                img = Image.frombytes(
                    "RGBA",
                    (camera.getWidth(), camera.getHeight()),
                    raw,
                    "raw",
                    "BGRA",
                )
                buf = io.BytesIO()
                img.convert("RGB").save(buf, format="JPEG", quality=75)
                cam_file = os.path.join(tmp, f"webots_{robot_name}_camera.jpg")
                with open(cam_file, "wb") as f:
                    f.write(buf.getvalue())
            except (OSError, ValueError, RuntimeError) as e:
                print(f"[robot_controller] camera encoding error: {e}")


if __name__ == "__main__":
    main()
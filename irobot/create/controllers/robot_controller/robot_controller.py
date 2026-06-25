"""
iRobot Create controller with REST API support.

Reads movement commands from the robot's customData field (written by api_supervisor)
and publishes sensor state and camera images to /tmp for the supervisor to read.

Command format (JSON in customData):
  {"speed_left": <float>, "speed_right": <float>}
"""

import io
import json
import os
import tempfile

from controller import Robot

try:
    from PIL import Image
except ImportError:
    import subprocess, sys
    subprocess.check_call([sys.executable, "-m", "pip", "install", "Pillow", "-q"])
    from PIL import Image  # noqa: E402

MAX_SPEED = 16.0
JPEG_QUALITY = 75   # quality / speed trade-off for streaming


def _encode_jpeg(camera, quality: int = JPEG_QUALITY) -> bytes:
    """Encode the camera's current frame to JPEG bytes in memory (no disk I/O)."""
    raw = camera.getImage()   # BGRA bytes
    img = Image.frombytes(
        "RGBA",
        (camera.getWidth(), camera.getHeight()),
        raw,
        "raw",
        "BGRA",
    )
    buf = io.BytesIO()
    img.convert("RGB").save(buf, format="JPEG", quality=quality, optimize=False)
    return buf.getvalue()


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    robot_name = robot.getName()   # "ROBOT_1" or "ROBOT_2"

    # ── Motors ──────────────────────────────────────────────────────────────
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # ── Wheel encoders ──────────────────────────────────────────────────────
    left_sensor = robot.getDevice("left wheel sensor")
    right_sensor = robot.getDevice("right wheel sensor")
    left_sensor.enable(timestep)
    right_sensor.enable(timestep)

    # ── Bump sensors ────────────────────────────────────────────────────────
    bumper_left = robot.getDevice("bumper_left")
    bumper_right = robot.getDevice("bumper_right")
    bumper_left.enable(timestep)
    bumper_right.enable(timestep)

    # ── Cliff sensors ───────────────────────────────────────────────────────
    cliff_names = ["cliff_left", "cliff_front_left", "cliff_front_right", "cliff_right"]
    cliff_sensors = []
    for name in cliff_names:
        s = robot.getDevice(name)
        s.enable(timestep)
        cliff_sensors.append(s)

    # ── Camera ──────────────────────────────────────────────────────────────
    camera = robot.getDevice("front_camera")
    camera.enable(timestep)

    # ── Temp file paths ─────────────────────────────────────────────────────
    _tmp = tempfile.gettempdir()
    state_file = os.path.join(_tmp, f"webots_{robot_name}_state.json")
    camera_file = os.path.join(_tmp, f"webots_{robot_name}_camera.jpg")

    while robot.step(timestep) != -1:
        # ── Apply speed command from supervisor ──────────────────────────────
        custom_data = robot.getCustomData()
        if custom_data:
            try:
                cmd = json.loads(custom_data)
                sl = max(-MAX_SPEED, min(MAX_SPEED, float(cmd.get("speed_left", 0.0))))
                sr = max(-MAX_SPEED, min(MAX_SPEED, float(cmd.get("speed_right", 0.0))))
                left_motor.setVelocity(sl)
                right_motor.setVelocity(sr)
            except (json.JSONDecodeError, KeyError, ValueError):
                pass

        # ── Write sensor state ───────────────────────────────────────────────
        state = {
            "name": robot_name,
            "time": robot.getTime(),
            "wheel_left": round(left_sensor.getValue(), 4),
            "wheel_right": round(right_sensor.getValue(), 4),
            "bumper_left": bumper_left.getValue() != 0.0,
            "bumper_right": bumper_right.getValue() != 0.0,
            "cliff": [round(s.getValue(), 2) for s in cliff_sensors],
        }
        try:
            tmp = state_file + ".tmp"
            with open(tmp, "w") as f:
                json.dump(state, f)
            os.replace(tmp, state_file)
        except OSError:
            pass

        # ── Write camera frame to file every step ────────────────────────────
        try:
            jpeg_bytes = _encode_jpeg(camera)
            tmp_path = camera_file + ".tmp"
            with open(tmp_path, "wb") as f:
                f.write(jpeg_bytes)
            os.replace(tmp_path, camera_file)
        except OSError as e:
            print(f"[robot_controller] camera I/O error: {e}")
        except Exception as e:
            print(f"[robot_controller] camera encoding error: {e}")


if __name__ == "__main__":
    main()

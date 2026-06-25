"""
iRobot Create controller with REST API support.

Reads movement commands from the robot's customData field (written by api_supervisor)
and publishes sensor state and camera images to /tmp for the supervisor to read.

Command format (JSON in customData):
  {"speed_left": <float>, "speed_right": <float>}
"""

import json
import os
import sys

from controller import Robot

MAX_SPEED = 16.0
CAMERA_SAVE_PERIOD = 5   # save camera every N timesteps to reduce I/O


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
    state_file = f"/tmp/webots_{robot_name}_state.json"
    camera_file = f"/tmp/webots_{robot_name}_camera.jpg"

    step_counter = 0

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

        # ── Save camera image periodically ───────────────────────────────────
        if step_counter % CAMERA_SAVE_PERIOD == 0:
            try:
                camera.saveImage(camera_file, 90)
            except Exception:
                pass

        step_counter += 1


if __name__ == "__main__":
    main()

"""
iRobot Create controller for the *scooting* project.

Mirrors :mod:`tiago_lite` but drives an iRobot Create using its bumpers and
cliff sensors for obstacle avoidance.

Command protocol (Webots ``customData``, written by ``api_supervisor``)::

    {"mode": "auto" | "manual" | "stopped",
     "speed_left": <float>, "speed_right": <float>}

* ``auto``    – built-in bumper/cliff obstacle avoidance (default).
* ``manual``  – apply ``speed_left`` / ``speed_right`` verbatim.
* ``stopped`` – hold still.

Each step the controller publishes its ``front_camera`` frame (shared memory)
and a JSON sensor snapshot, exactly like the sibling ``create`` project.
"""

import json
import random

from controller import Robot

from scooting_io import CameraPublisher, write_sensors

# ── Constants ────────────────────────────────────────────────────────────────
CRUISE_SPEED = 8.0        # rad/s forward cruising speed
TURN_SPEED = 5.0          # rad/s in-place turn speed
CLIFF_THRESHOLD = 100.0   # cliff sensor value below which a drop is detected

CLIFF_NAMES = ("cliff_left", "cliff_front_left", "cliff_front_right", "cliff_right")


def safe_device(robot, name):
    try:
        return robot.getDevice(name)
    except Exception:
        return None


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    robot_name = robot.getName()

    # ── Motors ───────────────────────────────────────────────────────────────
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    for m in (left_motor, right_motor):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    max_speed = min(left_motor.getMaxVelocity(), right_motor.getMaxVelocity())

    # ── Wheel encoders ───────────────────────────────────────────────────────
    left_enc = safe_device(robot, "left wheel sensor")
    right_enc = safe_device(robot, "right wheel sensor")
    for enc in (left_enc, right_enc):
        if enc:
            enc.enable(timestep)

    # ── Bumpers ──────────────────────────────────────────────────────────────
    bumper_left = safe_device(robot, "bumper_left")
    bumper_right = safe_device(robot, "bumper_right")
    for b in (bumper_left, bumper_right):
        if b:
            b.enable(timestep)

    # ── Cliff sensors ────────────────────────────────────────────────────────
    cliffs = []
    for name in CLIFF_NAMES:
        s = safe_device(robot, name)
        if s:
            s.enable(timestep)
        cliffs.append(s)

    # ── Camera ───────────────────────────────────────────────────────────────
    camera = safe_device(robot, "front_camera")
    if camera:
        camera.enable(timestep)
    cam_pub = CameraPublisher(robot_name)

    # Persistent turn counter so an avoidance manoeuvre lasts a few steps.
    turn_steps = 0
    turn_dir = 1.0   # +1 turn left, -1 turn right

    def hazards():
        bl = bool(bumper_left and bumper_left.getValue() != 0.0)
        br = bool(bumper_right and bumper_right.getValue() != 0.0)
        cl = bool(cliffs[0] and cliffs[0].getValue() < CLIFF_THRESHOLD) or \
            bool(cliffs[1] and cliffs[1].getValue() < CLIFF_THRESHOLD)
        cr = bool(cliffs[3] and cliffs[3].getValue() < CLIFF_THRESHOLD) or \
            bool(cliffs[2] and cliffs[2].getValue() < CLIFF_THRESHOLD)
        return bl, br, cl, cr

    while robot.step(timestep) != -1:
        # ── Decode command ──────────────────────────────────────────────────
        mode = "auto"
        cmd_sl = cmd_sr = 0.0
        custom = robot.getCustomData()
        if custom:
            try:
                cmd = json.loads(custom)
                mode = cmd.get("mode", "auto")
                cmd_sl = float(cmd.get("speed_left", 0.0))
                cmd_sr = float(cmd.get("speed_right", 0.0))
            except (ValueError, TypeError):
                mode = "auto"

        bl, br, cl, cr = hazards()

        # ── Decide wheel speeds ─────────────────────────────────────────────
        if mode == "stopped":
            sl = sr = 0.0
            turn_steps = 0
        elif mode == "manual":
            sl, sr = cmd_sl, cmd_sr
            turn_steps = 0
        else:  # auto → obstacle avoidance
            if turn_steps > 0:
                sl, sr = -turn_dir * TURN_SPEED, turn_dir * TURN_SPEED
                turn_steps -= 1
            elif bl or br or cl or cr:
                # Start a new avoidance turn away from the hazard.
                turn_dir = -1.0 if (br or cr) else 1.0
                turn_steps = int(30 + 30 * random.random())
                sl, sr = -turn_dir * TURN_SPEED, turn_dir * TURN_SPEED
            else:
                sl, sr = CRUISE_SPEED, CRUISE_SPEED

        sl = max(-max_speed, min(max_speed, sl))
        sr = max(-max_speed, min(max_speed, sr))
        left_motor.setVelocity(sl)
        right_motor.setVelocity(sr)

        # ── Publish sensors + camera ────────────────────────────────────────
        write_sensors(robot_name, {
            "name": robot_name,
            "time": robot.getTime(),
            "mode": mode,
            "speed_left": sl,
            "speed_right": sr,
            "wheel_left": left_enc.getValue() if left_enc else None,
            "wheel_right": right_enc.getValue() if right_enc else None,
            "bumper_left": bl,
            "bumper_right": br,
            "cliff": [s.getValue() if s else None for s in cliffs],
        })
        cam_pub.publish(camera)

    cam_pub.close()


if __name__ == "__main__":
    main()

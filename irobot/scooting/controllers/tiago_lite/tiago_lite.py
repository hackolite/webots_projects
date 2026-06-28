"""
TiagoLite controller for the *scooting* project.

Behaviour
─────────
The robot reads a small JSON command document from its Webots ``customData``
field (written by ``api_supervisor``)::

    {"mode": "auto" | "manual" | "stopped",
     "speed_left": <float>, "speed_right": <float>}

* ``auto``    – run the built-in lidar obstacle-avoidance behaviour with
                full iRobot-style state machine (FORWARD / TURN / BACKUP).
* ``manual``  – apply ``speed_left`` / ``speed_right`` verbatim (avoidance off).
* ``stopped`` – hold still (avoidance off).

Every step the controller also publishes its ``Astra rgb`` camera frame
(shared memory, consumed by the supervisor) and a JSON sensor snapshot.
"""

import json
import math
import random

from controller import Robot

from scooting_io import CameraPublisher, write_sensors

# ── Constants ────────────────────────────────────────────────────────────────
CRUISE_SPEED      = 4.0   # rad/s forward cruising speed
TURN_SPEED        = 3.0   # rad/s in-place turn speed
BACKUP_SPEED      = 2.0   # rad/s reverse speed

SAFE_DISTANCE     = 0.8   # m – start turning
DANGER_DISTANCE   = 0.5   # m – back up first

FRONT_HALF_ANGLE  = 0.5   # rad – half-width of the front sector (avoidance)
SIDE_HALF_ANGLE   = 1.0   # rad – wider sector used to pick best turn direction


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
    left_motor  = robot.getDevice("wheel_left_joint")
    right_motor = robot.getDevice("wheel_right_joint")
    for m in (left_motor, right_motor):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    max_speed = min(left_motor.getMaxVelocity(), right_motor.getMaxVelocity())

    # ── Wheel encoders ───────────────────────────────────────────────────────
    left_enc  = safe_device(robot, "wheel_left_joint_sensor")
    right_enc = safe_device(robot, "wheel_right_joint_sensor")
    for enc in (left_enc, right_enc):
        if enc:
            enc.enable(timestep)

    # ── Inertial sensors ─────────────────────────────────────────────────────
    imu   = safe_device(robot, "inertial unit")
    gyro  = safe_device(robot, "gyro")
    accel = safe_device(robot, "accelerometer")
    for dev in (imu, gyro, accel):
        if dev:
            dev.enable(timestep)

    # ── Front lidar ──────────────────────────────────────────────────────────
    lidar     = safe_device(robot, "Hokuyo URG-04LX-UG01")
    lidar_res = 0
    lidar_fov = 0.0
    if lidar:
        lidar.enable(timestep)
        lidar_res = lidar.getHorizontalResolution()
        lidar_fov = lidar.getFov()

    # ── Camera ───────────────────────────────────────────────────────────────
    camera = safe_device(robot, "Astra rgb")
    if camera:
        camera.enable(timestep)
    cam_pub = CameraPublisher(robot_name)

    # ── iRobot-style state machine ───────────────────────────────────────────
    STATE_FORWARD = "FORWARD"
    STATE_TURN    = "TURN"
    STATE_BACKUP  = "BACKUP"

    state       = STATE_FORWARD
    state_timer = 0
    turn_dir    = 1    # +1 = left,  -1 = right
    turn_steps  = 0

    # Durations in simulation steps
    BACKUP_STEPS   = int(0.8  * 1000 / timestep)
    MIN_TURN_STEPS = int(0.5  * 1000 / timestep)
    MAX_TURN_STEPS = int(2.5  * 1000 / timestep)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _scan_sector(ranges, max_range, center, per_index, half_angle):
        """Return min distance within ±half_angle of center index."""
        vals = []
        for i, d in enumerate(ranges):
            if d != d or d == float("inf"):
                d = max_range
            angle = (i - center) * per_index
            if abs(angle) <= half_angle:
                vals.append(d)
        return min(vals) if vals else float("inf")

    def front_distances():
        """Return (front_min, left_clearance, right_clearance) in metres."""
        if not lidar or lidar_res <= 0 or lidar_fov <= 0:
            return float("inf"), 0.0, 0.0
        ranges = lidar.getRangeImage()
        if not ranges:
            return float("inf"), 0.0, 0.0
        max_range  = lidar.getMaxRange()
        center     = lidar_res / 2.0
        per_index  = lidar_fov / lidar_res

        front_min = float("inf")
        left_sum  = 0.0
        right_sum = 0.0
        for i, d in enumerate(ranges):
            if d != d or d == float("inf"):
                d = max_range
            angle = (i - center) * per_index   # <0 right, >0 left
            if abs(angle) <= FRONT_HALF_ANGLE:
                front_min = min(front_min, d)
            if 0 < angle <= SIDE_HALF_ANGLE:
                left_sum += d
            elif -SIDE_HALF_ANGLE <= angle < 0:
                right_sum += d

        return front_min, left_sum, right_sum

    def set_speed(left, right):
        left_motor.setVelocity( max(-max_speed, min(max_speed, left)))
        right_motor.setVelocity(max(-max_speed, min(max_speed, right)))

    def auto_avoid():
        """
        iRobot-style state machine.
        Returns (left_speed, right_speed).
        """
        nonlocal state, state_timer, turn_dir, turn_steps

        state_timer += 1
        front_min, left_clear, right_clear = front_distances()
        best_dir = 1 if left_clear >= right_clear else -1   # turn toward free side

        if state == STATE_FORWARD:
            if front_min < DANGER_DISTANCE:
                state       = STATE_BACKUP
                state_timer = 0
                return -BACKUP_SPEED, -BACKUP_SPEED
            elif front_min < SAFE_DISTANCE:
                state       = STATE_TURN
                state_timer = 0
                turn_dir    = best_dir
                turn_steps  = random.randint(MIN_TURN_STEPS, MAX_TURN_STEPS)
                return TURN_SPEED * turn_dir, -TURN_SPEED * turn_dir
            else:
                return CRUISE_SPEED, CRUISE_SPEED

        elif state == STATE_BACKUP:
            if state_timer >= BACKUP_STEPS:
                state       = STATE_TURN
                state_timer = 0
                turn_dir    = best_dir
                turn_steps  = random.randint(MIN_TURN_STEPS, MAX_TURN_STEPS)
                return TURN_SPEED * turn_dir, -TURN_SPEED * turn_dir
            return -BACKUP_SPEED, -BACKUP_SPEED

        elif state == STATE_TURN:
            if front_min < DANGER_DISTANCE:
                state       = STATE_BACKUP
                state_timer = 0
                return -BACKUP_SPEED, -BACKUP_SPEED
            if state_timer >= turn_steps and front_min >= SAFE_DISTANCE:
                state       = STATE_FORWARD
                state_timer = 0
                return CRUISE_SPEED, CRUISE_SPEED
            return TURN_SPEED * turn_dir, -TURN_SPEED * turn_dir

        return 0.0, 0.0   # safety fallback

    # ── Main loop ─────────────────────────────────────────────────────────────
    while robot.step(timestep) != -1:

        # ── Decode command ───────────────────────────────────────────────────
        mode   = "auto"
        cmd_sl = cmd_sr = 0.0
        custom = robot.getCustomData()
        if custom:
            try:
                cmd    = json.loads(custom)
                mode   = cmd.get("mode", "auto")
                cmd_sl = float(cmd.get("speed_left",  0.0))
                cmd_sr = float(cmd.get("speed_right", 0.0))
            except (ValueError, TypeError):
                mode = "auto"

        # ── Decide wheel speeds ──────────────────────────────────────────────
        if mode == "stopped":
            sl = sr = 0.0
            set_speed(sl, sr)
        elif mode == "manual":
            sl, sr = cmd_sl, cmd_sr
            set_speed(sl, sr)
        else:   # auto → iRobot-style state machine
            sl, sr = auto_avoid()
            set_speed(sl, sr)

        # ── Publish sensors + camera ─────────────────────────────────────────
        front_min, _, _ = front_distances()
        write_sensors(robot_name, {
            "name":           robot_name,
            "time":           robot.getTime(),
            "mode":           mode,
            "state":          state,          # FORWARD / TURN / BACKUP
            "speed_left":     sl,
            "speed_right":    sr,
            "wheel_left":     left_enc.getValue()      if left_enc  else None,
            "wheel_right":    right_enc.getValue()     if right_enc else None,
            "front_distance": None if front_min == float("inf") else front_min,
            "imu_rpy":        imu.getRollPitchYaw()   if imu       else None,
            "gyro":           gyro.getValues()         if gyro      else None,
            "accel":          accel.getValues()        if accel     else None,
        })
        cam_pub.publish(camera)

    cam_pub.close()


if __name__ == "__main__":
    main()

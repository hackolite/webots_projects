"""
TiagoLite controller for the *scooting* project.

Behaviour
─────────
The robot reads a small JSON command document from its Webots ``customData``
field (written by ``api_supervisor``)::

    {"mode": "auto" | "manual" | "stopped",
     "speed_left": <float>, "speed_right": <float>}

* ``auto``    – run the built-in lidar obstacle-avoidance behaviour (default).
* ``manual``  – apply ``speed_left`` / ``speed_right`` verbatim (avoidance off).
* ``stopped`` – hold still (avoidance off).

Every step the controller also publishes its ``Astra rgb`` camera frame (shared
memory, consumed by the supervisor) and a JSON sensor snapshot.
"""

import json

from controller import Robot

from scooting_io import CameraPublisher, write_sensors

# ── Constants ────────────────────────────────────────────────────────────────
CRUISE_SPEED = 4.0        # rad/s forward cruising speed
TURN_SPEED = 3.0          # rad/s in-place turn speed
SAFE_DISTANCE = 0.8       # m – obstacle threshold in the front lidar sector
FRONT_HALF_ANGLE = 0.5    # rad – half-width of the front sector to inspect


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
    left_motor = robot.getDevice("wheel_left_joint")
    right_motor = robot.getDevice("wheel_right_joint")
    for m in (left_motor, right_motor):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    max_speed = min(left_motor.getMaxVelocity(), right_motor.getMaxVelocity())

    # ── Wheel encoders ───────────────────────────────────────────────────────
    left_enc = safe_device(robot, "wheel_left_joint_sensor")
    right_enc = safe_device(robot, "wheel_right_joint_sensor")
    for enc in (left_enc, right_enc):
        if enc:
            enc.enable(timestep)

    # ── Inertial sensors (TiagoBase) ─────────────────────────────────────────
    imu = safe_device(robot, "inertial unit")
    gyro = safe_device(robot, "gyro")
    accel = safe_device(robot, "accelerometer")
    for dev in (imu, gyro, accel):
        if dev:
            dev.enable(timestep)

    # ── Front lidar (used for obstacle avoidance) ────────────────────────────
    lidar = safe_device(robot, "Hokuyo URG-04LX-UG01")
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

    def front_distances():
        """Return (min_front, left_clearance, right_clearance) in metres."""
        if not lidar or lidar_res <= 0 or lidar_fov <= 0:
            return float("inf"), 0.0, 0.0
        ranges = lidar.getRangeImage()
        if not ranges:
            return float("inf"), 0.0, 0.0
        max_range = lidar.getMaxRange()
        center = lidar_res / 2.0
        per_index = lidar_fov / lidar_res
        front_min = float("inf")
        left_sum = 0.0
        right_sum = 0.0
        for i, d in enumerate(ranges):
            if d != d or d == float("inf"):   # NaN / inf → treat as max range
                d = max_range
            angle = (i - center) * per_index   # <0 right, >0 left
            if abs(angle) <= FRONT_HALF_ANGLE:
                front_min = min(front_min, d)
            if 0 < angle <= FRONT_HALF_ANGLE * 2:
                left_sum += d
            elif -FRONT_HALF_ANGLE * 2 <= angle < 0:
                right_sum += d
        return front_min, left_sum, right_sum

    def avoid():
        """Return (left_speed, right_speed) for autonomous obstacle avoidance."""
        front_min, left_clear, right_clear = front_distances()
        if front_min > SAFE_DISTANCE:
            return CRUISE_SPEED, CRUISE_SPEED
        # Obstacle ahead: rotate towards the side with more free space.
        if left_clear >= right_clear:
            return -TURN_SPEED, TURN_SPEED    # turn left
        return TURN_SPEED, -TURN_SPEED        # turn right

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

        # ── Decide wheel speeds ─────────────────────────────────────────────
        if mode == "stopped":
            sl = sr = 0.0
        elif mode == "manual":
            sl, sr = cmd_sl, cmd_sr
        else:  # auto → obstacle avoidance
            sl, sr = avoid()

        sl = max(-max_speed, min(max_speed, sl))
        sr = max(-max_speed, min(max_speed, sr))
        left_motor.setVelocity(sl)
        right_motor.setVelocity(sr)

        # ── Publish sensors + camera ────────────────────────────────────────
        front_min, _, _ = front_distances()
        write_sensors(robot_name, {
            "name": robot_name,
            "time": robot.getTime(),
            "mode": mode,
            "speed_left": sl,
            "speed_right": sr,
            "wheel_left": left_enc.getValue() if left_enc else None,
            "wheel_right": right_enc.getValue() if right_enc else None,
            "front_distance": None if front_min == float("inf") else front_min,
            "imu_rpy": imu.getRollPitchYaw() if imu else None,
            "gyro": gyro.getValues() if gyro else None,
            "accel": accel.getValues() if accel else None,
        })
        cam_pub.publish(camera)

    cam_pub.close()


if __name__ == "__main__":
    main()

"""
API Supervisor for iRobot Create simulation.

Exposes a REST API on http://localhost:5000 to control and monitor
the robots in the Webots simulation.

Endpoints
─────────
GET  /robots                  → state of all robots
GET  /robots/{id}             → position, sensors, status
GET  /robots/{id}/sensors     → robot internal sensor data only
POST /robots/{id}/move        → { "speed_left": f, "speed_right": f }
POST /robots/{id}/goto        → { "x": f, "z": f }  (autonomous navigation)
POST /robots/{id}/stop        → immediate stop
GET  /robots/{id}/camera      → JPEG image as base64 JSON
GET  /god/camera              → top-down god-view camera image as base64 JSON
GET  /robots/ceiling/camera   → bedroom ceiling camera image as base64 JSON
POST /simulation/pause        → pause the simulation
POST /simulation/resume       → resume the simulation
GET  /simulation/time         → current simulated time

Coordinate note: the world uses X/Y as the horizontal plane (Z is up).
The API "z" parameter in /goto maps to the world Y axis, matching the
Webots VRML convention used in the original scene description.
"""

import base64
import json
import math
import os
import tempfile
import threading
from copy import deepcopy

from controller import Supervisor

try:
    from flask import Flask, jsonify, request
except ImportError:
    import subprocess, sys
    subprocess.check_call([sys.executable, "-m", "pip", "install", "flask", "-q"])
    from flask import Flask, jsonify, request

# ── Constants ────────────────────────────────────────────────────────────────
API_PORT = 5000
MAX_SPEED = 8.0          # conservative max speed for goto navigation (m/s * gearing)
GOTO_ARRIVAL_DIST = 0.15  # metres – consider target reached when within this distance
CAMERA_SAVE_PERIOD = 5    # save ceiling camera every N timesteps

ROBOT_DEFS = {
    "ROBOT_1": "IROBOT_CREATE",   # DEF name in the .wbt file
    "ROBOT_2": "ROBOT_2",
}

# ── Shared state (accessed by Flask thread + main loop) ─────────────────────
_lock = threading.Lock()

_robots: dict = {
    rid: {
        "translation": [0.0, 0.0, 0.0],
        "rotation": [0.0, 0.0, 1.0, 0.0],
        "speed_left": 0.0,
        "speed_right": 0.0,
        "goto_target": None,   # [world_x, world_y] or None
        "status": "idle",
        "sensors": {},
    }
    for rid in ROBOT_DEFS
}

_sim_time: float = 0.0
_paused: bool = False
_pending_pause: bool = False
_pending_resume: bool = False

_god_camera_file = os.path.join(tempfile.gettempdir(), "webots_god_camera.jpg")
_ceiling_camera_file = os.path.join(tempfile.gettempdir(), "webots_ceiling_camera.jpg")


# ── Helper: extract yaw angle from Webots axis-angle rotation ────────────────

def _yaw_from_rotation(rot):
    """Return yaw (radians, around world Z) from a Webots [ax, ay, az, angle] tuple."""
    ax, ay, az, angle = rot
    s = math.sin(angle / 2.0)
    c = math.cos(angle / 2.0)
    qx, qy, qz, qw = ax * s, ay * s, az * s, c
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


# ── Goto navigation (proportional controller) ────────────────────────────────

def _compute_goto_speeds(pos, rot, target):
    """Return (speed_left, speed_right) to steer robot toward *target* [wx, wy]."""
    dx = target[0] - pos[0]
    dy = target[1] - pos[1]
    dist = math.sqrt(dx * dx + dy * dy)

    if dist < GOTO_ARRIVAL_DIST:
        return 0.0, 0.0

    target_angle = math.atan2(dy, dx)
    yaw = _yaw_from_rotation(rot)

    angle_err = target_angle - yaw
    # Normalise to [-π, π]
    while angle_err > math.pi:
        angle_err -= 2.0 * math.pi
    while angle_err < -math.pi:
        angle_err += 2.0 * math.pi

    # Simple proportional: turn harder when facing wrong way
    turn = 4.0 * angle_err
    forward = min(MAX_SPEED, dist * 3.0) * max(0.0, 1.0 - abs(angle_err) / math.pi)

    sl = forward - turn
    sr = forward + turn
    sl = max(-MAX_SPEED, min(MAX_SPEED, sl))
    sr = max(-MAX_SPEED, min(MAX_SPEED, sr))
    return sl, sr


# ── Flask application ────────────────────────────────────────────────────────

app = Flask(__name__)

# silence Flask startup banner in Webots console
import logging
log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)


def _robot_public_state(rid):
    """Return a JSON-serialisable snapshot of a robot's state."""
    with _lock:
        r = deepcopy(_robots[rid])
    sensors = {}
    try:
        sfp = os.path.join(tempfile.gettempdir(), f"webots_{rid}_state.json")
        if os.path.exists(sfp):
            with open(sfp) as fh:
                sensors = json.load(fh)
    except (OSError, json.JSONDecodeError):
        pass
    pos = r["translation"]
    rot = r["rotation"]
    return {
        "id": rid,
        "position": {"x": pos[0], "y": pos[1], "z": pos[2]},
        "rotation": {"ax": rot[0], "ay": rot[1], "az": rot[2], "angle": rot[3]},
        "speed_left": r["speed_left"],
        "speed_right": r["speed_right"],
        "status": r["status"],
        "goto_target": (
            {"x": r["goto_target"][0], "z": r["goto_target"][1]}
            if r["goto_target"]
            else None
        ),
        "sensors": sensors,
    }


@app.get("/robots")
def get_robots():
    return jsonify([_robot_public_state(rid) for rid in ROBOT_DEFS])


@app.get("/robots/<rid>")
def get_robot(rid):
    if rid not in ROBOT_DEFS:
        return jsonify({"error": "robot not found"}), 404
    return jsonify(_robot_public_state(rid))


@app.post("/robots/<rid>/move")
def move_robot(rid):
    if rid not in ROBOT_DEFS:
        return jsonify({"error": "robot not found"}), 404
    data = request.get_json(force=True, silent=True) or {}
    sl = float(data.get("speed_left", 0.0))
    sr = float(data.get("speed_right", 0.0))
    with _lock:
        _robots[rid]["speed_left"] = max(-MAX_SPEED, min(MAX_SPEED, sl))
        _robots[rid]["speed_right"] = max(-MAX_SPEED, min(MAX_SPEED, sr))
        _robots[rid]["goto_target"] = None
        _robots[rid]["status"] = "moving"
    return jsonify({"ok": True})


@app.post("/robots/<rid>/goto")
def goto_robot(rid):
    """Navigate autonomously to {x, z} where z maps to world Y axis."""
    if rid not in ROBOT_DEFS:
        return jsonify({"error": "robot not found"}), 404
    data = request.get_json(force=True, silent=True) or {}
    try:
        wx = float(data["x"])
        wy = float(data["z"])   # API "z" → world Y
    except (KeyError, ValueError, TypeError):
        return jsonify({"error": "body must contain numeric x and z"}), 400
    with _lock:
        _robots[rid]["goto_target"] = [wx, wy]
        _robots[rid]["status"] = "navigating"
    return jsonify({"ok": True, "target": {"x": wx, "z": wy}})


@app.post("/robots/<rid>/stop")
def stop_robot(rid):
    if rid not in ROBOT_DEFS:
        return jsonify({"error": "robot not found"}), 404
    with _lock:
        _robots[rid]["speed_left"] = 0.0
        _robots[rid]["speed_right"] = 0.0
        _robots[rid]["goto_target"] = None
        _robots[rid]["status"] = "stopped"
    return jsonify({"ok": True})


@app.get("/robots/<rid>/sensors")
def robot_sensors(rid):
    if rid not in ROBOT_DEFS:
        return jsonify({"error": "robot not found"}), 404
    sensors = {}
    try:
        sfp = os.path.join(tempfile.gettempdir(), f"webots_{rid}_state.json")
        if os.path.exists(sfp):
            with open(sfp) as fh:
                sensors = json.load(fh)
    except (OSError, json.JSONDecodeError):
        pass
    if not sensors:
        return jsonify({"error": "no sensor data available yet"}), 503
    return jsonify({"robot_id": rid, "sensors": sensors})


@app.get("/robots/<rid>/camera")
def robot_camera(rid):
    if rid not in ROBOT_DEFS:
        return jsonify({"error": "robot not found"}), 404
    img_path = os.path.join(tempfile.gettempdir(), f"webots_{rid}_camera.jpg")
    if not os.path.exists(img_path):
        return jsonify({"error": "no image available yet"}), 503
    with open(img_path, "rb") as fh:
        b64 = base64.b64encode(fh.read()).decode()
    return jsonify({"robot_id": rid, "format": "jpeg", "data": b64})


@app.get("/god/camera")
def god_camera():
    if not os.path.exists(_god_camera_file):
        return jsonify({"error": "no image available yet"}), 503
    with open(_god_camera_file, "rb") as fh:
        b64 = base64.b64encode(fh.read()).decode()
    return jsonify({"source": "god_camera", "format": "jpeg", "data": b64})


@app.get("/robots/ceiling/camera")
def ceiling_camera():
    if not os.path.exists(_ceiling_camera_file):
        return jsonify({"error": "no image available yet"}), 503
    with open(_ceiling_camera_file, "rb") as fh:
        b64 = base64.b64encode(fh.read()).decode()
    return jsonify({"source": "ceiling_camera", "format": "jpeg", "data": b64})


@app.post("/simulation/pause")
def sim_pause():
    global _pending_pause
    with _lock:
        _pending_pause = True
    return jsonify({"ok": True})


@app.post("/simulation/resume")
def sim_resume():
    global _pending_resume
    with _lock:
        _pending_resume = True
    return jsonify({"ok": True})


@app.get("/simulation/time")
def sim_time():
    with _lock:
        t = _sim_time
    return jsonify({"time": t})


def _run_flask():
    app.run(host="0.0.0.0", port=API_PORT, debug=False, use_reloader=False)


# ── Main supervisor loop ─────────────────────────────────────────────────────

def main():
    global _sim_time, _paused, _pending_pause, _pending_resume

    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    # ── Get robot nodes ──────────────────────────────────────────────────────
    nodes = {rid: supervisor.getFromDef(def_name) for rid, def_name in ROBOT_DEFS.items()}
    for rid, node in nodes.items():
        if node is None:
            print(f"[api_supervisor] WARNING: DEF '{ROBOT_DEFS[rid]}' not found for {rid}")

    # ── God camera (top-down view) ───────────────────────────────────────────
    god_cam = supervisor.getDevice("god_camera")
    if god_cam:
        god_cam.enable(timestep)
    else:
        print("[api_supervisor] WARNING: god_camera device not found")

    # ── Ceiling camera (bedroom overview) ────────────────────────────────────
    ceiling_cam = supervisor.getDevice("ceiling_camera")
    if ceiling_cam:
        ceiling_cam.enable(timestep)
    else:
        print("[api_supervisor] WARNING: ceiling_camera device not found")

    # ── Start Flask in a background thread ───────────────────────────────────
    flask_thread = threading.Thread(target=_run_flask, daemon=True)
    flask_thread.start()
    print(f"[api_supervisor] REST API listening on http://0.0.0.0:{API_PORT}")

    step_counter = 0

    while supervisor.step(timestep) != -1:
        # ── Handle pause / resume requests ──────────────────────────────────
        with _lock:
            do_pause = _pending_pause
            do_resume = _pending_resume
            _pending_pause = False
            _pending_resume = False

        if do_pause and not _paused:
            supervisor.simulationSetMode(supervisor.SIMULATION_MODE_PAUSE)
            with _lock:
                _paused = True

        if do_resume and _paused:
            supervisor.simulationSetMode(supervisor.SIMULATION_MODE_RUN)
            with _lock:
                _paused = False

        # ── Update simulation time ───────────────────────────────────────────
        with _lock:
            _sim_time = supervisor.getTime()

        # ── Update robot state and apply commands ────────────────────────────
        for rid, node in nodes.items():
            if node is None:
                continue

            # Read position & orientation from simulation
            trans = node.getField("translation").getSFVec3f()
            rot = node.getField("rotation").getSFRotation()

            with _lock:
                _robots[rid]["translation"] = list(trans)
                _robots[rid]["rotation"] = list(rot)

                target = _robots[rid].get("goto_target")
                if target is not None:
                    sl, sr = _compute_goto_speeds(trans, rot, target)
                    if sl == 0.0 and sr == 0.0:
                        # Arrived
                        _robots[rid]["goto_target"] = None
                        _robots[rid]["status"] = "idle"
                    _robots[rid]["speed_left"] = sl
                    _robots[rid]["speed_right"] = sr

                sl = _robots[rid]["speed_left"]
                sr = _robots[rid]["speed_right"]

            # Write command to robot's customData (read by robot_controller.py)
            cmd = json.dumps({"speed_left": sl, "speed_right": sr})
            node.getField("customData").setSFString(cmd)

        # ── Save god camera image periodically ───────────────────────────────
        if god_cam and step_counter % CAMERA_SAVE_PERIOD == 0:
            try:
                god_cam.saveImage(_god_camera_file, 90)
            except Exception:
                pass

        # ── Save ceiling camera image periodically ────────────────────────────
        if ceiling_cam and step_counter % CAMERA_SAVE_PERIOD == 0:
            try:
                ceiling_cam.saveImage(_ceiling_camera_file, 90)
            except Exception:
                pass

        step_counter += 1


if __name__ == "__main__":
    main()

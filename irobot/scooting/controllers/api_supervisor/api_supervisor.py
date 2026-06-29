"""
API Supervisor for the *scooting* Webots simulation.

This controller mirrors the architecture of the sibling ``create`` project:
a single Webots :class:`Supervisor` runs a Flask REST server in a background
thread, tracks every robot in the scene and forwards control commands.

Robots in the scene
───────────────────
Controllable (REST move/stop/start + obstacle avoidance toggle):
    TIAGO_1, TIAGO_2  – PAL Robotics TiagoLite (differential drive)
    CREATE            – iRobot Create (differential drive)
Camera + sensors only (read-only):
    BLIMP             – LIS Blimp (flying)
    RANGEROVER        – Range Rover Sport SVR (land rover)

Every robot owns an on-board camera.  Because a Supervisor can only read its
own devices, each robot controller publishes its camera frame either through a
POSIX shared-memory block (fast, in-memory path used by the Python robots) or a
JPEG file in the temp dir (fallback path used by the C Range Rover controller).
The supervisor consumes those frames and re-serves them as base64 snapshots and
MJPEG live streams.  A top-down ``god_camera`` owned by the supervisor itself
gives a full map overview.

Obstacle avoidance protocol (controllable robots, via ``customData``)
────────────────────────────────────────────────────────────────────
The supervisor writes a small JSON document into each controllable robot's
``customData`` field every step::

    {"mode": "auto" | "manual" | "stopped",
     "speed_left": <float>, "speed_right": <float>}

* ``auto``    – the robot runs its built-in obstacle-avoidance behaviour
                (this is the default on start-up).
* ``manual``  – the robot applies ``speed_left``/``speed_right`` verbatim and
                obstacle avoidance is suspended (set by ``POST /move``).
* ``stopped`` – the robot halts and obstacle avoidance stays suspended
                (set by ``POST /stop``).

``POST /start`` switches the robot back to ``auto`` (resumes avoidance).

Endpoints
─────────
GET /robots
    List all robots.
    Response 200 – JSON array of robot state objects (see GET /robots/{id}).

GET /robots/{id}
    Full state of one robot.
    Path: id = "TIAGO_1" | "TIAGO_2" | "CREATE" | "BLIMP" | "RANGEROVER"
          or numeric aliases "1" / "2" / "3" for the controllable robots.
    Response 200::

        {
          "id":           "TIAGO_1",
          "controllable": true,
          "has_camera":   true,
          "position":     {"x": 0.0, "y": 0.0, "z": 0.0},
          "rotation":     {"ax": 0.0, "ay": 0.0, "az": 1.0, "angle": 0.0},
          "mode":         "auto" | "manual" | "stopped",
          "speed_left":   0.0,
          "speed_right":  0.0,
          "sensors":      { <sensor_key>: <value>, ... }
        }

    Response 404: ``{"error": "robot not found"}``

GET /robots/{id}/sensors
    Raw sensor data published by the robot controller.
    Response 200: ``{"robot_id": str, "sensors": {...}}``
    Response 503: ``{"error": "no sensor data available yet"}``
    Response 404: ``{"error": "robot not found"}``

POST /robots/{id}/move
    Set wheel speeds directly (manual drive).  Controllable robots only.
    Request body: ``{"speed_left": <float>, "speed_right": <float>}``
    Speeds are clamped to ±MAX_SPEED (10.4 rad/s).  Sets mode to ``"manual"``
    and suspends obstacle avoidance until ``POST /start`` is called.
    Response 200: ``{"ok": true, "mode": "manual"}``
    Response 403: ``{"error": "robot is not controllable"}``
    Response 404: ``{"error": "robot not found"}``

POST /robots/{id}/stop
    Halt the robot and suspend obstacle avoidance.  Controllable robots only.
    Sets both wheel speeds to 0 and mode to ``"stopped"``.
    Response 200: ``{"ok": true, "mode": "stopped"}``
    Response 403: ``{"error": "robot is not controllable"}``
    Response 404: ``{"error": "robot not found"}``

POST /robots/{id}/start
    Resume autonomous obstacle-avoidance behaviour.  Controllable robots only.
    Sets mode back to ``"auto"``; the robot controller re-enables its built-in
    avoidance logic.
    Response 200: ``{"ok": true, "mode": "auto"}``
    Response 403: ``{"error": "robot is not controllable"}``
    Response 404: ``{"error": "robot not found"}``

GET /robots/{id}/camera
    Latest camera snapshot as a base64-encoded JPEG.
    Available for all robots in CAMERA_ROBOTS (TIAGO_1, TIAGO_2, CREATE,
    BLIMP, RANGEROVER).  Python robots use POSIX shared memory; the C
    Range Rover controller falls back to ``/tmp/webots_{id}_camera.jpg``.
    Response 200: ``{"robot_id": str, "format": "jpeg", "data": "<base64>"}``
    Response 503: ``{"error": "no image available yet"}``
    Response 404: ``{"error": "no camera for this robot"}``

GET /robots/{id}/camera/stream
    MJPEG live stream of the robot's on-board camera.
    Content-Type: ``multipart/x-mixed-replace; boundary=frame``
    Response 404: ``{"error": "no camera for this robot"}``

GET /god/camera
    Top-down god-view camera snapshot.
    Response 200: ``{"source": "god_camera", "format": "jpeg", "data": "<base64>"}``
    Response 503: ``{"error": "no image available yet"}``

GET /god/camera/stream
    MJPEG live stream of the top-down god camera.
    Content-Type: ``multipart/x-mixed-replace; boundary=frame``

POST /simulation/pause
    Pause the simulation (applied on the next supervisor step).
    Response 200: ``{"ok": true}``

POST /simulation/resume
    Resume a paused simulation (applied on the next supervisor step).
    Response 200: ``{"ok": true}``

GET /simulation/time
    Current simulated time.
    Response 200: ``{"time": <float>}``  (seconds since simulation start)
"""

import base64
import json
import logging
import os
import struct
import tempfile
import threading

from controller import Supervisor

try:
    from flask import Flask, Response, jsonify, request
except ImportError:
    import subprocess
    import sys

    subprocess.check_call([sys.executable, "-m", "pip", "install", "flask", "-q"])
    from flask import Flask, Response, jsonify, request

# PIL is only needed for the supervisor-owned god camera; import lazily so the
# controllers that publish their own frames are not forced to depend on it.
try:
    from PIL import Image
except ImportError:  # pragma: no cover - resolved via requirements.txt in Webots
    Image = None

# ── Constants ────────────────────────────────────────────────────────────────
API_PORT = 5000
MAX_SPEED = 10.4         # rad/s, conservative cap shared by the wheeled robots
JPEG_QUALITY = 75

# Controllable robots: DEF name → canonical id (kept identical for clarity).
CONTROLLABLE = ("TIAGO_1", "TIAGO_2", "CREATE")

# All robots that expose an on-board camera, with the IPC transport used by
# their controller: "shm" (shared memory, Python controllers) or "file"
# (JPEG file written by the C Range Rover controller).
CAMERA_ROBOTS = {
    "TIAGO_1": "shm",
    "TIAGO_2": "shm",
    "CREATE": "shm",
    "BLIMP": "shm",
    "RANGEROVER": "file",
}

# Every robot whose pose the supervisor tracks (DEF name in the .wbt file).
ROBOT_DEFS = ("TIAGO_1", "TIAGO_2", "CREATE", "BLIMP", "RANGEROVER")

# ── Shared-memory layout for robot cameras ────────────────────────────────────
# Header: [seq: uint32 LE][length: uint32 LE] followed by the JPEG payload.
_SHM_HEADER = 8
_SHM_SIZE = 512 * 1024 + _SHM_HEADER   # 512 KB payload + header

_tmp = tempfile.gettempdir()

# ── Shared state (Flask thread + main loop) ──────────────────────────────────
_lock = threading.Lock()

_robots = {
    rid: {
        "translation": [0.0, 0.0, 0.0],
        "rotation": [0.0, 0.0, 1.0, 0.0],
        "mode": "auto",          # auto | manual | stopped
        "speed_left": 0.0,
        "speed_right": 0.0,
        "controllable": rid in CONTROLLABLE,
    }
    for rid in ROBOT_DEFS
}

_sim_time = 0.0
_paused = False
_pending_pause = False
_pending_resume = False

# ── In-memory camera frame buffers ───────────────────────────────────────────
_god_cam_cond = threading.Condition()
_god_frame = b""
_god_frame_seq = 0

_robot_cam_cond = {rid: threading.Condition() for rid in CAMERA_ROBOTS}
_robot_frames = {rid: b"" for rid in CAMERA_ROBOTS}
_robot_frame_seq = {rid: 0 for rid in CAMERA_ROBOTS}

# Per-robot IPC bookkeeping (populated in main()).
_robot_shm = {}                                       # rid → SharedMemory | None
_robot_shm_last_seq = {rid: 0 for rid in CAMERA_ROBOTS}
_robot_file_mtime = {rid: 0.0 for rid in CAMERA_ROBOTS}


# ── JPEG encoding helper (god camera) ────────────────────────────────────────

def _encode_jpeg(camera, quality=JPEG_QUALITY):
    """Encode a Webots camera frame to JPEG bytes entirely in memory."""
    import io

    raw = camera.getImage()
    img = Image.frombytes(
        "RGBA", (camera.getWidth(), camera.getHeight()), raw, "raw", "BGRA"
    )
    buf = io.BytesIO()
    img.convert("RGB").save(buf, format="JPEG", quality=quality, optimize=False)
    return buf.getvalue()


# ── MJPEG streaming generators ───────────────────────────────────────────────

def _camera_stream(cond, get_frame, get_seq):
    """Yield MJPEG chunks, blocking on *cond* until a new frame is published."""
    last_seq = -1
    while True:
        with cond:
            cond.wait_for(lambda: get_seq() != last_seq, timeout=1.0)
            seq = get_seq()
            frame = get_frame()
        if seq != last_seq:
            last_seq = seq
            if frame:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )


# ── Flask application ─────────────────────────────────────────────────────────

app = Flask(__name__)
app.url_map.merge_slashes = True

logging.getLogger("werkzeug").setLevel(logging.ERROR)  # silence Flask banner

# Numeric aliases: "1"/"2"/"3" → the controllable robots in declaration order.
_NUMERIC_ID_MAP = {str(i + 1): rid for i, rid in enumerate(CONTROLLABLE)}


def _resolve_rid(rid):
    """Return the canonical robot id for *rid*, accepting numeric aliases."""
    if rid in _robots:
        return rid
    return _NUMERIC_ID_MAP.get(rid, rid)


def _read_sensors(rid):
    """Read the latest sensor JSON document published by a robot controller."""
    sfp = os.path.join(_tmp, f"webots_{rid}_state.json")
    try:
        if os.path.exists(sfp):
            with open(sfp) as fh:
                return json.load(fh)
    except (OSError, json.JSONDecodeError):
        pass
    return {}


def _robot_public_state(rid):
    """Return a JSON-serialisable snapshot of a robot's state."""
    with _lock:
        r = dict(_robots[rid])
    pos = r["translation"]
    rot = r["rotation"]
    return {
        "id": rid,
        "controllable": r["controllable"],
        "has_camera": rid in CAMERA_ROBOTS,
        "position": {"x": pos[0], "y": pos[1], "z": pos[2]},
        "rotation": {"ax": rot[0], "ay": rot[1], "az": rot[2], "angle": rot[3]},
        "mode": r["mode"],
        "speed_left": r["speed_left"],
        "speed_right": r["speed_right"],
        "sensors": _read_sensors(rid),
    }


@app.get("/robots")
def get_robots():
    """Return the state of all robots in the scene.

    GET /robots

    Response 200 – JSON array of robot state objects, one per robot in
    ``ROBOT_DEFS`` (TIAGO_1, TIAGO_2, CREATE, BLIMP, RANGEROVER).
    Each element has the same structure as ``GET /robots/{id}``.
    """
    return jsonify([_robot_public_state(rid) for rid in ROBOT_DEFS])


@app.get("/robots/<rid>")
def get_robot(rid):
    """Return the full state of a single robot.

    GET /robots/{id}

    Path parameter:
        id – Robot identifier: "TIAGO_1", "TIAGO_2", "CREATE", "BLIMP",
             "RANGEROVER", or numeric aliases "1" / "2" / "3" for the
             controllable robots.

    Response 200::

        {
          "id":           "TIAGO_1",
          "controllable": true,
          "has_camera":   true,
          "position":     {"x": 0.0, "y": 0.0, "z": 0.0},
          "rotation":     {"ax": 0.0, "ay": 0.0, "az": 1.0, "angle": 0.0},
          "mode":         "auto" | "manual" | "stopped",
          "speed_left":   0.0,
          "speed_right":  0.0,
          "sensors":      { <sensor_key>: <value>, ... }
        }

    Response 404: ``{"error": "robot not found"}``
    """
    rid = _resolve_rid(rid)
    if rid not in _robots:
        return jsonify({"error": "robot not found"}), 404
    return jsonify(_robot_public_state(rid))


@app.get("/robots/<rid>/sensors")
def robot_sensors(rid):
    """Return raw sensor data published by the robot controller.

    GET /robots/{id}/sensors

    The sensor document is written by the robot controller to
    ``/tmp/webots_{id}_state.json`` every step and typically contains
    distance-sensor readings, wheel encoder values and IMU data.

    Response 200::

        {"robot_id": "TIAGO_1", "sensors": { <sensor_key>: <value>, ... }}

    Response 503: ``{"error": "no sensor data available yet"}`` – the
    controller has not yet written its first snapshot.
    Response 404: ``{"error": "robot not found"}``
    """
    rid = _resolve_rid(rid)
    if rid not in _robots:
        return jsonify({"error": "robot not found"}), 404
    sensors = _read_sensors(rid)
    if not sensors:
        return jsonify({"error": "no sensor data available yet"}), 503
    return jsonify({"robot_id": rid, "sensors": sensors})


@app.post("/robots/<rid>/move")
def move_robot(rid):
    """Set the wheel speeds for direct manual driving.

    POST /robots/{id}/move

    Only controllable robots (TIAGO_1, TIAGO_2, CREATE) accept this command.

    Request body (JSON)::

        {"speed_left": <float>, "speed_right": <float>}

    Both values are clamped to ±MAX_SPEED (10.4 rad/s).  Sets mode to
    ``"manual"``; obstacle avoidance is suspended until ``POST /start`` is called.

    Response 200: ``{"ok": true, "mode": "manual"}``
    Response 403: ``{"error": "robot is not controllable"}``
    Response 404: ``{"error": "robot not found"}``
    """
    rid = _resolve_rid(rid)
    if rid not in _robots:
        return jsonify({"error": "robot not found"}), 404
    if not _robots[rid]["controllable"]:
        return jsonify({"error": "robot is not controllable"}), 403
    data = request.get_json(force=True, silent=True) or {}
    sl = float(data.get("speed_left", 0.0))
    sr = float(data.get("speed_right", 0.0))
    with _lock:
        _robots[rid]["speed_left"] = max(-MAX_SPEED, min(MAX_SPEED, sl))
        _robots[rid]["speed_right"] = max(-MAX_SPEED, min(MAX_SPEED, sr))
        _robots[rid]["mode"] = "manual"
    return jsonify({"ok": True, "mode": "manual"})


@app.post("/robots/<rid>/stop")
def stop_robot(rid):
    """Halt the robot and suspend its obstacle-avoidance behaviour.

    POST /robots/{id}/stop

    Only controllable robots (TIAGO_1, TIAGO_2, CREATE) accept this command.
    Zeroes both wheel speeds and sets mode to ``"stopped"``.  Obstacle
    avoidance remains suspended until ``POST /start`` is called.

    Response 200: ``{"ok": true, "mode": "stopped"}``
    Response 403: ``{"error": "robot is not controllable"}``
    Response 404: ``{"error": "robot not found"}``
    """
    rid = _resolve_rid(rid)
    if rid not in _robots:
        return jsonify({"error": "robot not found"}), 404
    if not _robots[rid]["controllable"]:
        return jsonify({"error": "robot is not controllable"}), 403
    with _lock:
        _robots[rid]["speed_left"] = 0.0
        _robots[rid]["speed_right"] = 0.0
        _robots[rid]["mode"] = "stopped"
    return jsonify({"ok": True, "mode": "stopped"})


@app.post("/robots/<rid>/start")
def start_robot(rid):
    """Resume the robot's autonomous obstacle-avoidance behaviour.

    POST /robots/{id}/start

    Only controllable robots (TIAGO_1, TIAGO_2, CREATE) accept this command.
    Sets mode to ``"auto"``; the robot controller re-enables its built-in
    avoidance logic on the next step.

    Response 200: ``{"ok": true, "mode": "auto"}``
    Response 403: ``{"error": "robot is not controllable"}``
    Response 404: ``{"error": "robot not found"}``
    """
    rid = _resolve_rid(rid)
    if rid not in _robots:
        return jsonify({"error": "robot not found"}), 404
    if not _robots[rid]["controllable"]:
        return jsonify({"error": "robot is not controllable"}), 403
    with _lock:
        _robots[rid]["mode"] = "auto"
    return jsonify({"ok": True, "mode": "auto"})


@app.get("/robots/<rid>/camera")
def robot_camera(rid):
    """Return the latest camera frame as a base64-encoded JPEG snapshot.

    GET /robots/{id}/camera

    Available for all robots in CAMERA_ROBOTS (TIAGO_1, TIAGO_2, CREATE,
    BLIMP, RANGEROVER).  Python-based robots use POSIX shared memory
    (``webots_{id}_camera_shm``); the C Range Rover controller writes a JPEG
    file at ``/tmp/webots_{id}_camera.jpg``.

    Response 200::

        {"robot_id": "TIAGO_1", "format": "jpeg", "data": "<base64-string>"}

    Response 503: ``{"error": "no image available yet"}`` – no frame published yet.
    Response 404: ``{"error": "no camera for this robot"}``
    """
    rid = _resolve_rid(rid)
    if rid not in CAMERA_ROBOTS:
        return jsonify({"error": "no camera for this robot"}), 404
    with _robot_cam_cond[rid]:
        frame = _robot_frames[rid]
    if not frame:
        return jsonify({"error": "no image available yet"}), 503
    b64 = base64.b64encode(frame).decode()
    return jsonify({"robot_id": rid, "format": "jpeg", "data": b64})


@app.get("/robots/<rid>/camera/stream")
def robot_camera_mjpeg(rid):
    """Stream the robot camera as a live MJPEG feed.

    GET /robots/{id}/camera/stream

    Blocks efficiently on a per-robot ``threading.Condition``; a new MJPEG
    part is pushed to the client only when the supervisor main loop publishes
    a new frame from shared memory (no polling overhead).

    Content-Type: ``multipart/x-mixed-replace; boundary=frame``
    Response 404: ``{"error": "no camera for this robot"}``
    """
    rid = _resolve_rid(rid)
    if rid not in CAMERA_ROBOTS:
        return jsonify({"error": "no camera for this robot"}), 404
    cond = _robot_cam_cond[rid]
    return Response(
        _camera_stream(
            cond,
            lambda r=rid: _robot_frames[r],
            lambda r=rid: _robot_frame_seq[r],
        ),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/god/camera")
def god_camera():
    """Return the latest top-down god-view camera snapshot.

    GET /god/camera

    The supervisor encodes a JPEG from the ``god_camera`` Webots device each
    simulation step.

    Response 200::

        {"source": "god_camera", "format": "jpeg", "data": "<base64-string>"}

    Response 503: ``{"error": "no image available yet"}``
    """
    with _god_cam_cond:
        frame = _god_frame
    if not frame:
        return jsonify({"error": "no image available yet"}), 503
    b64 = base64.b64encode(frame).decode()
    return jsonify({"source": "god_camera", "format": "jpeg", "data": b64})


@app.get("/god/camera/stream")
def god_camera_mjpeg():
    """Stream the top-down god-view camera as a live MJPEG feed.

    GET /god/camera/stream

    Content-Type: ``multipart/x-mixed-replace; boundary=frame``
    """
    return Response(
        _camera_stream(_god_cam_cond, lambda: _god_frame, lambda: _god_frame_seq),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.post("/simulation/pause")
def sim_pause():
    """Pause the Webots simulation.

    POST /simulation/pause

    The pause is applied on the next supervisor step (not instantaneously).
    Has no effect if the simulation is already paused.

    Response 200: ``{"ok": true}``
    """
    global _pending_pause
    with _lock:
        _pending_pause = True
    return jsonify({"ok": True})


@app.post("/simulation/resume")
def sim_resume():
    """Resume the Webots simulation after a pause.

    POST /simulation/resume

    The resume is applied on the next supervisor step.
    Has no effect if the simulation is already running.

    Response 200: ``{"ok": true}``
    """
    global _pending_resume
    with _lock:
        _pending_resume = True
    return jsonify({"ok": True})


@app.get("/simulation/time")
def sim_time():
    """Return the current simulated time.

    GET /simulation/time

    Response 200::

        {"time": <float>}   # seconds since simulation start
    """
    with _lock:
        t = _sim_time
    return jsonify({"time": t})


def _run_flask():
    app.run(host="0.0.0.0", port=API_PORT, debug=False, use_reloader=False)


# ── Shared-memory setup ───────────────────────────────────────────────────────

def _setup_camera_shm():
    """Create one shared-memory block per shm-based camera robot."""
    from multiprocessing.shared_memory import SharedMemory

    for rid, ipc in CAMERA_ROBOTS.items():
        if ipc != "shm":
            _robot_shm[rid] = None
            continue
        shm_name = f"webots_{rid}_camera_shm"
        shm = None
        try:
            shm = SharedMemory(name=shm_name, create=True, size=_SHM_SIZE)
            shm.buf[:_SHM_HEADER] = b"\x00" * _SHM_HEADER
        except FileExistsError:
            try:
                stale = SharedMemory(name=shm_name, create=False)
                stale.unlink()
                stale.close()
                shm = SharedMemory(name=shm_name, create=True, size=_SHM_SIZE)
                shm.buf[:_SHM_HEADER] = b"\x00" * _SHM_HEADER
            except Exception as exc:  # pragma: no cover
                print(f"[api_supervisor] shm recreate error for {rid}: {exc}")
        except Exception as exc:  # pragma: no cover
            print(f"[api_supervisor] shm create error for {rid}: {exc}")
        _robot_shm[rid] = shm


def _pull_camera_frames():
    """Pull new camera frames from shared memory / files into memory buffers."""
    for rid, ipc in CAMERA_ROBOTS.items():
        if ipc == "shm":
            shm = _robot_shm.get(rid)
            if shm is None:
                continue
            try:
                seq = struct.unpack_from("<I", shm.buf, 0)[0]
                if seq != _robot_shm_last_seq[rid]:
                    length = struct.unpack_from("<I", shm.buf, 4)[0]
                    if 0 < length <= _SHM_SIZE - _SHM_HEADER:
                        frame = bytes(shm.buf[_SHM_HEADER:_SHM_HEADER + length])
                        _robot_shm_last_seq[rid] = seq
                        with _robot_cam_cond[rid]:
                            _robot_frames[rid] = frame
                            _robot_frame_seq[rid] += 1
                            _robot_cam_cond[rid].notify_all()
            except Exception as exc:  # pragma: no cover
                print(f"[api_supervisor] shm read error for {rid}: {exc}")
        else:  # file-based transport (Range Rover)
            path = os.path.join(_tmp, f"webots_{rid}_camera.jpg")
            try:
                mtime = os.path.getmtime(path)
                if mtime != _robot_file_mtime[rid]:
                    _robot_file_mtime[rid] = mtime
                    with open(path, "rb") as fh:
                        frame = fh.read()
                    if frame:
                        with _robot_cam_cond[rid]:
                            _robot_frames[rid] = frame
                            _robot_frame_seq[rid] += 1
                            _robot_cam_cond[rid].notify_all()
            except OSError:
                pass


# ── Main supervisor loop ──────────────────────────────────────────────────────

def main():
    global _sim_time, _paused, _pending_pause, _pending_resume
    global _god_frame, _god_frame_seq

    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    nodes = {rid: supervisor.getFromDef(rid) for rid in ROBOT_DEFS}
    for rid, node in nodes.items():
        if node is None:
            print(f"[api_supervisor] WARNING: DEF '{rid}' not found")

    god_cam = supervisor.getDevice("god_camera")
    if god_cam:
        god_cam.enable(timestep)
    else:
        print("[api_supervisor] WARNING: god_camera device not found")

    _setup_camera_shm()

    flask_thread = threading.Thread(target=_run_flask, daemon=True)
    flask_thread.start()
    print(f"[api_supervisor] REST API listening on http://0.0.0.0:{API_PORT}")

    try:
        while supervisor.step(timestep) != -1:
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

            with _lock:
                _sim_time = supervisor.getTime()

            # Update poses and forward commands to controllable robots.
            for rid, node in nodes.items():
                if node is None:
                    continue
                trans = node.getField("translation").getSFVec3f()
                rot = node.getField("rotation").getSFRotation()
                with _lock:
                    _robots[rid]["translation"] = list(trans)
                    _robots[rid]["rotation"] = list(rot)
                    if not _robots[rid]["controllable"]:
                        continue
                    cmd = json.dumps({
                        "mode": _robots[rid]["mode"],
                        "speed_left": _robots[rid]["speed_left"],
                        "speed_right": _robots[rid]["speed_right"],
                    })
                node.getField("customData").setSFString(cmd)

            _pull_camera_frames()

            if god_cam and Image is not None:
                try:
                    frame = _encode_jpeg(god_cam)
                    with _god_cam_cond:
                        _god_frame = frame
                        _god_frame_seq += 1
                        _god_cam_cond.notify_all()
                except Exception as exc:  # pragma: no cover
                    print(f"[api_supervisor] god_camera encoding error: {exc}")
    finally:
        for shm in _robot_shm.values():
            if shm is not None:
                try:
                    shm.unlink()
                    shm.close()
                except Exception:
                    pass


if __name__ == "__main__":
    main()

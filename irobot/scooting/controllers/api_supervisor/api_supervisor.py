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

Every robot owns an on-board camera.  Because a Supervisor can only read its
own devices, each robot controller publishes its camera frame through a POSIX
shared-memory block (fast, in-memory path).  The supervisor consumes those
frames and re-serves them as base64 snapshots and MJPEG live streams.  A
top-down ``god_camera`` owned by the supervisor itself gives a full map
overview.

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
GET  /robots                         → state of all robots
GET  /robots/{id}                    → position, status, sensors
GET  /robots/{id}/sensors            → robot internal sensor data only
POST /robots/{id}/move               → { "speed_left": f, "speed_right": f }
POST /robots/{id}/stop               → halt + pause obstacle avoidance
POST /robots/{id}/start              → resume obstacle avoidance (mode=auto)
GET  /robots/{id}/camera             → JPEG snapshot as base64 JSON
GET  /robots/{id}/camera/stream      → MJPEG live stream
GET  /god/camera                     → top-down god-view snapshot as base64 JSON
GET  /god/camera/stream              → MJPEG live stream
POST /simulation/pause               → pause the simulation
POST /simulation/resume              → resume the simulation
GET  /simulation/time                → current simulated time
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

# All robots that expose an on-board camera (shared-memory IPC transport).
CAMERA_ROBOTS = {
    "TIAGO_1": "shm",
    "TIAGO_2": "shm",
    "CREATE": "shm",
    "BLIMP": "shm",
}

# Every robot whose pose the supervisor tracks (DEF name in the .wbt file).
ROBOT_DEFS = ("TIAGO_1", "TIAGO_2", "CREATE", "BLIMP")

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

# DEF names that were successfully resolved at start-up (populated in main()).
# Used by Flask endpoints to return 503 immediately instead of hanging when a
# robot's node is absent from the scene.
_nodes_found: set = set()


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
    """Yield MJPEG chunks, blocking on *cond* until a new frame is published.

    Each yield returns one complete MJPEG part (boundary + headers + JPEG body).
    Handles client disconnects cleanly via ``GeneratorExit``.
    """
    last_seq = -1
    while True:
        try:
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
        except GeneratorExit:
            return


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
    return jsonify([_robot_public_state(rid) for rid in ROBOT_DEFS])


@app.get("/robots/<rid>")
def get_robot(rid):
    rid = _resolve_rid(rid)
    if rid not in _robots:
        return jsonify({"error": "robot not found"}), 404
    return jsonify(_robot_public_state(rid))


@app.get("/robots/<rid>/sensors")
def robot_sensors(rid):
    rid = _resolve_rid(rid)
    if rid not in _robots:
        return jsonify({"error": "robot not found"}), 404
    sensors = _read_sensors(rid)
    if not sensors:
        return jsonify({"error": "no sensor data available yet"}), 503
    return jsonify({"robot_id": rid, "sensors": sensors})


@app.post("/robots/<rid>/move")
def move_robot(rid):
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
    """Halt the robot and suspend its obstacle-avoidance behaviour."""
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
    """Resume the robot's autonomous obstacle-avoidance behaviour."""
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
    rid = _resolve_rid(rid)
    if rid not in CAMERA_ROBOTS:
        return jsonify({"error": "no camera for this robot"}), 404
    if rid not in _nodes_found:
        return jsonify({"error": "robot not active in simulation"}), 503
    with _robot_cam_cond[rid]:
        frame = _robot_frames[rid]
    if not frame:
        return jsonify({"error": "no image available yet"}), 503
    b64 = base64.b64encode(frame).decode()
    return jsonify({"robot_id": rid, "format": "jpeg", "data": b64})


@app.get("/robots/<rid>/camera/stream")
def robot_camera_mjpeg(rid):
    rid = _resolve_rid(rid)
    if rid not in CAMERA_ROBOTS:
        return jsonify({"error": "no camera for this robot"}), 404
    if rid not in _nodes_found:
        return jsonify({"error": "robot not active in simulation"}), 503
    cond = _robot_cam_cond[rid]
    return Response(
        _camera_stream(
            cond,
            lambda r=rid: _robot_frames[r],
            lambda r=rid: _robot_frame_seq[r],
        ),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


@app.get("/god/camera")
def god_camera():
    if Image is None:
        return jsonify({"error": "Pillow not installed; god camera unavailable"}), 503
    with _god_cam_cond:
        frame = _god_frame
    if not frame:
        return jsonify({"error": "no image available yet"}), 503
    b64 = base64.b64encode(frame).decode()
    return jsonify({"source": "god_camera", "format": "jpeg", "data": b64})


@app.get("/god/camera/stream")
def god_camera_mjpeg():
    if Image is None:
        return jsonify({"error": "Pillow not installed; god camera unavailable"}), 503
    return Response(
        _camera_stream(_god_cam_cond, lambda: _god_frame, lambda: _god_frame_seq),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


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
    app.run(host="0.0.0.0", port=API_PORT, debug=False, use_reloader=False, threaded=True)


# ── Shared-memory setup ───────────────────────────────────────────────────────

def _setup_camera_shm():
    """Create one shared-memory block per camera robot."""
    from multiprocessing.shared_memory import SharedMemory

    for rid in CAMERA_ROBOTS:
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
    """Pull new camera frames from shared memory into the in-memory buffers."""
    for rid in CAMERA_ROBOTS:
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

    # Record which nodes were actually resolved so Flask endpoints can return
    # 503 immediately for absent robots rather than hanging indefinitely.
    _nodes_found.update(rid for rid, node in nodes.items() if node is not None)

    # Create shared-memory blocks as early as possible so robot controllers
    # can attach to them during their own start-up (before the first step).
    _setup_camera_shm()

    god_cam = supervisor.getDevice("god_camera")
    if god_cam:
        god_cam.enable(timestep)
    else:
        print("[api_supervisor] WARNING: god_camera device not found")

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

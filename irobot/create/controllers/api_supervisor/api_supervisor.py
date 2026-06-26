"""
API Supervisor for iRobot Create simulation.

Exposes a REST API on http://localhost:5000 to control and monitor
the robots in the Webots simulation.

Endpoints
─────────
GET  /robots                         → state of all robots
GET  /robots/{id}                    → position, sensors, status
GET  /robots/{id}/sensors            → robot internal sensor data only
POST /robots/{id}/move               → { "speed_left": f, "speed_right": f }
POST /robots/{id}/goto               → { "x": f, "z": f }  (autonomous navigation)
POST /robots/{id}/stop               → immediate stop
GET  /robots/{id}/camera             → JPEG image as base64 JSON (snapshot)
GET  /robots/{id}/camera/stream      → MJPEG live stream
GET  /god/camera                     → top-down god-view camera image as base64 JSON (snapshot)
GET  /god/camera/stream              → MJPEG live stream
GET  /robots/ceiling/camera          → bedroom ceiling camera image as base64 JSON (snapshot)
GET  /robots/ceiling/camera/stream   → MJPEG live stream
POST /simulation/pause               → pause the simulation
POST /simulation/resume              → resume the simulation
GET  /simulation/time                → current simulated time

Coordinate note: the world uses X/Y as the horizontal plane (Z is up).
The API "z" parameter in /goto maps to the world Y axis, matching the
Webots VRML convention used in the original scene description.
"""

import base64
import io
import json
import math
import os
import queue
import struct
import tempfile
import threading
import time
from copy import deepcopy
from multiprocessing.shared_memory import SharedMemory

from controller import Supervisor

try:
    from flask import Flask, Response, jsonify, request
except ImportError:
    import subprocess, sys
    subprocess.check_call([sys.executable, "-m", "pip", "install", "flask", "-q"])
    from flask import Flask, Response, jsonify, request

from PIL import Image  # installed via requirements.txt

# ── Constants ────────────────────────────────────────────────────────────────
API_PORT = 5000
MAX_SPEED = 8.0          # conservative max speed for goto navigation (m/s * gearing)
GOTO_ARRIVAL_DIST = 0.15  # metres – consider target reached when within this distance
JPEG_QUALITY = 75        # balance of speed and visual quality for streaming

ROBOT_DEFS = {
    "ROBOT_1": "IROBOT_CREATE",   # DEF name in the .wbt file
    "ROBOT_2": "ROBOT_2",
}

# Fluorescent body colors for each robot (linear RGB 0–1)
ROBOT_COLORS = {
    "ROBOT_1": [0.0, 1.0, 0.0],   # fluorescent green
    "ROBOT_2": [1.0, 0.45, 0.0],  # fluorescent orange
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

# ── In-memory camera frame buffers (JPEG bytes) ──────────────────────────────
# Each camera has its own Condition so streaming clients wake immediately
# on a new frame without polling, and reads are always consistent.
_god_cam_cond = threading.Condition()
_god_frame: bytes = b""
_god_frame_seq: int = 0

_ceiling_cam_cond = threading.Condition()
_ceiling_frame: bytes = b""
_ceiling_frame_seq: int = 0

_tmp = tempfile.gettempdir()

# ── Shared-memory layout for robot cameras ────────────────────────────────────
# Header: [seq: uint32 LE][length: uint32 LE] + JPEG payload
# The robot controller (separate process) writes; the supervisor reads.
_SHM_HEADER = 8           # bytes reserved for the two uint32 fields
_SHM_SIZE = 512 * 1024 + _SHM_HEADER   # 512 KB payload + header (ample for any JPEG frame)

# Per-robot shm handles (created in main(), populated at runtime)
_robot_shm: dict = {}          # rid → SharedMemory | None

# Per-robot in-memory frame state (supervisor main loop → Flask thread)
_robot_cam_cond: dict = {rid: threading.Condition() for rid in ROBOT_DEFS}
_robot_frames: dict = {rid: b"" for rid in ROBOT_DEFS}
_robot_frame_seq: dict = {rid: 0 for rid in ROBOT_DEFS}
_robot_shm_last_seq: dict = {rid: 0 for rid in ROBOT_DEFS}    # last shm seq consumed
_robot_file_mtime: dict = {rid: 0.0 for rid in ROBOT_DEFS}   # for file-fallback path


# ── JPEG encoding helper ──────────────────────────────────────────────────────

def _encode_jpeg(camera, quality: int = JPEG_QUALITY) -> bytes:
    """Encode a Webots camera frame to JPEG bytes entirely in memory.

    Uses PIL to convert the raw BGRA pixel data returned by ``getImage()``
    to a JPEG without any disk I/O, which is significantly faster than
    ``Camera.saveImage()``.
    """
    raw = camera.getImage()   # BGRA bytes, width × height × 4
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


# ── Background JPEG encoder queues for supervisor cameras ────────────────────
# PIL encoding is CPU-intensive. Running it inside the Webots step loop makes
# each simulation step take a variable amount of real time, so motor commands
# arrive at irregular intervals and the robot appears to move jerkily.
# The main loop only calls camera.getImage() (fast, returns a bytes snapshot)
# and drops the raw data into a single-slot queue; background threads do the
# slow PIL work without ever touching the simulation step loop.

_god_cam_raw_queue: "queue.Queue" = None      # type: ignore  (created in main)
_ceiling_cam_raw_queue: "queue.Queue" = None  # type: ignore  (created in main)


def _god_cam_encoder_worker() -> None:
    """Background thread: encode god-view camera frames to JPEG."""
    global _god_frame, _god_frame_seq
    while True:
        try:
            item = _god_cam_raw_queue.get(timeout=1.0)
        except queue.Empty:
            continue
        if item is None:
            break
        raw, w, h = item
        try:
            img = Image.frombytes("RGBA", (w, h), raw, "raw", "BGRA")
            buf = io.BytesIO()
            img.convert("RGB").save(buf, format="JPEG", quality=JPEG_QUALITY, optimize=False)
            frame = buf.getvalue()
            with _god_cam_cond:
                _god_frame = frame
                _god_frame_seq += 1
                _god_cam_cond.notify_all()
        except Exception as e:
            print(f"[api_supervisor] god_camera encoding error: {e}")


def _ceiling_cam_encoder_worker() -> None:
    """Background thread: encode ceiling camera frames to JPEG."""
    global _ceiling_frame, _ceiling_frame_seq
    while True:
        try:
            item = _ceiling_cam_raw_queue.get(timeout=1.0)
        except queue.Empty:
            continue
        if item is None:
            break
        raw, w, h = item
        try:
            img = Image.frombytes("RGBA", (w, h), raw, "raw", "BGRA")
            buf = io.BytesIO()
            img.convert("RGB").save(buf, format="JPEG", quality=JPEG_QUALITY, optimize=False)
            frame = buf.getvalue()
            with _ceiling_cam_cond:
                _ceiling_frame = frame
                _ceiling_frame_seq += 1
                _ceiling_cam_cond.notify_all()
        except Exception as e:
            print(f"[api_supervisor] ceiling_camera encoding error: {e}")


# ── MJPEG streaming generators ────────────────────────────────────────────────

def _supervisor_camera_stream(cond, get_frame, get_seq):
    """Generator for supervisor-side cameras (god, ceiling).

    Blocks efficiently on ``cond`` until the main loop publishes a new frame,
    then immediately yields the MJPEG chunk.  Multiple simultaneous clients
    are each tracked by their own ``last_seq`` so no frame is ever missed.
    """
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
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + frame
                    + b"\r\n"
                )


def _robot_camera_stream(rid: str):
    """Generator for robot cameras.

    Waits efficiently on a per-robot ``Condition`` that is notified by the
    supervisor main loop whenever a new frame arrives (from shared memory or,
    as a fallback, from the /tmp JPEG file).  This is identical in structure to
    ``_supervisor_camera_stream`` – no polling, no sleep.
    """
    cond = _robot_cam_cond[rid]
    last_seq = -1
    while True:
        with cond:
            cond.wait_for(lambda: _robot_frame_seq[rid] != last_seq, timeout=1.0)
            seq = _robot_frame_seq[rid]
            frame = _robot_frames[rid]
        if seq != last_seq:
            last_seq = seq
            if frame:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + frame
                    + b"\r\n"
                )


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


# ── Robot appearance helpers ─────────────────────────────────────────────────

def _apply_robot_color(node, color):
    """Set the fluorescent body color of a Create robot via the Supervisor API.

    Navigates the robot's child list to find the first Shape whose appearance
    is a PBRAppearance, then overwrites *baseColor* and adds a matching
    *emissiveColor* for a glowing fluorescent effect.  The texture map is
    removed so the flat saturated color is fully visible.
    """
    children_field = node.getField("children")
    if children_field is None:
        return
    count = children_field.getCount()
    for i in range(count):
        child = children_field.getMFNode(i)
        if child.getTypeName() != "Shape":
            continue
        app_field = child.getField("appearance")
        if app_field is None:
            continue
        app_node = app_field.getSFNode()
        if app_node is None or app_node.getTypeName() != "PBRAppearance":
            continue
        base_color_field = app_node.getField("baseColor")
        if base_color_field:
            base_color_field.setSFColor(color)
        # Remove the default texture so the flat colour is fully visible
        base_color_map_field = app_node.getField("baseColorMap")
        if base_color_map_field:
            base_color_map_field.setSFNode(None)
        # Add a subtle self-illumination for a fluorescent glow
        emissive_field = app_node.getField("emissiveColor")
        if emissive_field:
            emissive_field.setSFColor([c * 0.35 for c in color])
        break  # Only the first Shape carries the main body appearance


# ── Flask application ────────────────────────────────────────────────────────

app = Flask(__name__)
# Merge consecutive slashes so clients that send //robots/… still match routes
app.url_map.merge_slashes = True

# silence Flask startup banner in Webots console
import logging
log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)

# Numeric alias map so clients can use "1" / "2" in place of "ROBOT_1" / "ROBOT_2".
# ROBOT_DEFS is a static constant; Python 3.7+ dicts preserve insertion order,
# so "1" always maps to the first entry and "2" to the second.
_NUMERIC_ID_MAP: dict[str, str] = {str(i + 1): rid for i, rid in enumerate(ROBOT_DEFS)}


def _resolve_rid(rid: str) -> str:
    """Return the canonical robot ID for *rid*, accepting numeric aliases."""
    return _NUMERIC_ID_MAP.get(rid, rid)


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
    rid = _resolve_rid(rid)
    if rid not in ROBOT_DEFS:
        return jsonify({"error": "robot not found"}), 404
    return jsonify(_robot_public_state(rid))


@app.post("/robots/<rid>/move")
def move_robot(rid):
    rid = _resolve_rid(rid)
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
    rid = _resolve_rid(rid)
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
    rid = _resolve_rid(rid)
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
    rid = _resolve_rid(rid)
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
    rid = _resolve_rid(rid)
    if rid not in ROBOT_DEFS:
        return jsonify({"error": "robot not found"}), 404
    # Fast path: use in-memory frame populated from shared memory
    with _robot_cam_cond[rid]:
        frame = _robot_frames[rid]
    if frame:
        b64 = base64.b64encode(frame).decode()
        return jsonify({"robot_id": rid, "format": "jpeg", "data": b64})
    # Fallback: read from /tmp file (shm not yet ready or not available)
    img_path = os.path.join(_tmp, f"webots_{rid}_camera.jpg")
    if not os.path.exists(img_path):
        return jsonify({"error": "no image available yet"}), 503
    with open(img_path, "rb") as fh:
        b64 = base64.b64encode(fh.read()).decode()
    return jsonify({"robot_id": rid, "format": "jpeg", "data": b64})


@app.get("/robots/<rid>/camera/stream")
def robot_camera_mjpeg(rid):
    """MJPEG live stream for a robot's front camera."""
    rid = _resolve_rid(rid)
    if rid not in ROBOT_DEFS:
        return jsonify({"error": "robot not found"}), 404
    return Response(
        _robot_camera_stream(rid),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/god/camera")
def god_camera():
    with _god_cam_cond:
        frame = _god_frame
    if not frame:
        return jsonify({"error": "no image available yet"}), 503
    b64 = base64.b64encode(frame).decode()
    return jsonify({"source": "god_camera", "format": "jpeg", "data": b64})


@app.get("/god/camera/stream")
def god_camera_mjpeg():
    """MJPEG live stream for the top-down god camera."""
    return Response(
        _supervisor_camera_stream(
            _god_cam_cond,
            lambda: _god_frame,
            lambda: _god_frame_seq,
        ),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/robots/ceiling/camera")
def ceiling_camera():
    with _ceiling_cam_cond:
        frame = _ceiling_frame
    if not frame:
        return jsonify({"error": "no image available yet"}), 503
    b64 = base64.b64encode(frame).decode()
    return jsonify({"source": "ceiling_camera", "format": "jpeg", "data": b64})


@app.get("/robots/ceiling/camera/stream")
def ceiling_camera_mjpeg():
    """MJPEG live stream for the bedroom ceiling camera."""
    return Response(
        _supervisor_camera_stream(
            _ceiling_cam_cond,
            lambda: _ceiling_frame,
            lambda: _ceiling_frame_seq,
        ),
        mimetype="multipart/x-mixed-replace; boundary=frame",
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
    app.run(host="0.0.0.0", port=API_PORT, debug=False, use_reloader=False)


# ── Main supervisor loop ─────────────────────────────────────────────────────

def main():
    global _sim_time, _paused, _pending_pause, _pending_resume
    global _god_frame, _god_frame_seq, _ceiling_frame, _ceiling_frame_seq

    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    # ── Get robot nodes ──────────────────────────────────────────────────────
    nodes = {rid: supervisor.getFromDef(def_name) for rid, def_name in ROBOT_DEFS.items()}
    for rid, node in nodes.items():
        if node is None:
            print(f"[api_supervisor] WARNING: DEF '{ROBOT_DEFS[rid]}' not found for {rid}")

    # ── Apply fluorescent colors ─────────────────────────────────────────────
    for rid, node in nodes.items():
        if node is None:
            continue
        color = ROBOT_COLORS.get(rid)
        if color:
            _apply_robot_color(node, color)
            print(f"[api_supervisor] Set color {color} on {rid}")

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

    # ── Create shared-memory blocks for robot cameras ────────────────────────
    # The supervisor creates the blocks; robot controllers attach to them.
    # If a stale block exists from a previous run it is unlinked and recreated.
    for rid in ROBOT_DEFS:
        shm_name = f"webots_{rid}_camera_shm"
        shm = None
        try:
            shm = SharedMemory(name=shm_name, create=True, size=_SHM_SIZE)
            shm.buf[:_SHM_HEADER] = b"\x00" * _SHM_HEADER
            print(f"[api_supervisor] shm created: {shm_name}")
        except FileExistsError:
            try:
                stale = SharedMemory(name=shm_name, create=False)
                stale.unlink()
                stale.close()
                shm = SharedMemory(name=shm_name, create=True, size=_SHM_SIZE)
                shm.buf[:_SHM_HEADER] = b"\x00" * _SHM_HEADER
                print(f"[api_supervisor] shm recreated (stale removed): {shm_name}")
            except Exception as exc:
                print(f"[api_supervisor] shm recreate error for {rid}: {exc}")
        except Exception as exc:
            print(f"[api_supervisor] shm create error for {rid}: {exc}")
        _robot_shm[rid] = shm

    # ── Start background JPEG encoder threads for supervisor cameras ─────────
    # Camera dimensions are fixed for the lifetime of the simulation; capture
    # them once here so the main loop only needs a single getImage() call.
    global _god_cam_raw_queue, _ceiling_cam_raw_queue

    god_cam_size: "tuple[int, int] | None" = None
    ceiling_cam_size: "tuple[int, int] | None" = None

    if god_cam:
        god_cam_size = (god_cam.getWidth(), god_cam.getHeight())
        _god_cam_raw_queue = queue.Queue(maxsize=1)
        threading.Thread(target=_god_cam_encoder_worker, daemon=True).start()

    if ceiling_cam:
        ceiling_cam_size = (ceiling_cam.getWidth(), ceiling_cam.getHeight())
        _ceiling_cam_raw_queue = queue.Queue(maxsize=1)
        threading.Thread(target=_ceiling_cam_encoder_worker, daemon=True).start()

    # ── Start Flask in a background thread ───────────────────────────────────
    flask_thread = threading.Thread(target=_run_flask, daemon=True)
    flask_thread.start()
    print(f"[api_supervisor] REST API listening on http://0.0.0.0:{API_PORT}")

    try:
        while supervisor.step(timestep) != -1:
            # ── Handle pause / resume requests ──────────────────────────────
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

            # ── Update simulation time ───────────────────────────────────────
            with _lock:
                _sim_time = supervisor.getTime()

            # ── Update robot state and apply commands ────────────────────────
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

                # Re-acquire the lock to read the freshest speed values.
                # A /move request that was waiting on the first lock will have
                # updated _robots[rid] by the time we reach this second lock,
                # so its speed is applied in the current step instead of the next.
                with _lock:
                    sl = _robots[rid]["speed_left"]
                    sr = _robots[rid]["speed_right"]

                # Write command to robot's customData (read by robot_controller.py)
                cmd = json.dumps({"speed_left": sl, "speed_right": sr})
                node.getField("customData").setSFString(cmd)

            # ── Pull robot camera frames from shm (or file fallback) ─────────
            for rid in ROBOT_DEFS:
                new_frame = None

                shm = _robot_shm.get(rid)
                if shm is not None:
                    try:
                        # Read the 8-byte header as a single copy to minimise torn reads.
                        header = bytes(shm.buf[0:_SHM_HEADER])
                        seq = struct.unpack_from("<I", header, 0)[0]
                        if seq != _robot_shm_last_seq[rid]:
                            length = struct.unpack_from("<I", header, 4)[0]
                            if 0 < length < _SHM_SIZE - _SHM_HEADER:
                                new_frame = bytes(shm.buf[_SHM_HEADER:_SHM_HEADER + length])
                                _robot_shm_last_seq[rid] = seq
                    except Exception as exc:
                        print(f"[api_supervisor] shm read error for {rid}: {exc}")

                # File fallback: always attempted when SHM produced no new frame this
                # step.  Covers the case where the robot controller fell back to file
                # writing because it could not attach to the shared-memory block in time.
                if new_frame is None:
                    path = os.path.join(_tmp, f"webots_{rid}_camera.jpg")
                    try:
                        mtime = os.path.getmtime(path)
                        if mtime != _robot_file_mtime[rid]:
                            _robot_file_mtime[rid] = mtime
                            with open(path, "rb") as fh:
                                data = fh.read()
                            if data:
                                new_frame = data
                    except OSError:
                        pass

                if new_frame is not None:
                    with _robot_cam_cond[rid]:
                        _robot_frames[rid] = new_frame
                        _robot_frame_seq[rid] += 1
                        _robot_cam_cond[rid].notify_all()

            # ── Capture god camera frame into memory ─────────────────────────
            # Only grab the raw bytes here (fast). The background encoder thread
            # does the CPU-intensive PIL + JPEG work outside the step loop.
            if god_cam and god_cam_size is not None:
                try:
                    raw = god_cam.getImage()
                    try:
                        _god_cam_raw_queue.put_nowait((raw,) + god_cam_size)
                    except queue.Full:
                        pass  # encoder busy, drop this frame
                except Exception as e:
                    print(f"[api_supervisor] god_camera capture error: {e}")

            # ── Capture ceiling camera frame into memory ──────────────────────
            if ceiling_cam and ceiling_cam_size is not None:
                try:
                    raw = ceiling_cam.getImage()
                    try:
                        _ceiling_cam_raw_queue.put_nowait((raw,) + ceiling_cam_size)
                    except queue.Full:
                        pass  # encoder busy, drop this frame
                except Exception as e:
                    print(f"[api_supervisor] ceiling_camera capture error: {e}")

    finally:
        # ── Release shared-memory blocks ─────────────────────────────────────
        for rid, shm in _robot_shm.items():
            if shm is not None:
                try:
                    shm.unlink()
                    shm.close()
                except Exception:
                    pass


if __name__ == "__main__":
    main()

# iRobot Create – Webots Simulation with REST API

## World layout

The world uses the official Webots **complete apartment**
(`complete_apartment.wbt`): a fully furnished flat with two bedrooms, a
living room, a kitchen, a bathroom, a restroom, stairs and a balcony. The
apartment floor is centred around **(−4.96, −6.5)** and spans roughly
X ∈ [−11, 1.5], Y ∈ [−13, 0]. The roof is omitted so the god camera can
observe the whole flat from the sky.

> **Coordinate convention** – this world uses **Z-up**: X and Y are the
> horizontal plane, Z is altitude.  The API `/goto` endpoint accepts `x` and
> `z` where the API's **z** maps to the world's **Y** axis (matching the
> naming used in the task specification).

## Robots

| ID | DEF name | Starting position | Room |
|----|----------|-------------------|------|
| `ROBOT_1` | `IROBOT_CREATE` | (−4.5, −3.0) | Living room |
| `ROBOT_2` | `ROBOT_2` | (−5.0, −5.5) | Living room |

Each robot carries a **wide-field front camera** (FOV ≈ 120 °, 320 × 240 px)
oriented to look **forward** (along the robot's +X axis).

A **god camera** (`god_camera`, FOV ≈ 1.2 rad ≈ 69 °, 512 × 512 px) is mounted
high above the apartment at (−4.96, −6.5, 25), looking straight down from the
sky for a full **2D top-down view** of the whole apartment (covers ≈ 33 m per
side, more than enough for the full flat).

A **ceiling camera** (`ceiling_camera`, FOV ≈ 90 °, 320 × 320 px) is mounted at
(−1.94, −3.3, 2.3) above the **kitchen**, looking straight down for a close-up
top-down view of the kitchen area.

## Webots camera overlays

When the simulation opens, four camera overlay windows are automatically
displayed inside the Webots 3D view, each labelled with its device name:

| Overlay label | Source | Resolution |
|---------------|--------|------------|
| `front_camera` (ROBOT_1) | ROBOT_1 front-facing camera | 320 × 240 px |
| `front_camera` (ROBOT_2) | ROBOT_2 front-facing camera | 320 × 240 px |
| `god_camera` | Top-down view of the whole apartment | 512 × 512 px |
| `ceiling_camera` | Kitchen ceiling top-down view | 320 × 320 px |

All overlays can be dragged and resized inside Webots via the **View → Rendering
devices** menu. The `.wbproj` file stores the saved positions.

## Running

1. Install Webots R2025a.
2. Install Python dependencies for the API supervisor:
   ```bash
   pip install flask
   ```
3. Open `irobot/create/worlds/create.wbt` in Webots.

The REST API starts automatically on **http://localhost:5000** when the
simulation runs.

## REST API reference

### Robots

```
GET  /robots                     List all robots (position, sensors, status)
GET  /robots/{id}                Single robot detail (includes sensor data)
GET  /robots/{id}/sensors        Robot internal sensor data only
                                 Response: {"robot_id": ..., "sensors": {
                                   "name", "time", "wheel_left", "wheel_right",
                                   "bumper_left", "bumper_right", "cliff": [...]
                                 }}
POST /robots/{id}/move           Set wheel speeds (differential drive)
                                 Body: {"speed_left": <float>, "speed_right": <float>}
                                 Range: −8.0 … 8.0  (m/s, capped by controller)
                                 Cancels any active /goto navigation.
POST /robots/{id}/goto           Autonomous navigation to a point
                                 Body: {"x": <float>, "z": <float>}
                                 (z maps to world Y axis)
POST /robots/{id}/stop           Immediate stop (sets both speeds to 0)
GET  /robots/{id}/camera         Robot's front camera as base64 JPEG
                                 Response: {"robot_id": ..., "format": "jpeg", "data": "<b64>"}
GET  /robots/{id}/camera/stream  Robot's front camera as MJPEG live stream
                                 Content-Type: multipart/x-mixed-replace; boundary=frame
GET  /god/camera                 Top-down god-view camera as base64 JPEG (512×512, whole apartment)
                                 Response: {"source": "god_camera", "format": "jpeg", "data": "<b64>"}
GET  /god/camera/stream          Top-down god-view camera as MJPEG live stream
                                 Content-Type: multipart/x-mixed-replace; boundary=frame
GET  /robots/ceiling/camera      Kitchen ceiling camera as base64 JPEG (320×320)
                                 Response: {"source": "ceiling_camera", "format": "jpeg", "data": "<b64>"}
GET  /robots/ceiling/camera/stream  Kitchen ceiling camera as MJPEG live stream
                                 Content-Type: multipart/x-mixed-replace; boundary=frame
```

### Movement model — differential drive

The iRobot Create uses **differential drive**: two independent wheels whose
speed difference determines direction.  Both speeds share the same range:
**−8.0 … 8.0** (positive = forward rotation).

| Movement | `speed_left` | `speed_right` | Notes |
|----------|-------------|--------------|-------|
| **Forward** | `+S` | `+S` | Same positive speed on both wheels |
| **Backward** | `−S` | `−S` | Same negative speed on both wheels |
| **Gentle curve left** | `+S/2` | `+S` | Right wheel faster → curves left |
| **Gentle curve right** | `+S` | `+S/2` | Left wheel faster → curves right |
| **Rotate in-place left** (CCW) | `−S` | `+S` | Equal & opposite speeds |
| **Rotate in-place right** (CW) | `+S` | `−S` | Equal & opposite speeds |
| **Stop** | `0` | `0` | Or use `POST /robots/{id}/stop` |

> Use `S = 5.0` for normal speed (≈ 60 % of maximum). The maximum value `8.0`
> is a conservative cap applied at the API level; the robot controller clamps
> to `16.0` internally. Start with lower values to avoid collisions.

### Movement examples (curl)

```bash
# ── Forward ────────────────────────────────────────────────────────────────
curl -X POST http://localhost:5000/robots/ROBOT_1/move \
     -H 'Content-Type: application/json' \
     -d '{"speed_left": 5.0, "speed_right": 5.0}'

# ── Backward ───────────────────────────────────────────────────────────────
curl -X POST http://localhost:5000/robots/ROBOT_1/move \
     -H 'Content-Type: application/json' \
     -d '{"speed_left": -5.0, "speed_right": -5.0}'

# ── Gentle curve left (arc) ────────────────────────────────────────────────
curl -X POST http://localhost:5000/robots/ROBOT_1/move \
     -H 'Content-Type: application/json' \
     -d '{"speed_left": 2.5, "speed_right": 5.0}'

# ── Gentle curve right (arc) ───────────────────────────────────────────────
curl -X POST http://localhost:5000/robots/ROBOT_1/move \
     -H 'Content-Type: application/json' \
     -d '{"speed_left": 5.0, "speed_right": 2.5}'

# ── Rotate in-place left (counter-clockwise) ───────────────────────────────
curl -X POST http://localhost:5000/robots/ROBOT_1/move \
     -H 'Content-Type: application/json' \
     -d '{"speed_left": -4.0, "speed_right": 4.0}'

# ── Rotate in-place right (clockwise) ─────────────────────────────────────
curl -X POST http://localhost:5000/robots/ROBOT_1/move \
     -H 'Content-Type: application/json' \
     -d '{"speed_left": 4.0, "speed_right": -4.0}'

# ── Stop immediately ───────────────────────────────────────────────────────
curl -X POST http://localhost:5000/robots/ROBOT_1/stop
```

### Simulation

```
POST /simulation/pause           Pause the simulation
POST /simulation/resume          Resume the simulation
GET  /simulation/time            Current simulated time (seconds)
                                 Response: {"time": <float>}
```

### Other examples (curl)

```bash
# List all robots
curl http://localhost:5000/robots

# Get full state of ROBOT_1 (includes position, rotation, speeds and sensors)
curl http://localhost:5000/robots/ROBOT_1

# Get only internal sensor data for ROBOT_1
curl http://localhost:5000/robots/ROBOT_1/sensors

# Navigate ROBOT_2 autonomously to a point (x, z where z = world Y)
curl -X POST http://localhost:5000/robots/ROBOT_2/goto \
     -H 'Content-Type: application/json' \
     -d '{"x": 3.5, "z": 2.0}'

# Get the god-view camera image and save it locally
curl http://localhost:5000/god/camera | python3 -c \
  "import sys,json,base64; d=json.load(sys.stdin); open('god_view.jpg','wb').write(base64.b64decode(d['data']))"

# Get the kitchen ceiling camera image
curl http://localhost:5000/robots/ceiling/camera | python3 -c \
  "import sys,json,base64; d=json.load(sys.stdin); open('ceiling.jpg','wb').write(base64.b64decode(d['data']))"

# Get ROBOT_1's front camera image
curl http://localhost:5000/robots/ROBOT_1/camera | python3 -c \
  "import sys,json,base64; d=json.load(sys.stdin); open('robot1_cam.jpg','wb').write(base64.b64decode(d['data']))"

# View ROBOT_1's front camera as a live MJPEG stream (open in browser or VLC)
# URL: http://localhost:5000/robots/ROBOT_1/camera/stream
# or with curl:
curl http://localhost:5000/robots/ROBOT_1/camera/stream --output - | mpv -

# Get simulated time
curl http://localhost:5000/simulation/time

# Pause / resume
curl -X POST http://localhost:5000/simulation/pause
curl -X POST http://localhost:5000/simulation/resume
```

## Controller architecture

```
api_supervisor.py  (Webots supervisor, main loop + Flask server in thread)
│
│  customData field  →  robot controllers read speed commands
│  translation/rotation fields  ←  supervisor reads positions
│
├── robot_controller.py  (ROBOT_1 instance)
│     reads customData, drives motors, saves camera + state to /tmp
└── robot_controller.py  (ROBOT_2 instance)
      reads customData, drives motors, saves camera + state to /tmp
```

Camera images are written to `/tmp/webots_<ID>_camera.jpg`,
`/tmp/webots_god_camera.jpg`, and `/tmp/webots_ceiling_camera.jpg`
by the respective controllers.

### Robot sensor data (`/robots/{id}/sensors`)

Each robot continuously saves its internal state to a JSON file in `/tmp`.
The supervisor reads this file and exposes it through the API:

| Field | Type | Description |
|-------|------|-------------|
| `name` | string | Robot identifier |
| `time` | float | Simulated time (s) |
| `wheel_left` | float | Left wheel encoder value (rad) |
| `wheel_right` | float | Right wheel encoder value (rad) |
| `bumper_left` | bool | Left bumper triggered |
| `bumper_right` | bool | Right bumper triggered |
| `cliff` | float[4] | Cliff sensor values: left, front-left, front-right, right |

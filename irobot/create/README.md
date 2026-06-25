# iRobot Create – Webots Simulation with REST API

## World layout

| Room | X range | Y range | Contents |
|------|---------|---------|---------|
| Living room | −5 → −1.65 | −5 → 5 | Table, chair, armchair, virtual wall |
| Bedroom | −1.65 → 5 | −5 → 5 | Bed, second table |

> **Coordinate convention** – this world uses **Z-up**: X and Y are the
> horizontal plane, Z is altitude.  The API `/goto` endpoint accepts `x` and
> `z` where the API's **z** maps to the world's **Y** axis (matching the
> naming used in the task specification).

## Robots

| ID | DEF name | Starting position | Room |
|----|----------|-------------------|------|
| `ROBOT_1` | `IROBOT_CREATE` | (−3.69, 0.02) | Living room |
| `ROBOT_2` | `ROBOT_2` | (2.5, −2.5) | Bedroom |

Each robot carries a **wide-field front camera** (FOV ≈ 120 °, 320 × 240 px)
oriented to look **forward** (along the robot's +X axis).

A **god camera** (`god_camera`, FOV ≈ 74 °, 512 × 512 px) is mounted at
(0, 0, 12) – centred above the entire arena – looking straight down for a
full top-down view of both rooms.

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
POST /robots/{id}/move           Set wheel speeds
                                 Body: {"speed_left": <float>, "speed_right": <float>}
                                 Range: −8.0 … 8.0
POST /robots/{id}/goto           Autonomous navigation to a point
                                 Body: {"x": <float>, "z": <float>}
                                 (z maps to world Y axis)
POST /robots/{id}/stop           Immediate stop
GET  /robots/{id}/camera         Robot's front camera as base64 JPEG
                                 Response: {"robot_id": ..., "format": "jpeg", "data": "<b64>"}
GET  /god/camera                 Top-down god-view camera as base64 JPEG (512×512, full arena)
                                 Response: {"source": "god_camera", "format": "jpeg", "data": "<b64>"}
```

### Simulation

```
POST /simulation/pause           Pause the simulation
POST /simulation/resume          Resume the simulation
GET  /simulation/time            Current simulated time (seconds)
                                 Response: {"time": <float>}
```

### Examples (curl)

```bash
# List all robots
curl http://localhost:5000/robots

# Get full state of ROBOT_1 (includes position, rotation, speeds and sensors)
curl http://localhost:5000/robots/ROBOT_1

# Get only internal sensor data for ROBOT_1
curl http://localhost:5000/robots/ROBOT_1/sensors

# Drive ROBOT_1 forward
curl -X POST http://localhost:5000/robots/ROBOT_1/move \
     -H 'Content-Type: application/json' \
     -d '{"speed_left": 5.0, "speed_right": 5.0}'

# Navigate ROBOT_2 to bedroom corner
curl -X POST http://localhost:5000/robots/ROBOT_2/goto \
     -H 'Content-Type: application/json' \
     -d '{"x": 3.5, "z": 2.0}'

# Stop ROBOT_1
curl -X POST http://localhost:5000/robots/ROBOT_1/stop

# Get the god-view camera image and decode it
curl http://localhost:5000/god/camera | python3 -c \
  "import sys,json,base64; d=json.load(sys.stdin); open('god_view.jpg','wb').write(base64.b64decode(d['data']))"

# Get a robot's front camera image
curl http://localhost:5000/robots/ROBOT_1/camera | python3 -c \
  "import sys,json,base64; d=json.load(sys.stdin); open('robot1_cam.jpg','wb').write(base64.b64decode(d['data']))"

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

Camera images are written to `/tmp/webots_<ID>_camera.jpg` and
`/tmp/webots_god_camera.jpg` by the respective controllers.

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

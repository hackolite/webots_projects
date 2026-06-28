# Scooting – Webots neighbourhood with REST API, camera streams & obstacle avoidance

This project places five robots in an open residential neighbourhood and exposes
a Flask REST API to monitor them, stream their cameras and drive the wheeled
ones. It follows the exact same architecture as the sibling
[`create`](../create/README.md) project (in-memory camera transport + a
single Webots supervisor hosting the API).

## World layout

`worlds/scooting.wbt` uses a flat 100 × 100 m `Floor` surrounded by houses and
residential towers. The roof is open so the top-down *god camera* can observe
the whole map.

> **Coordinate convention** – the world is **Z-up**: X and Y are the horizontal
> plane, Z is altitude.

### Robots

| ID / DEF / name | Proto | Controller | Controllable | Camera |
|-----------------|-------|------------|:------------:|:------:|
| `TIAGO_1` | TiagoLite | `tiago_lite` | ✅ | `Astra rgb` |
| `TIAGO_2` | TiagoLite | `tiago_lite` | ✅ | `Astra rgb` |
| `CREATE` | Create | `create_controller` | ✅ | `front_camera` |
| `BLIMP` | Blimp | `blimp_controller` | ❌ (camera + sensors only) | `camera` |
| `RANGEROVER` | RangeRoverSportSVR | `autonomous_vehicle` | ❌ (camera + sensors only) | `camera` |

For every robot the **DEF name**, the **Webots node name** and the **REST id**
are kept identical (e.g. `TIAGO_1`) so the supervisor can both locate the node
(`getFromDef`) and match the IPC files a controller publishes
(`webots_<id>_camera_shm`, `webots_<id>_state.json`).

The **Blimp** is now an actual node in the world (it was previously only
referenced by the viewpoint), placed above the map so it appears correctly, and
the **iRobot Create** has been added next to the Range Rover.

A supervisor-owned **god camera** (`god_camera`, 512 × 512) is mounted 70 m above
the centre of the map looking straight down for a full top-down overview.

## Architecture

```
api_supervisor.py  (Webots Supervisor + Flask server in a background thread)
│
│  customData field    →  controllable robots read {mode, speed_left, speed_right}
│  translation/rotation ←  supervisor reads every robot's pose
│  shared memory / file ←  every robot publishes its on-board camera (JPEG)
│  /tmp/*_state.json    ←  every robot publishes its sensor snapshot
│
├── tiago_lite.py        (TIAGO_1, TIAGO_2)  lidar obstacle avoidance
├── create_controller.py (CREATE)            bumper/cliff obstacle avoidance
├── blimp_controller.py  (BLIMP)             flight + camera/sensor publishing
└── autonomous_vehicle.c (RANGEROVER)        camera/sensor publishing (file)
```

Because a Webots `Supervisor` can only read its **own** devices, each robot
controller encodes its camera frame to JPEG and publishes it:

* **Shared memory (in-memory, fast path)** – used by the Python robots
  (`TIAGO_1`, `TIAGO_2`, `CREATE`, `BLIMP`). The supervisor creates one POSIX
  shared-memory block per robot; the controller attaches and writes frames with
  no disk I/O.
* **JPEG file (fallback)** – used by the C Range Rover controller via
  `wb_camera_save_image`. The supervisor detects new frames by file mtime.

The supervisor consumes those frames and re-serves them as base64 snapshots and
MJPEG live streams. A shared helper module, `scooting_io.py`, is copied into
each Python controller directory and implements both the camera publisher and
the atomic sensor-JSON writer.

## Obstacle avoidance + start/stop

The two TiagoLite robots and the Create run **obstacle avoidance by default**.
The supervisor writes a small JSON command into each controllable robot's
`customData` field every step:

```json
{"mode": "auto" | "manual" | "stopped", "speed_left": 0.0, "speed_right": 0.0}
```

| Mode | Behaviour |
|------|-----------|
| `auto` | The robot runs its built-in obstacle avoidance (**default on start-up**). |
| `manual` | The robot applies `speed_left`/`speed_right`; avoidance is suspended. |
| `stopped` | The robot halts and avoidance stays suspended. |

* `POST /robots/{id}/stop` → `mode = stopped` (**stops obstacle avoidance**).
* `POST /robots/{id}/start` → `mode = auto` (**resumes obstacle avoidance**).
* `POST /robots/{id}/move` → `mode = manual` (drive directly).

TiagoLite avoidance uses the front sector of its **Hokuyo lidar**; the Create
uses its **bumpers and cliff sensors**.

## Running

1. Install Webots R2025a.
2. Install the supervisor's Python dependencies (the controllers only need
   Pillow, which is listed in each `requirements.txt`):
   ```bash
   pip install flask pillow
   ```
3. Open `irobot/scooting/worlds/scooting.wbt` in Webots.

The REST API starts automatically on **http://localhost:5000**.

## REST API reference

```
GET  /robots                         List every robot (pose, mode, sensors)
GET  /robots/{id}                    Single robot detail (pose, mode, sensors)
GET  /robots/{id}/sensors            Robot internal sensor data only
POST /robots/{id}/move               Manual drive (controllable robots only)
                                     Body: {"speed_left": <float>, "speed_right": <float>}
                                     Sets mode=manual (suspends obstacle avoidance)
POST /robots/{id}/stop               Halt + stop obstacle avoidance (mode=stopped)
POST /robots/{id}/start              Resume obstacle avoidance (mode=auto)
GET  /robots/{id}/camera             On-board camera as base64 JPEG
GET  /robots/{id}/camera/stream      On-board camera as MJPEG live stream
GET  /god/camera                     Top-down god-view camera as base64 JPEG
GET  /god/camera/stream              Top-down god-view camera as MJPEG live stream
POST /simulation/pause               Pause the simulation
POST /simulation/resume              Resume the simulation
GET  /simulation/time                Current simulated time (seconds)
```

* `id` is one of `TIAGO_1`, `TIAGO_2`, `CREATE`, `BLIMP`, `RANGEROVER`.
  The numeric aliases `1`, `2`, `3` map to the controllable robots
  (`TIAGO_1`, `TIAGO_2`, `CREATE`).
* `move`, `stop` and `start` return **403** for `BLIMP` and `RANGEROVER`
  (they are camera + sensors only).

### Sensor data (`GET /robots/{id}/sensors`)

Each controller writes a JSON snapshot that the supervisor serves verbatim.

| Robot | Notable fields |
|-------|----------------|
| `TIAGO_1`, `TIAGO_2` | `mode`, `speed_left/right`, `wheel_left/right`, `front_distance` (lidar), `imu_rpy`, `gyro`, `accel` |
| `CREATE` | `mode`, `speed_left/right`, `wheel_left/right`, `bumper_left/right`, `cliff[4]` |
| `BLIMP` | `gps`, `imu_rpy`, `gyro`, `accel`, `ds0`, `ds6` |
| `RANGEROVER` | `speed_kmh`, `steering_angle`, `autodrive`, `gps[3]` |

## Examples (curl)

```bash
# List all robots
curl http://localhost:5000/robots

# Drive TIAGO_1 forward (manual mode → avoidance suspended)
curl -X POST http://localhost:5000/robots/TIAGO_1/move \
     -H 'Content-Type: application/json' \
     -d '{"speed_left": 3.0, "speed_right": 3.0}'

# Stop TIAGO_1 and stop its obstacle avoidance
curl -X POST http://localhost:5000/robots/TIAGO_1/stop

# Resume autonomous obstacle avoidance
curl -X POST http://localhost:5000/robots/TIAGO_1/start

# Get the Create's sensor data
curl http://localhost:5000/robots/CREATE/sensors

# Save the Range Rover camera frame locally
curl http://localhost:5000/robots/RANGEROVER/camera | python3 -c \
  "import sys,json,base64; d=json.load(sys.stdin); open('rover.jpg','wb').write(base64.b64decode(d['data']))"

# View the Blimp camera as a live MJPEG stream (browser or VLC):
#   http://localhost:5000/robots/BLIMP/camera/stream

# Top-down map overview
curl http://localhost:5000/god/camera | python3 -c \
  "import sys,json,base64; d=json.load(sys.stdin); open('map.jpg','wb').write(base64.b64decode(d['data']))"
```

## Verification

The camera transport, the REST API and the controller decision logic are
covered by in-memory checks that run without Webots:

* the Flask app served by `api_supervisor.py` (list/detail, move/stop/start,
  403 for non-controllable robots, sensors, camera snapshots, god camera,
  numeric aliases, shared-memory and file frame pulls);
* the `tiago_lite` and `create_controller` decision logic (cruise when clear,
  turn on obstacle/bumper, manual speeds, halt when stopped);
* the `scooting_io` camera publisher shared-memory round-trip producing a valid
  JPEG.

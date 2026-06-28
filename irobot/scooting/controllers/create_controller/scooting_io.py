"""
Shared IPC helpers for the *scooting* Webots robot controllers.

A copy of this module lives in every Python controller directory so that each
controller can publish, with no central dependency:

* its on-board camera frame, encoded to JPEG and written either to a POSIX
  shared-memory block (fast, in-memory path consumed by ``api_supervisor``) or
  to an atomic JPEG file as a fallback, and
* a JSON snapshot of its sensors in the system temp directory.

The shared-memory block is created by ``api_supervisor`` before the robot
controllers start; this module only attaches to it.
"""

import io
import json
import os
import struct
import tempfile
import time

try:
    from multiprocessing.shared_memory import SharedMemory
except ImportError:  # pragma: no cover - very old Python
    SharedMemory = None

try:
    from PIL import Image
except ImportError:  # pragma: no cover - resolved via requirements.txt in Webots
    Image = None

_SHM_HEADER = 8           # [seq: uint32 LE][length: uint32 LE]
_TMP = tempfile.gettempdir()


class CameraPublisher:
    """Publish JPEG camera frames over shared memory (or a file fallback)."""

    def __init__(self, robot_name, quality=75):
        self.robot_name = robot_name
        self.quality = quality
        self._seq = 0
        self._shm = None
        if SharedMemory is not None:
            shm_name = f"webots_{robot_name}_camera_shm"
            for _ in range(20):           # retry ~2 s for start-up ordering
                try:
                    self._shm = SharedMemory(name=shm_name, create=False)
                    break
                except FileNotFoundError:
                    time.sleep(0.1)
            if self._shm is None:
                print(f"[{robot_name}] camera shm not found, using file fallback")

    def publish(self, camera):
        """Encode and publish one frame from a Webots ``Camera`` device."""
        if camera is None or Image is None:
            return
        try:
            raw = camera.getImage()
            if not raw:
                return
            img = Image.frombytes(
                "RGBA",
                (camera.getWidth(), camera.getHeight()),
                raw,
                "raw",
                "BGRA",
            )
            buf = io.BytesIO()
            img.convert("RGB").save(buf, format="JPEG", quality=self.quality)
            jpeg = buf.getvalue()
        except (OSError, ValueError, RuntimeError) as exc:
            print(f"[{self.robot_name}] camera encoding error: {exc}")
            return

        if self._shm is not None:
            # Write payload, then length, then seq last so a reader never sees
            # a torn frame.
            length = len(jpeg)
            try:
                self._shm.buf[_SHM_HEADER:_SHM_HEADER + length] = jpeg
                struct.pack_into("<I", self._shm.buf, 4, length)
                self._seq += 1
                struct.pack_into("<I", self._shm.buf, 0, self._seq)
                return
            except (ValueError, struct.error) as exc:
                print(f"[{self.robot_name}] camera shm write error: {exc}")
        # File fallback (atomic replace).
        cam_file = os.path.join(_TMP, f"webots_{self.robot_name}_camera.jpg")
        tmp_file = cam_file + ".tmp"
        with open(tmp_file, "wb") as fh:
            fh.write(jpeg)
        os.replace(tmp_file, cam_file)

    def close(self):
        if self._shm is not None:
            try:
                self._shm.close()       # detach only; supervisor owns the block
            except Exception:
                pass


def write_sensors(robot_name, sensors):
    """Atomically write a sensor snapshot dict to the temp-dir JSON file."""
    path = os.path.join(_TMP, f"webots_{robot_name}_state.json")
    tmp_path = path + ".tmp"
    try:
        with open(tmp_path, "w") as fh:
            json.dump(sensors, fh)
        os.replace(tmp_path, path)
    except OSError as exc:
        print(f"[{robot_name}] sensor write error: {exc}")

"""
Dummy controller Tiago Lite
Forward / Backward loop test (robust version)
"""

from controller import Robot

# =========================
# INIT
# =========================
robot = Robot()
timestep = int(robot.getBasicTimeStep())



# =========================
# CAMERA
# =========================
camera = robot.getDevice("Astra rgb")  # ou "front_camera"
if camera:
    camera.enable(timestep)
    print("[OK] Camera enabled")
else:
    print("[WARN] No camera found")



for i in range(robot.getNumberOfDevices()):
    dev = robot.getDeviceByIndex(i)
    print("DEVICE:", dev.getName(), "TYPE:", dev.getNodeType())
    
    
# =========================
# MOTORS (safe init)
# =========================
def safe_device(name):
    dev = robot.getDevice(name)
    if dev is None:
        print(f"[ERROR] Device not found: {name}")
    return dev

left_motor = safe_device("wheel_left_joint")
right_motor = safe_device("wheel_right_joint")

if left_motor:
    left_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)

if right_motor:
    right_motor.setPosition(float("inf"))
    right_motor.setVelocity(0.0)

# =========================
# SPEED CONFIG (IMPORTANT FIX)
# =========================
SPEED = 2.5   # ⚠️ 100 est beaucoup trop élevé pour Webots wheels

# =========================
# STATE MACHINE
# =========================
state = 0
step_count = 0
STEP_DURATION = 200   # plus réaliste

# =========================
# MAIN LOOP
# =========================
while robot.step(timestep) != -1:

    # safety check (important)
    if left_motor is None or right_motor is None:
        continue

    # switch state
    if step_count >= STEP_DURATION:
        state = 1 - state
        step_count = 0

    # forward / backward
    if state == 0:
        left_motor.setVelocity(SPEED)
        right_motor.setVelocity(SPEED)
    else:
        left_motor.setVelocity(-SPEED)
        right_motor.setVelocity(-SPEED)

    step_count += 1
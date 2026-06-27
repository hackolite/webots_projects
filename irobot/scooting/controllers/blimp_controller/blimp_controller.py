"""
Controleur Blimp - Navigation inertielle + Auto-stabilisation
=============================================================
- Auto-stab attitude : correcteur PD sur roll/pitch via IMU
- Auto-stab altitude : maintien de la hauteur GPS si aucune commande verticale
- Smoothing des commandes pilote
- Amortissement angulaire via Gyro

Controles UNIVERSELS (Évite les bugs AZERTY) :
  FLECHES   : avant / arriere / lacet gauche / droite
  A / E    : monter / descendre  (desactive l'autostab altitude)
  ESPACE   : stop moteurs
  R        : frein urgence
"""

from controller import Robot, Keyboard, InertialUnit, GPS
import math
import random

robot = Robot()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

# --- Debug devices ---
n = robot.getNumberOfDevices()
print(f"[BLIMP] {n} devices :")
for i in range(n):
    d = robot.getDeviceByIndex(i)
    print(f"  [{i}] '{d.getName()}' type={d.getNodeType()}")

# --- IMU (InertialUnit) ---
imu = robot.getDevice("inertial unit")
if not imu:
    imu = robot.getDevice("imu")
if imu:
    imu.enable(timestep)
    print("[OK] IMU activee")
else:
    print("[WARN] Pas d'IMU - auto-stab attitude desactivee")

# --- GPS ---
gps = robot.getDevice("gps")
if gps:
    gps.enable(timestep)
    print("[OK] GPS actif")

# --- Gyro ---
gyro = robot.getDevice("gyro")
if gyro:
    gyro.enable(timestep)
    print("[OK] Gyro actif")

# --- Accelerometre ---
accel = robot.getDevice("accelerometer")
if accel:
    accel.enable(timestep)
    print("[OK] Accelerometre actif")

# --- Camera ---
camera = robot.getDevice("camera")
if camera:
    camera.enable(timestep)
    print(f"[OK] Camera {camera.getWidth()}x{camera.getHeight()}")

# --- Capteurs distance ---
ds0 = robot.getDevice("ds0")
ds6 = robot.getDevice("ds6")
if ds0: ds0.enable(timestep)
if ds6: ds6.enable(timestep)

# --- Moteurs ---
m1 = robot.getDevice("motor1")   # avant  axe X
m2 = robot.getDevice("motor2")   # arriere axe X
m3 = robot.getDevice("motor3")   # vertical axe Z

for m, name in [(m1,"motor1"),(m2,"motor2"),(m3,"motor3")]:
    if m:
        m.setPosition(float('inf'))
        m.setVelocity(0.0)
        print(f"[OK] {name} pret")
    else:
        print(f"[WARN] {name} introuvable")

# --- Clavier ---
keyboard = Keyboard()
keyboard.enable(timestep)

# ============================================================
# Parametres
# ============================================================

# Smoothing commandes pilote
SMOOTH_H_ATTACK = 0.12  # montee rapide du boost horizontal
SMOOTH_H_DECAY  = 0.50  # descente tres rapide => boost momentane uniquement
SMOOTH_YAW = 0.04
SMOOTH_V   = 0.04

# Trainee
DRAG_H   = 0.04   # tres faible => haute inertie (blimp coaste longtemps)
DRAG_V   = 0.30
DRAG_YAW = 0.35

# Vitesses max
VMAX_H   = 1.0
VMAX_V   = 1.0
VMAX_YAW = 1.0

# Poussee max pilote
THRUST_H   = 1.5   # reduit : le boost est bref, pas besoin de beaucoup de force
THRUST_V   = 3.0
THRUST_YAW = 2.0

# Derive aleatoire
DRIFT = 0.0

# Echelle moteurs
MOTOR_SCALE_H = 1.0
MOTOR_SCALE_V = 1.0

# Puissance de base pour le stationnaire vertical
# F = thrustConstants[0] * omega => hover omega = m*g / 50 = 3*9.81/50 = 0.589 rad/s (linéaire)
# ou sqrt(m*g/50) = 0.767 rad/s (quadratique). On utilise 0.7 comme valeur intermédiaire.
HOVER_OMEGA = 0.7

# --- Auto-stab attitude (roll / pitch) ---
KP_ATT = 2.0
KD_ATT = 0.6
MAX_ATT_CORR = 4.0
ATT_DEADBAND = 0.02
ATT_SATURATE = 0.8

# --- Auto-stab altitude ---
KP_ALT = 0.3
KI_ALT = 0.0
KD_ALT = 0.15
MAX_ALT_CORR = 1.5
ALT_DEADBAND = 0.02

# --- Amortissement lacet gyro ---
KD_YAW_GYRO = 1.0

# --- Stabilisation démarrage ---
# Au démarrage : m1/m2 éteints, m3 maintient l'altitude
print("[BLIMP] Stabilisation initiale (2s)...")
for _ in range(int(2000 / timestep)):
    if m1: m1.setVelocity(0.0)
    if m2: m2.setVelocity(0.0)
    if m3: m3.setVelocity(HOVER_OMEGA)  # maintien altitude uniquement
    robot.step(timestep)
print("[BLIMP] Pret !\n")

# ============================================================
# Etat interne
# ============================================================
vx   = 0.0
vz   = 0.0
vyaw = 0.0

cmd_x_smooth   = 0.0
cmd_yaw_smooth = 0.0
cmd_z_smooth   = 0.0

# Auto-stab altitude
target_altitude  = None
alt_error_prev   = 0.0
alt_integral     = 0.0
pilot_wants_alt  = False

# Auto-stab attitude
roll_prev  = 0.0
pitch_prev = 0.0

log_timer = 0.0

def smooth(current, target, rate):
    return current + (target - current) * rate

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

print("Controles :")
print("  FLECHES (Haut/Bas) avant/arriere   FLECHES (Gauche/Droite) lacet   A/E altitude")
print("  ESPACE stop              R    frein urgence")
print("  Auto-stab attitude et altitude ACTIFS\n")

while robot.step(timestep) != -1:

    # Fix écran noir caméra
    if camera:
        camera.getImage()

    # --------------------------------------------------------
    # 1. Lecture capteurs
    # --------------------------------------------------------
    roll = pitch = yaw = 0.0
    if imu:
        rpy = imu.getRollPitchYaw()
        roll  = rpy[0]
        pitch = rpy[1]
        yaw   = rpy[2]

    yaw_rate = 0.0
    if gyro:
        gyro_vals = gyro.getValues()
        yaw_rate = gyro_vals[2]  # vitesse angulaire axe Z (lacet)

    accel_vals = [0.0, 0.0, 0.0]
    if accel:
        accel_vals = accel.getValues()

    altitude = None
    pos = [0.0, 0.0, 0.0]
    if gps:
        pos = gps.getValues()
        altitude = pos[2]
        if target_altitude is None:
            target_altitude = altitude
            print(f"[AUTOSTAB] Altitude cible initiale : {target_altitude:.2f} m")

    # --------------------------------------------------------
    # 2. Commandes pilote (Correction AZERTY via touches Flèches)
    # --------------------------------------------------------
    key = keyboard.getKey()

    target_x   = 0.0
    target_yaw = 0.0
    target_z   = 0.0
    braking    = False
    pilot_wants_alt = False

    if key == Keyboard.UP:
        target_x = THRUST_H
    elif key == Keyboard.DOWN:
        target_x = -THRUST_H

    if key == Keyboard.LEFT:
        target_yaw = THRUST_YAW
    elif key == Keyboard.RIGHT:
        target_yaw = -THRUST_YAW

    if key in (ord('A'), ord('a')):
        target_z = THRUST_V
        pilot_wants_alt = True
        if target_altitude is not None and altitude is not None:
            target_altitude = altitude
    elif key in (ord('E'), ord('e')):
        target_z = -THRUST_V
        pilot_wants_alt = True
        if target_altitude is not None and altitude is not None:
            target_altitude = altitude

    if key in (ord('R'), ord('r')):
        braking = True

    if key == ord(' '):
        cmd_x_smooth = cmd_yaw_smooth = cmd_z_smooth = 0.0
        vx = vz = vyaw = 0.0
        if altitude is not None:
            target_altitude = altitude

    # --------------------------------------------------------
    # 3. Smoothing commandes
    # --------------------------------------------------------
    # Horizontal : montee moderee, descente rapide => boost momentane
    h_rate = SMOOTH_H_ATTACK if target_x != 0.0 else SMOOTH_H_DECAY
    cmd_x_smooth   = smooth(cmd_x_smooth,   target_x,   h_rate)
    cmd_yaw_smooth = smooth(cmd_yaw_smooth, target_yaw, SMOOTH_YAW)
    cmd_z_smooth   = smooth(cmd_z_smooth,   target_z,   SMOOTH_V)

    if braking:
        vx   *= 0.5
        vz   *= 0.5
        vyaw *= 0.5
        cmd_x_smooth   *= 0.5
        cmd_yaw_smooth *= 0.5
        cmd_z_smooth   *= 0.5

    # --------------------------------------------------------
    # 4. Integration inertielle
    # --------------------------------------------------------
    vx   += (cmd_x_smooth   - DRAG_H   * vx)   * dt
    vz   += (cmd_z_smooth   - DRAG_V   * vz)   * dt
    vyaw += (cmd_yaw_smooth - DRAG_YAW * vyaw) * dt

    vx   = clamp(vx,   -VMAX_H,   VMAX_H)
    vz   = clamp(vz,   -VMAX_V,   VMAX_V)
    vyaw = clamp(vyaw, -VMAX_YAW, VMAX_YAW)

    # --------------------------------------------------------
    # 5. Auto-stab ATTITUDE (roll / pitch)
    # --------------------------------------------------------
    corr_roll  = 0.0
    corr_pitch = 0.0

    if imu:
        droll  = (roll  - roll_prev)  / dt
        dpitch = (pitch - pitch_prev) / dt

        if abs(roll) > ATT_SATURATE or abs(pitch) > ATT_SATURATE:
            corr_roll  = 0.0
            corr_pitch = 0.0
        else:
            roll_corr_input  = roll  if abs(roll)  > ATT_DEADBAND else 0.0
            pitch_corr_input = pitch if abs(pitch) > ATT_DEADBAND else 0.0

            corr_roll  = (KP_ATT * roll_corr_input  + KD_ATT * droll)
            corr_pitch = (KP_ATT * pitch_corr_input + KD_ATT * dpitch)

            corr_roll  = clamp(corr_roll,  -MAX_ATT_CORR, MAX_ATT_CORR)
            corr_pitch = clamp(corr_pitch, -MAX_ATT_CORR, MAX_ATT_CORR)

        roll_prev  = roll
        pitch_prev = pitch

    # --------------------------------------------------------
    # 6. Auto-stab ALTITUDE
    # --------------------------------------------------------
    corr_alt = 0.0

    if gps and altitude is not None and target_altitude is not None and not pilot_wants_alt:
        alt_error = target_altitude - altitude

        if abs(alt_error) > ALT_DEADBAND:
            alt_integral = clamp(alt_integral + alt_error * dt, -1.0, 1.0)
            dalt = (alt_error - alt_error_prev) / dt
            corr_alt = KP_ALT * alt_error + KI_ALT * alt_integral + KD_ALT * dalt
            corr_alt = clamp(corr_alt, -MAX_ALT_CORR, MAX_ALT_CORR)
            alt_error_prev = alt_error
        else:
            alt_error_prev = 0.0

    # --------------------------------------------------------
    # 7. Amortissement lacet via Gyro
    # --------------------------------------------------------
    yaw_damp = clamp(-KD_YAW_GYRO * yaw_rate, -MAX_ATT_CORR, MAX_ATT_CORR)

    # --------------------------------------------------------
    # 8. Commandes moteurs finales (bridées sous 10)
    # --------------------------------------------------------
    base1 = (vx + vyaw + yaw_damp) * MOTOR_SCALE_H
    base2 = (vx - vyaw - yaw_damp) * MOTOR_SCALE_H

    omega1 = base1 - corr_pitch + corr_roll
    omega2 = base2 + corr_pitch + corr_roll

    omega1 = clamp(omega1, -9.5, 9.5)
    omega2 = clamp(omega2, -9.5, 9.5)

    if pilot_wants_alt:
        omega3 = HOVER_OMEGA + (vz * MOTOR_SCALE_V)
    else:
        omega3 = HOVER_OMEGA + corr_alt

    omega3 = clamp(omega3, 0.0, 9.5)

    if m1: m1.setVelocity(omega1)
    if m2: m2.setVelocity(omega2)
    if m3: m3.setVelocity(omega3)

    # --------------------------------------------------------
    # 9. Log toutes les secondes
    # --------------------------------------------------------
    log_timer += dt
    if log_timer >= 1.0:
        log_timer = 0.0

        if braking:             cmd_str = "FREIN   "
        elif target_x > 0.01:   cmd_str = "AVANT   "
        elif target_x < -0.01: cmd_str = "ARRIERE "
        elif target_yaw > 0.01:cmd_str = "GAUCHE  "
        elif target_yaw <-0.01:cmd_str = "DROITE  "
        elif target_z > 0.01:  cmd_str = "MONTE   "
        elif target_z < -0.01: cmd_str = "DESCEND "
        else:                   cmd_str = "STOP    "

        pos_str = f"x={pos[0]:.1f} y={pos[1]:.1f} z={altitude:.1f}" if (gps and altitude is not None) else "GPS N/A"
        alt_err = (target_altitude - altitude) if (target_altitude is not None and altitude is not None) else 0.0

        print(f"[NAV] {pos_str} | cmd={cmd_str}"
              f"| vx={vx:.2f} vz={vz:.2f} vyaw={vyaw:.3f}"
              f"| roll={math.degrees(roll):.1f}° pitch={math.degrees(pitch):.1f}°"
              f"| yaw_rate={math.degrees(yaw_rate):.1f}°/s"
              f"| alt_err={alt_err:.2f}m corr_alt={corr_alt:.2f}"
              f"| accel=[{accel_vals[0]:.2f},{accel_vals[1]:.2f},{accel_vals[2]:.2f}]")


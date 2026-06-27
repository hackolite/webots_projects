"""
Controleur Blimp - Navigation inertielle + Auto-stabilisation
=============================================================
- Multi-touches : combinaison libre (ex. avant + lacet simultanes)
- Auto-stab attitude  : correcteur PD sur roll/pitch via IMU
- Auto-stab altitude  : maintien de la hauteur GPS si aucune commande verticale
- Maintien de cap     : PD yaw quand aucune commande de lacet
- Smoothing des commandes pilote
- Amortissement angulaire via Gyro

Controles UNIVERSELS (Évite les bugs AZERTY) :
  FLECHES   : avant / arriere / lacet gauche / droite  (combinables !)
  A / E    : monter / descendre  (desactive l'autostab altitude)
  ESPACE   : stop moteurs + gel altitude et cap
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
SMOOTH_H_ATTACK = 0.18  # montee plus reactive
SMOOTH_H_DECAY  = 0.35  # descente rapide => boost momentane uniquement
SMOOTH_YAW = 0.08       # lacet plus vif
SMOOTH_V   = 0.06       # vertical plus reactif

# Trainee
# DRAG_H eleve : constante de temps vx = 1/DRAG_H/dt ≈ 2.5s => arret net apres relachement fleche
DRAG_H   = 0.40
DRAG_V   = 0.30
DRAG_YAW = 0.40

# Vitesses max
# VMAX_H limite : couple piquer = 50*omega*1.5 m ; couple restaurant pendule = 44*theta
# A omega=0.12 : equilibre a theta ≈ 0.20 rad (11.7°), bien en dessous de ATT_SATURATE
VMAX_H   = 0.12
VMAX_V   = 1.0
VMAX_YAW = 1.0

# Poussee max pilote
THRUST_H   = 0.25  # reduit : evite un couple a piquer excessif (moteurs 1.5m au-dessus du CoM)
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
# KP_ATT reduit : evite l'inversion des moteurs (inversion si pitch > VMAX_H/KP_ATT)
# Avec KP_ATT=0.3 et VMAX_H=0.12 : inversion seulement a partir de 22.9° (>> equilibre ~12°)
KP_ATT = 0.3
KD_ATT = 0.10
MAX_ATT_CORR = 2.0
ATT_DEADBAND = 0.02
# Seuil reduit : laisser l'effet pendule dominer aux grands angles plutot que de corriger
ATT_SATURATE = 0.5

# --- Auto-stab altitude ---
KP_ALT = 0.4
KI_ALT = 0.02
KD_ALT = 0.20
MAX_ALT_CORR = 1.5
ALT_DEADBAND = 0.02

# --- Amortissement lacet gyro ---
KD_YAW_GYRO = 1.0

# --- Maintien de cap (heading hold) ---
KP_YAW_HOLD = 1.8   # correcteur proportionnel cap
KD_YAW_HOLD = 0.5   # amortissement derive de cap
MAX_YAW_HOLD_CORR = 1.5
YAW_HOLD_DEADBAND = 0.015

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

# Maintien de cap
target_heading   = None
yaw_hold_active  = False
heading_err_prev = 0.0

log_timer = 0.0

def smooth(current, target, rate):
    return current + (target - current) * rate

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

print("Controles :")
print("  FLECHES (Haut/Bas) avant/arriere   FLECHES (Gauche/Droite) lacet   A/E altitude")
print("  Combinaisons libres (ex. avant + lacet simultanes)")
print("  ESPACE stop + gel altitude/cap    R frein urgence")
print("  Auto-stab attitude, altitude et cap ACTIFS\n")

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
    # 2. Commandes pilote - lecture MULTI-TOUCHES simultanees
    # --------------------------------------------------------
    keys = set()
    k = keyboard.getKey()
    while k != -1:
        keys.add(k)
        k = keyboard.getKey()

    target_x   = 0.0
    target_yaw = 0.0
    target_z   = 0.0
    braking    = False
    pilot_wants_alt = False
    pilot_wants_yaw = False

    if Keyboard.UP in keys:
        target_x = THRUST_H
    elif Keyboard.DOWN in keys:
        target_x = -THRUST_H

    if Keyboard.LEFT in keys:
        target_yaw = THRUST_YAW
        pilot_wants_yaw = True
    elif Keyboard.RIGHT in keys:
        target_yaw = -THRUST_YAW
        pilot_wants_yaw = True

    if keys & {ord('A'), ord('a')}:
        target_z = THRUST_V
        pilot_wants_alt = True
        if target_altitude is not None and altitude is not None:
            target_altitude = altitude
    elif keys & {ord('E'), ord('e')}:
        target_z = -THRUST_V
        pilot_wants_alt = True
        if target_altitude is not None and altitude is not None:
            target_altitude = altitude

    if keys & {ord('R'), ord('r')}:
        braking = True

    if ord(' ') in keys:
        cmd_x_smooth = cmd_yaw_smooth = cmd_z_smooth = 0.0
        vx = vz = vyaw = 0.0
        if altitude is not None:
            target_altitude = altitude
        if imu:
            target_heading = yaw
            heading_err_prev = 0.0

    # Quand le pilote relache le lacet, geler le cap courant
    if not pilot_wants_yaw and imu:
        if not yaw_hold_active:
            target_heading = yaw
            heading_err_prev = 0.0
            yaw_hold_active = True
    else:
        yaw_hold_active = False

    # --------------------------------------------------------
    # 3. Smoothing commandes
    # --------------------------------------------------------
    # Horizontal : montee moderee, descente rapide => boost momentane
    h_rate = SMOOTH_H_ATTACK if target_x != 0.0 else SMOOTH_H_DECAY
    cmd_x_smooth   = smooth(cmd_x_smooth,   target_x,   h_rate)
    cmd_yaw_smooth = smooth(cmd_yaw_smooth, target_yaw, SMOOTH_YAW)
    cmd_z_smooth   = smooth(cmd_z_smooth,   target_z,   SMOOTH_V)

    if braking:
        vx   *= 0.4
        vz   *= 0.4
        vyaw *= 0.4
        cmd_x_smooth   *= 0.4
        cmd_yaw_smooth *= 0.4
        cmd_z_smooth   *= 0.4

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
        # Utiliser le gyro pour le terme dérivé (plus stable que les differences finies)
        if gyro:
            droll  = gyro_vals[0]   # vitesse angulaire axe X (roll)
            dpitch = gyro_vals[1]   # vitesse angulaire axe Y (pitch)
        else:
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
    # 7. Maintien de cap (heading hold)
    # --------------------------------------------------------
    corr_yaw_hold = 0.0

    if imu and yaw_hold_active and target_heading is not None and not pilot_wants_yaw:
        # Erreur angulaire normalisee dans [-pi, pi]
        heading_err = (target_heading - yaw + math.pi) % (2 * math.pi) - math.pi

        if abs(heading_err) > YAW_HOLD_DEADBAND:
            dheading = (heading_err - heading_err_prev) / dt
            corr_yaw_hold = KP_YAW_HOLD * heading_err + KD_YAW_HOLD * dheading
            corr_yaw_hold = clamp(corr_yaw_hold, -MAX_YAW_HOLD_CORR, MAX_YAW_HOLD_CORR)
        heading_err_prev = heading_err

    # --------------------------------------------------------
    # 8. Amortissement lacet via Gyro
    # --------------------------------------------------------
    yaw_damp = clamp(-KD_YAW_GYRO * yaw_rate, -MAX_ATT_CORR, MAX_ATT_CORR)

    # --------------------------------------------------------
    # 9. Commandes moteurs finales (bridées sous 10)
    # --------------------------------------------------------
    base1 = (vx + vyaw + yaw_damp + corr_yaw_hold) * MOTOR_SCALE_H
    base2 = (vx - vyaw - yaw_damp - corr_yaw_hold) * MOTOR_SCALE_H

    # corr_pitch en mode commun : reduit/augmente la poussee des deux moteurs egalement
    # => couple a piquer negatif/positif qui compense l'inclinaison (correct physiquement)
    # corr_roll supprime : les moteurs horizontaux ne peuvent pas créer de couple de roulis
    # (la stabilite en roulis est assuree par l'effet pendule, CoM 1.5 m sous le centre)
    omega1 = base1 - corr_pitch
    omega2 = base2 - corr_pitch

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
    # 10. Log toutes les secondes
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
        hdg_err = (target_heading - yaw + math.pi) % (2 * math.pi) - math.pi if (target_heading is not None and imu) else 0.0

        print(f"[NAV] {pos_str} | cmd={cmd_str}"
              f"| vx={vx:.2f} vz={vz:.2f} vyaw={vyaw:.3f}"
              f"| roll={math.degrees(roll):.1f}° pitch={math.degrees(pitch):.1f}°"
              f"| yaw_rate={math.degrees(yaw_rate):.1f}°/s"
              f"| alt_err={alt_err:.2f}m corr_alt={corr_alt:.2f}"
              f"| hdg_err={math.degrees(hdg_err):.1f}° corr_yaw={corr_yaw_hold:.2f}"
              f"| accel=[{accel_vals[0]:.2f},{accel_vals[1]:.2f},{accel_vals[2]:.2f}]")


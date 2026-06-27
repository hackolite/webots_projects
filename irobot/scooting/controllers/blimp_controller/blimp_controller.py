"""
Controleur Blimp - Navigation inertielle + Auto-stabilisation
=============================================================
- Multi-touches : combinaison libre (ex. avant + lacet simultanes)
- Anti-pique     : amortissement D-only du taux de tangage en croisiere
                   (le terme P est desactive pendant la poussee horizontale :
                    le pendule fournit deja le rappel naturel, le P augmenterait
                    la raideur sans amortissement suffisant → balancier sous-amorti)
                   KD_ATT_CRUISE << KD_ATT : evite l'inversion des moteurs quand
                   corr_pitch depasse vx, ce qui creerait une retroaction positive
                   amplifiant le tangage (oscillation croissante)
- Anti-dérive    : maintien de position GPS (hold XY) quand aucune commande
                   horizontale — correction PD vitesse + position en axe avant
- Auto-stab altitude  : maintien de la hauteur GPS si aucune commande verticale
- Maintien de cap     : PD yaw quand aucune commande de lacet
- Smoothing des commandes pilote
- Amortissement angulaire via Gyro (D-only pitch en croisiere, gain adaptatif)
- Log inertiels complets : position GPS, vitesse estimée, RPY, gyro XYZ, accél XYZ

Controles UNIVERSELS (Évite les bugs AZERTY) :
  FLECHES   : avant / arriere / lacet gauche / droite  (combinables !)
  A / E    : monter / descendre  (desactive l'autostab altitude)
  ESPACE   : stop moteurs + gel altitude, cap ET position
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
# DRAG_H : constante de temps vx = 1/DRAG_H ≈ 0.5s => arret net en ~1.5s après relachement fleche
DRAG_H   = 2.0
DRAG_V   = 0.30
DRAG_YAW = 0.40

# Vitesses max
# La poussee des helices Webots est QUADRATIQUE : F = thrustConstants[0]*omega^2 = 50*omega^2
# (et non lineaire). A omega=0.12 la poussee horizontale n'etait que 50*0.12^2 = 0.72 N sur un
# dirigeable de 3 kg => translation quasi nulle, donc maniabilite tres faible.
# On releve VMAX_H pour obtenir une poussee utile tout en restant stable :
#   omega=0.45 => poussee = 50*0.45^2 = 10.1 N (accel ~3.4 m/s^2)
#   couple a piquer = 10.1*0.8 = 8.1 Nm ; couple restaurateur pendule = 53*sin(theta)
#   => equilibre a theta ≈ 0.154 rad (8.8°), bien en dessous de ATT_SATURATE (0.5 rad)
VMAX_H   = 0.45
VMAX_V   = 1.0
VMAX_YAW = 1.0

# Poussee max pilote
# vx tend vers THRUST_H/DRAG_H en regime etabli => THRUST_H = DRAG_H*VMAX_H = 2.0*0.45 = 0.9
THRUST_H   = 0.1   # permet d'atteindre VMAX_H tout en gardant le couple a piquer maitrise
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
# Geometrie : CoM a Z=-1.8 m, moteurs a Z=-1.0 m → bras de levier = 0.8 m
# Pendule : k = m*g*L = 3*9.81*1.8 = 52.97 Nm/rad
# Amortissement critique D-only : c_crit = 2*sqrt(k*J) = 2*sqrt(52.97*4.7) = 31.56 Nm.s/rad
# c_eff (2 moteurs) = 2 * thrustConst * KD_ATT * lever = 2*50*KD_ATT*0.8 = 80*KD_ATT
#
# Stationnaire (|vx|≈0) : P+D. KD_ATT=1.0 → c_eff=80 → zeta=2.54 (suramorti) ✓
# Croisiere : D-only avec KD_ATT_CRUISE.
#   Probleme si KD_ATT trop grand en croisiere : corr_pitch = KD_ATT * dpitch peut
#   depasser vx (≈0.12) et INVERSER les moteurs. L'inversion genere une poussee
#   arriere + couple cabrage oppose, puis les moteurs repartent en avant → boucle
#   de retroaction positive qui AMPLIFIE le tangage.
#   Solution : KD_ATT_CRUISE << KD_ATT, plus verrou adaptatif sur |corr_pitch| ≤ 0.9*|vx|.
#   KD_ATT_CRUISE=0.35 → c_eff=28 → zeta≈1.0 (critique) sans saturer pour |dpitch|<0.34 rad/s
KP_ATT = 0.5           # utilise uniquement en stationnaire (pas de poussee horizontale)
KD_ATT = 1.0           # amortissement stationnaire (zeta≈2.54, suramorti)
KD_ATT_CRUISE = 0.35   # amortissement croisiere (zeta≈1.0, sans inversion moteur)
MAX_ATT_CORR = 2.0
ATT_DEADBAND = 0.01
# Seuil au-delà duquel la correction est suspendue : laisser l'effet pendule dominer
ATT_SATURATE = 0.5
# Seuil vx en-dessous duquel le mode stationnaire (P+D) est actif
# Hysteresis : entree croisiere a VX_CRUISE_ENTER, sortie a VX_CRUISE_EXIT
# evite le chatter de mode quand vx oscille autour du seuil
VX_CRUISE_ENTER = 0.015   # |vx| > seuil → passage en mode croisiere (D-only pitch)
VX_CRUISE_EXIT  = 0.005   # |vx| < seuil → retour en mode stationnaire (PD pitch)
# Bande morte sur le taux de tangage (rad/s) en mode croisiere
# filtre le bruit gyro sans degrader l'amortissement
DPITCH_DEADBAND = 0.005   # rad/s (~0.3°/s)

# --- Anti-dérive : maintien de position horizontale (GPS) ---
# Activé dès que le pilote relâche les touches avant/arrière
# Correction PD en axe avant (body frame) : position + vitesse estimée GPS
KP_POS      = 0.15   # proportionnel position (rad/s / m)
KD_VEL_HOLD = 1.0    # dérivé vitesse (rad/s / m·s⁻¹)
MAX_POS_CORR = 0.08  # limite ≈ 67 % de VMAX_H
VEL_SMOOTH   = 0.25  # lissage passe-bas vitesse GPS (0=figé, 1=instantané)

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

# Mode croisiere pitch (D-only) : True quand blimp en mouvement horizontal
# Initialise a False (stationnaire au demarrage)
in_cruise_pitch = False

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

# Anti-dérive : maintien de position horizontale
target_pos_x   = None
target_pos_y   = None
pos_hold_active = False
prev_pos_x     = None
prev_pos_y     = None
vel_est_x      = 0.0   # vitesse monde estimée (m/s), filtrée
vel_est_y      = 0.0

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
    gyro_vals = [0.0, 0.0, 0.0]
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

        # Estimation vitesse horizontale monde (passe-bas) pour l'anti-dérive
        if prev_pos_x is not None:
            raw_vx = (pos[0] - prev_pos_x) / dt
            raw_vy = (pos[1] - prev_pos_y) / dt
            vel_est_x = smooth(vel_est_x, raw_vx, VEL_SMOOTH)
            vel_est_y = smooth(vel_est_y, raw_vy, VEL_SMOOTH)
        prev_pos_x = pos[0]
        prev_pos_y = pos[1]

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
    pilot_wants_h   = False

    if Keyboard.UP in keys:
        target_x = THRUST_H
        pilot_wants_h = True
    elif Keyboard.DOWN in keys:
        target_x = -THRUST_H
        pilot_wants_h = True

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
        if gps:
            target_pos_x = pos[0]
            target_pos_y = pos[1]
            pos_hold_active = True

    # Quand le pilote relache le lacet, geler le cap courant
    if not pilot_wants_yaw and imu:
        if not yaw_hold_active:
            target_heading = yaw
            heading_err_prev = 0.0
            yaw_hold_active = True
    else:
        yaw_hold_active = False

    # Anti-dérive : capturer la position GPS dès que la touche avant/arrière est relâchée
    if not pilot_wants_h and gps and prev_pos_x is not None:
        if not pos_hold_active:
            target_pos_x = pos[0]
            target_pos_y = pos[1]
            pos_hold_active = True
    elif pilot_wants_h:
        pos_hold_active = False

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
            # Mise a jour du mode croisiere avec hysteresis :
            #   evite le chatter de mode quand vx oscille autour du seuil
            if pilot_wants_h or (abs(vx) > VX_CRUISE_ENTER):
                in_cruise_pitch = True
            elif abs(vx) < VX_CRUISE_EXIT:
                in_cruise_pitch = False

            if in_cruise_pitch:
                # D-only avec gain reduit : evite que corr_pitch > vx inverse les moteurs.
                # Verrou adaptatif supplementaire : |corr_pitch| ≤ 0.9*|vx| garantit
                # que omega1/omega2 ne changent jamais de signe sous l'effet du pitch.
                dpitch_filtered = dpitch if abs(dpitch) > DPITCH_DEADBAND else 0.0
                max_cruise_corr = clamp(abs(vx) * 0.9, 0.0, MAX_ATT_CORR)
                corr_pitch = clamp(KD_ATT_CRUISE * dpitch_filtered,
                                   -max_cruise_corr, max_cruise_corr)
            else:
                pitch_corr_input = pitch if abs(pitch) > ATT_DEADBAND else 0.0
                corr_pitch = KP_ATT * pitch_corr_input + KD_ATT * dpitch

            corr_pitch = clamp(corr_pitch, -MAX_ATT_CORR, MAX_ATT_CORR)

            # Roll : toujours D-only (les moteurs horizontaux ne créent pas de
            # couple de roulis — la stabilite roulis est assuree par le pendule)
            corr_roll = clamp(KD_ATT * droll, -MAX_ATT_CORR, MAX_ATT_CORR)

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
    # 7b. Anti-dérive : maintien de position horizontale (GPS)
    # --------------------------------------------------------
    corr_pos = 0.0

    if pos_hold_active and gps and target_pos_x is not None:
        # Erreur position monde → projection sur l'axe avant du blimp (body frame)
        err_world_x = target_pos_x - pos[0]
        err_world_y = target_pos_y - pos[1]
        err_fwd = err_world_x * math.cos(yaw) + err_world_y * math.sin(yaw)

        # Vitesse avant estimée (body frame)
        vel_fwd = vel_est_x * math.cos(yaw) + vel_est_y * math.sin(yaw)

        corr_pos = clamp(
            KP_POS * err_fwd - KD_VEL_HOLD * vel_fwd,
            -MAX_POS_CORR, MAX_POS_CORR
        )

    # --------------------------------------------------------
    # 8. Amortissement lacet via Gyro
    # --------------------------------------------------------
    yaw_damp = clamp(-KD_YAW_GYRO * yaw_rate, -MAX_ATT_CORR, MAX_ATT_CORR)

    # --------------------------------------------------------
    # 9. Commandes moteurs finales (bridées sous 10)
    # --------------------------------------------------------
    base1 = (vx + corr_pos + vyaw + yaw_damp + corr_yaw_hold) * MOTOR_SCALE_H
    base2 = (vx + corr_pos - vyaw - yaw_damp - corr_yaw_hold) * MOTOR_SCALE_H

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

        pos_str = f"x={pos[0]:.2f} y={pos[1]:.2f} z={altitude:.2f}" if (gps and altitude is not None) else "GPS N/A"
        vel_str = f"vx={vel_est_x:.2f} vy={vel_est_y:.2f}" if gps else "vel N/A"
        alt_err = (target_altitude - altitude) if (target_altitude is not None and altitude is not None) else 0.0
        hdg_err = (target_heading - yaw + math.pi) % (2 * math.pi) - math.pi if (target_heading is not None and imu) else 0.0
        pos_err_fwd = 0.0
        if pos_hold_active and gps and target_pos_x is not None:
            pos_err_fwd = (target_pos_x - pos[0]) * math.cos(yaw) + (target_pos_y - pos[1]) * math.sin(yaw)

        print(
            f"[NAV]  cmd={cmd_str} | {pos_str} | {vel_str}\n"
            f"       vx_cmd={vx:.3f} vz_cmd={vz:.3f} vyaw={vyaw:.3f}\n"
            f"[IMU]  roll={math.degrees(roll):.2f}° pitch={math.degrees(pitch):.2f}° yaw={math.degrees(yaw):.2f}°\n"
            f"[GYRO] wx={math.degrees(gyro_vals[0]):.2f}°/s wy={math.degrees(gyro_vals[1]):.2f}°/s wz={math.degrees(yaw_rate):.2f}°/s\n"
            f"[ACCEL]ax={accel_vals[0]:.3f} ay={accel_vals[1]:.3f} az={accel_vals[2]:.3f} m/s²\n"
            f"[CORR] alt_err={alt_err:.3f}m corr_alt={corr_alt:.3f}"
            f" | hdg_err={math.degrees(hdg_err):.2f}° corr_yaw={corr_yaw_hold:.3f}"
            f" | pos_err_fwd={pos_err_fwd:.3f}m corr_pos={corr_pos:.3f}"
            f" | corr_pitch={corr_pitch:.3f}"
        )

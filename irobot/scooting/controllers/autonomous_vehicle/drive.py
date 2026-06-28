"""
Range Rover Sport SVR - Contrôleur pavé numérique
==================================================
  8 : avancer
  2 : reculer
  4 : tourner gauche
  6 : tourner droite
  5 : stop

Utilise l'API Driver de Webots (Car proto).
"""

from controller import Robot, Keyboard
try:
    from vehicle import Driver
    USE_DRIVER = True
except ImportError:
    USE_DRIVER = False

# ============================================================
# Paramètres
# ============================================================
MAX_SPEED      = 60.0   # km/h
SPEED_STEP     =  5.0   # km/h par impulsion
MAX_STEER      =  0.5   # rad (angle volant max)
STEER_STEP     =  0.05  # rad par step
STEER_RETURN   =  0.1   # vitesse de retour au centre quand aucune touche

# Touches pavé numérique
KEY_FWD   = ord('8')
KEY_BACK  = ord('2')
KEY_LEFT  = ord('4')
KEY_RIGHT = ord('6')
KEY_STOP  = ord('5')

# ============================================================
# Init
# ============================================================
if USE_DRIVER:
    robot = Driver()
else:
    robot = Robot()

timestep = int(robot.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(timestep)

speed   = 0.0   # km/h
steer   = 0.0   # rad

print("[RR] Contrôleur Range Rover prêt")
print("  8=avant  2=arrière  4=gauche  6=droite  5=stop")

# ============================================================
# Boucle principale
# ============================================================
while robot.step(timestep) != -1:

    # --- Lecture touches (multi) ---
    keys = set()
    k = keyboard.getKey()
    while k != -1:
        keys.add(k)
        k = keyboard.getKey()

    # --- Vitesse ---
    if KEY_FWD in keys:
        speed = min(speed + SPEED_STEP, MAX_SPEED)
    elif KEY_BACK in keys:
        speed = max(speed - SPEED_STEP, -MAX_SPEED / 2)
    elif KEY_STOP in keys:
        speed = 0.0

    # --- Direction ---
    if KEY_LEFT in keys:
        steer = max(steer - STEER_STEP, -MAX_STEER)
    elif KEY_RIGHT in keys:
        steer = min(steer + STEER_STEP,  MAX_STEER)
    else:
        # Retour progressif au centre
        if steer > 0:
            steer = max(0.0, steer - STEER_RETURN)
        elif steer < 0:
            steer = min(0.0, steer + STEER_RETURN)

    # --- Commandes Driver ---
    if USE_DRIVER:
        robot.setCruisingSpeed(speed)
        robot.setSteeringAngle(steer)
    else:
        print(f"[RR] speed={speed:.1f} km/h  steer={steer:.3f} rad  (Driver non disponible)")

    # --- Log ---
    if USE_DRIVER:
        actual_speed = robot.getCurrentSpeed()
        print(f"\r[RR] speed={actual_speed:.1f} km/h  steer={steer:.3f} rad  "
              f"gear={robot.getGear()}    ", end="", flush=True)
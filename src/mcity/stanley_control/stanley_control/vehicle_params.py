"""
Mustang Mach-E vehicle constants.

Sources / notes
---------------
WHEELBASE       : Ford spec — 2,984 mm (118.1 in)
VEHICLE_WEIGHT  : ~2,066 kg Standard Range RWD / ~2,255 kg AWD;
                  2,100 kg used as a nominal mid-point — update per variant.
WHEEL_DIAMETER  : 225/55R19 standard tire
                  sidewall = 225 × 0.55 = 123.75 mm
                  diameter = 19 × 25.4 + 2 × 123.75 = 730 mm = 0.730 m
STEERING_RATIO  : Ford Mach-E electric rack-and-pinion; approx 17:1.
                  Verify from CAN steer-angle vs wheel-angle telemetry.

ACCTABLE / BRAKETABLE in speed_control.py were measured on the MKZ and
must be re-collected via pedal-sweep tests on the Mach-E before deployment.
"""

# ── Drivetrain / geometry ────────────────────────────────────────────────────
WHEELBASE        = 2.984    # m  — front-to-rear axle distance
VEHICLE_WEIGHT   = 2100.0   # kg — nominal; adjust for battery/trim variant
WHEEL_DIAMETER   = 0.730    # m  — 225/55R19 OEM tire
STEERING_RATIO   = 17.0     # steering-wheel deg / front-wheel deg

# ── Speed / actuator limits ──────────────────────────────────────────────────
MAX_SPEED        = 25.0     # m/s  (~56 mph soft cap, same as MKZ)
MAX_ACC          =  3.00    # m/s²
MAX_DCC          = -8.00    # m/s²
MAX_THROTTLE     =  0.45    # pedal fraction [0–1]
MAX_BRAKE        =  0.70    # pedal fraction [0–1]

# ── Steering limits ──────────────────────────────────────────────────────────
import math
MAX_STEERING_WHEEL_ANGLE = 2.5 * math.pi          # rad  (±450°)
MAX_WHEEL_ANGLE          = MAX_STEERING_WHEEL_ANGLE / STEERING_RATIO  # rad

# Ballbot configuration. Edit these with YOUR calibrated values.
#
# Axes (used everywhere in this project): stand in front of the robot and
# look down at the plate. +x = to your right, +y = away from you.
# Angles around the plate are measured anticlockwise from +x:
#   0 = right, 90 = away from you, 180 = left, 270 = towards you.

# Servo order: servo 1, 2, 3 = GPIO17, GPIO27, GPIO22 (physical pins 11, 13, 15)
PINS = [17, 27, 22]

# Pulse mapping, same as the working interactive script: 0 deg = 500 us, 180 deg = 2500 us
MIN_PULSE = 500
MAX_PULSE = 2500

# Servo angles in DEGREES. 0 = arm upright (corner up), 90 = arm down (corner down).
LEVEL_DEG = [21, 30, 30]   # calibrated level angles: Servo 1 = 21 deg, Servos 2 & 3 = 30 deg
MIN_DEG   = [0, 0, 0]      # free movement range per servo
MAX_DEG   = [90, 90, 90]

# -1 because a BIGGER angle moves the arm DOWN (lowers that corner).
# If one servo is mounted mirrored, set its entry to +1.
DIR = [-1, -1, -1]

# Where each servo sits around the plate, seen from above, in degrees
ANGLE = [90, 210, 330]

DEG_PER_UNIT = 15   # servo degrees at a corner for a full tilt command of 1.0

# ==============================================================================
# Camera Defaults
# ==============================================================================
CAM_DEV    = 0      # default camera device index (/dev/video0)
CAM_WIDTH  = 640    # capture width
CAM_HEIGHT = 480    # capture height
CAM_FPS    = 30     # requested capture FPS

# ==============================================================================
# State Estimator (Kalman Filter) Parameters
# ==============================================================================
# Process noise (acceleration disturbance variance):
# Higher = more responsive to fast movements, lower = smoother velocity
EST_PROCESS_NOISE = 6.0

# Measurement noise variance (in normalized plate units^2):
# Higher = filter trusts measurements less (filters out pixel jitter)
EST_MEAS_NOISE = 0.0001

# Max seconds before declaring the ball officially lost
EST_LOST_TIMEOUT = 0.35

# ==============================================================================
# Feedback Controller (PD / PID) Parameters
# ==============================================================================
# Command: gx = -(Kp * error_x + Ki * int_error_x + Kd * vx)
# Start testing with Stage 6: P-only (CTRL_KP = 0.4 - 0.6, CTRL_KD = 0)
# Then Stage 7: add Kd (CTRL_KD = 0.25 - 0.45) to damp oscillations
CTRL_KP = 0.55       # Proportional gain
CTRL_KD = 0.35       # Derivative gain
CTRL_KI = 0.02       # Integral gain (eliminates steady-state bias)

# Maximum tilt magnitude in [0, 1] (0.70 * 15 deg = ~10.5 degrees maximum tilt)
CTRL_MAX_TILT = 0.70

# Error deadband around center in normalized plate units (2.5% of plate radius)
CTRL_DEADBAND = 0.025

# Slew-rate limit: maximum tilt command change per second (prevents jerk)
CTRL_MAX_SLEW = 4.0


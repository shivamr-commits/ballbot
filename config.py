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
LEVEL_DEG = [45, 45, 45]   # angle where the plate is level, per servo  <-- PUT YOUR VALUES
MIN_DEG   = [0, 0, 0]      # free movement range per servo
MAX_DEG   = [90, 90, 90]

# -1 because a BIGGER angle moves the arm DOWN (lowers that corner).
# If one servo is mounted mirrored, set its entry to +1.
DIR = [-1, -1, -1]

# Where each servo sits around the plate, seen from above, in degrees
ANGLE = [90, 210, 330]

DEG_PER_UNIT = 15   # servo degrees at a corner for a full tilt command of 1.0

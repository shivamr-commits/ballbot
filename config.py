# Ballbot configuration. Edit these with YOUR calibrated values.
#
# Axes (used everywhere in this project): stand in front of the robot and
# look down at the plate. +x = to your right, +y = away from you.
# Angles are measured anticlockwise from +x:
#   0 = right, 90 = away from you, 180 = left, 270 = towards you.

# Servo order: servo 1, 2, 3 = GPIO17, GPIO27, GPIO22 (physical pins 11, 13, 15)
PINS = [17, 27, 22]

LEVEL  = [1500, 1500, 1500]   # pulse (us) where the plate is level, per servo
DIR    = [1, 1, 1]            # +1 if a BIGGER pulse RAISES that corner, -1 if it lowers it
MIN_US = [1000, 1000, 1000]   # smallest safe pulse per servo
MAX_US = [2000, 2000, 2000]   # largest safe pulse per servo

# Where each servo sits around the plate, seen from above, in degrees
ANGLE = [90, 210, 330]

US_PER_UNIT = 200   # pulse change at a corner for a full tilt command of 1.0

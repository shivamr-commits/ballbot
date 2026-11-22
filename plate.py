# NOTE: do not rename this file to platform.py (that name clashes with a
# built-in Python module and breaks OpenCV / numpy imports).
import math
import config as C


def tilt_to_pulses(gx, gy):
    """gx, gy in [-1, 1] = the direction the ball should roll (+x right, +y away)."""
    m = math.hypot(gx, gy)
    if m > 1:
        gx, gy = gx / m, gy / m
    pulses = []
    for i, a in enumerate(C.ANGLE):
        t = math.radians(a)
        height = -(gx * math.cos(t) + gy * math.sin(t))   # the corner on the roll side goes down
        pulses.append(C.LEVEL[i] + C.DIR[i] * C.US_PER_UNIT * height)
    return pulses

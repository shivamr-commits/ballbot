import math
import time
from servos import Servos
from plate import tilt_to_pulses

S = 0.5   # test tilt size (0 to 1)
moves = [("level", 0, 0),
         ("roll RIGHT (+x)", S, 0), ("level", 0, 0),
         ("roll LEFT (-x)", -S, 0), ("level", 0, 0),
         ("roll AWAY (+y)", 0, S), ("level", 0, 0),
         ("roll TOWARDS YOU (-y)", 0, -S), ("level", 0, 0)]

s = Servos()
try:
    for name, gx, gy in moves:
        p = s.write(tilt_to_pulses(gx, gy))
        print(f"{name:22s} pulses {p}")
        time.sleep(2)

    print("circle: low side goes round anticlockwise, 2 turns")
    t0 = time.time()
    while time.time() - t0 < 8:
        a = 2 * math.pi * (time.time() - t0) / 4
        s.write(tilt_to_pulses(S * math.cos(a), S * math.sin(a)))
        time.sleep(0.03)

    print("level"); s.level(); time.sleep(1)
finally:
    s.close()
    print("Done")

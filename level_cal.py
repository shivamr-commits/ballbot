"""Level the plate: hold all three servos and nudge each one in degrees.
0 deg = arm upright (corner UP), 90 deg = arm down (corner DOWN).
When the plate is flat, copy the printed LEVEL_DEG line into config.py (on the laptop).
"""
from servos import Servos
import config as C

HELP = """Type a command, then Enter:
  1 2 3      select servo
  a / d      corner UP / DOWN by 1 deg     A / D   by 5 deg
  =45        set selected servo to an exact angle
  all=45     set all three to the same angle
  u / j      move the WHOLE plate up / down 1 deg (stays level)
  q          quit and print the LEVEL_DEG line"""

ang = list(C.LEVEL_DEG)
s = Servos()
sel = 0


def show():
    a = s.write(ang)
    for i in range(3):
        ang[i] = a[i]                      # keep within MIN_DEG..MAX_DEG
    print("   angles:", [f"S{i+1}={a[i]}" for i in range(3)])


try:
    show()
    print(HELP)
    while True:
        c = input(f"[servo {sel+1} = {ang[sel]} deg] > ").strip()
        if c in ("1", "2", "3"):
            sel = int(c) - 1
            continue
        if c in ("a", "d", "A", "D"):
            ang[sel] += {"a": -1, "d": 1, "A": -5, "D": 5}[c]
        elif c in ("u", "j"):
            for i in range(3):
                ang[i] += -1 if c == "u" else 1
        elif c.startswith("all="):
            try:
                ang = [float(c[4:])] * 3
            except ValueError:
                print("Use like all=45"); continue
        elif c.startswith("="):
            try:
                ang[sel] = float(c[1:])
            except ValueError:
                print("Use like =45"); continue
        elif c == "q":
            break
        else:
            print(HELP); continue
        show()
finally:
    s.close()
    print("\nPut this line in config.py:")
    print(f"LEVEL_DEG = [{ang[0]}, {ang[1]}, {ang[2]}]")

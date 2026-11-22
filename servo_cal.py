import lgpio, json, os

PINS = [17, 27, 22]          # servo 1, 2, 3 (physical pins 11, 13, 15)
FILE = "servo_cal.json"
HARD_MIN, HARD_MAX = 500, 2500

cal = json.load(open(FILE)) if os.path.exists(FILE) else {}
h = lgpio.gpiochip_open(0)
for p in PINS:
    lgpio.gpio_claim_output(h, p)
pulse = {p: cal.get(str(p), {}).get("level", 1500) for p in PINS}

HELP = """Type a command, then Enter:
  1 2 3      select servo
  a / d      -10 / +10 us     A / D   -50 / +50 us
  =1500      jump to an exact pulse
  l          save current pulse as LEVEL
  min / max  save current pulse as smallest / largest SAFE pulse
  up / down  a BIGGER pulse makes this corner go up / down
  q          save and quit"""


def send(p):
    pulse[p] = max(HARD_MIN, min(HARD_MAX, pulse[p]))
    lgpio.tx_servo(h, p, pulse[p])


try:
    for p in PINS:
        send(p)
    sel = PINS[0]
    print(HELP)
    while True:
        c = input(f"[servo {PINS.index(sel)+1} GPIO{sel} = {pulse[sel]} us] > ").strip()
        e = cal.setdefault(str(sel), {})
        if c in ("1", "2", "3"):
            sel = PINS[int(c) - 1]
        elif c in ("a", "d", "A", "D"):
            pulse[sel] += {"a": -10, "d": 10, "A": -50, "D": 50}[c]
            send(sel)
        elif c.startswith("="):
            try:
                pulse[sel] = int(c[1:]); send(sel)
            except ValueError:
                print("Use like =1500")
        elif c == "l":
            e["level"] = pulse[sel]; print("level saved")
        elif c in ("min", "max"):
            e[c] = pulse[sel]; print(c, "saved")
        elif c in ("up", "down"):
            e["up"] = 1 if c == "up" else -1; print("direction saved")
        elif c == "q":
            break
        else:
            print(HELP)
finally:
    json.dump(cal, open(FILE, "w"), indent=2)
    for p in PINS:
        lgpio.tx_servo(h, p, 0)
    lgpio.gpiochip_close(h)
    print("Saved and stopped:", cal)

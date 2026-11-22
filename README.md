# Ballbot: full project context

This file is written so that a person or another AI model can pick up the project with no other background. It records the goal, the exact hardware, the exact software state, every command that was run, every measurement taken, every mistake made, and what remains to do. Facts marked **(confirmed)** came from the user's own terminal output or measurements. Facts marked **(assumed)** were not confirmed and should be verified before relying on them.

---

## 1. Goal

Build a ball-following robot ("ballbot"):

1. A USB camera connected to a Raspberry Pi 400 detects a coloured ball.
2. The ball's offset from the image centre (`dx`, `dy`) is computed.
3. Servo motors move the camera (or a mechanism) so that it follows the ball.

The user originally wanted 60 FPS capture. The current camera (Logitech C270) is limited to 30 FPS, which is enough for servo-based tracking.

The user works **step by step**, runs each step, then reports the terminal output. Keep instructions short, one stage at a time, and wait for the output before continuing.

---

## 2. Current status at a glance

| Area | Status |
|---|---|
| Pi 400 boots, VNC works | Working (confirmed) |
| Camera (Logitech C270) streams video | Working with YUYV at 640x480, 30 FPS (confirmed) |
| Ball detection (v1, slider based, `ball_detect.py`) | Detected the ball but was inaccurate and FPS dropped after a replug (confirmed) |
| Ball detection (v2, click to calibrate, `ball_detect2.py`) | Written and cloned onto the Pi. **Not yet reported as tested by the user.** |
| GitHub repo, pushed from laptop, cloned on Pi | Working (confirmed) |
| Servos 1, 2, 3 (MG996R on physical pins 11, 13, 15) | All connected and tested. Free movement 0 to 90 degrees. User reports all three calibrated (confirmed). Section 9 is now history. |
| Project goal | **Ball balancing plate** driven by 3 servos (see section 16 for the staged plan). |
| Servo angle mapping | **500 to 2500 us = 0 to 180 degrees** (confirmed working in the user's interactive `servo_move.py`). **0 degrees = arm upright (corner up), 90 degrees = arm down.** Free range 0 to 90. So `DIR = -1` (bigger angle lowers the corner). |
| Stage 2 (tilt mixing: `config.py`, `servos.py`, `plate.py`, `plate_test.py`) | Written, config in **degrees**. Not yet reported as tested. `LEVEL_DEG` in `config.py` must hold the user's real level angles. |
| Stage 3 (vision on the plate: `vision.py`) | Written and tested on synthetic frames (correct plate coordinates under rotation and mirroring; off-plate blobs rejected). Not yet run on the Pi. User asked to move on before reporting Stage 2 results. |
| Stage 5 (estimator & logger: `estimator.py`, `log_ball.py`) | **Fully Implemented.** 2D Kalman filter for latency compensation and smooth velocity estimation; CSV logging testbench. |
| Stages 6 & 7 (feedback controller: `controller.py`) | **Fully Implemented.** 2D PD/PID controller with error deadband, integrator anti-windup, saturation, and slew-rate limiting. |
| Stage 8 (closed-loop balancing: `balance.py`) | **Fully Implemented.** Real-time balancing loop with safe ball-lost leveling, live hotkey tuning, setpoint trajectories, and headless support. |

---

## 3. Hardware (all confirmed unless noted)

- **Computer:** Raspberry Pi 400 (keyboard-integrated Pi 4 class board). Own USB-C power supply. Hostname `hostenr312`, user `shivam_host`.
- **Access to the Pi:** VNC Viewer from a Windows laptop, plus the Pi's own terminal. A monitor is attached or available via micro-HDMI (the port nearest the USB-C power port).
- **Power key:** Fn + F10 powers the Pi 400 on after a shutdown.
- **Camera (current):** Logitech C270, USB ID `046d:0825`, appears as `/dev/video0` (video) and `/dev/video1` (metadata). Plugged into a USB port on the Pi directly.
- **Previous camera (replaced):** Sonix "USB 2.0 Camera", USB ID `0c45:62f1`. It only offered YUYV, advertised 60 FPS but delivered about 20 FPS, had no exposure controls, and repeatedly dropped off the USB bus. It is no longer used.
- **Servos:** 3 x MG996R.
- **Servo driver board:** **None.** The user does NOT have a PCA9685. Early instructions assumed one; ignore them.
- **Servo power:** a 5 V, 5 A DC adapter, used only for the servos. **How the adapter physically connects (barrel plug, bare wires, or screw terminals) was asked several times and never answered.** The user uses a breadboard with power rails.
- **Meter:** the user has a multimeter.
- **Laptop:** Windows 10, git 2.55.0 for Windows. Project folder on the laptop: `C:\Users\Apple\Desktop\ENR 213 & ENR312\ballbot`.

### Raspberry Pi 400 header facts

- The 40-pin header is **not labelled** with numbers. The case has **"Pin 1" engraved at the top right corner and "40" engraved at the bottom left corner** (confirmed by the user).
- Pin 1's row contains the odd pins (1, 3, 5, ...). The other row contains the even pins.
- Pin 1 is 3.3 V. Pins 2 and 4 are 5 V and sit in the other row at the same end. These are the danger pins.
- Pin map from the user's `pinout` command (confirmed):

```
 3V3  (1) (2)  5V
 GPIO2  (3) (4)  5V
 GPIO3  (5) (6)  GND
 GPIO4  (7) (8)  GPIO14
   GND  (9) (10) GPIO15
GPIO17 (11) (12) GPIO18
GPIO27 (13) (14) GND
GPIO22 (15) (16) GPIO23
   3V3 (17) (18) GPIO24
GPIO10 (19) (20) GND
 GPIO9 (21) (22) GPIO25
GPIO11 (23) (24) GPIO8
   GND (25) (26) GPIO7
 GPIO0 (27) (28) GPIO1
 GPIO5 (29) (30) GND
 GPIO6 (31) (32) GPIO12
GPIO13 (33) (34) GND
GPIO19 (35) (36) GPIO16
GPIO26 (37) (38) GPIO20
   GND (39) (40) GPIO21
```

---

## 4. Operating system and software environment

- **OS:** Raspberry Pi OS based on Debian 13 "trixie" (confirmed by the package versions `0.2.2-1~rpt1+trixie`).
- **Installed and confirmed:** `python3-lgpio 0.2.2`, `i2c-tools 4.4`, `v4l-utils 1.30.1`, `python3-opencv 4.10.0`, `python3-numpy 2.2.4`, `git`.
- **`pigpio` is NOT available** on this OS (`apt` says "Package pigpio is not available"). Earlier instructions that used `pigpio` / `pigpiod` are obsolete. The project now uses **`lgpio`**.
- **Virtual environment:** `~/ballbot/venv`, created with `python3 -m venv --system-site-packages venv` so the apt-installed OpenCV, numpy and lgpio are visible inside it.
- **I2C / `adafruit-circuitpython-servokit`:** these were part of the original PCA9685 plan. They are not needed now because there is no PCA9685.
- Harmless noise: the message `GLib-GObject-CRITICAL ... g_object_unref: assertion 'G_IS_OBJECT (object)' failed` appears when OpenCV windows close and can be ignored.

### Starting every new Pi terminal

```bash
cd ~/ballbot
source venv/bin/activate
```

The prompt should read `(venv) shivam_host@hostenr312:~/ballbot $`.

### Terminal usage notes (these caused real problems)

- In the Pi terminal, paste with **Ctrl + Shift + V** or right-click. Plain Ctrl + V does not paste.
- Pasting a long `cat > file << 'EOF'` block corrupted the file (the end of the script got scrambled). **Create files with `nano file.py` and paste the code into nano, or use the Text Editor.**
- In the Windows `cmd` terminal, pasting several lines at once merged them into one line (for example `maingit init`). Paste **one command at a time**.
- Typing a password in `sudo` shows nothing on screen. That is normal.

---

## 5. GitHub and file workflow

- **Repo:** `https://github.com/shivamr-commits/ballbot.git`, private, branch `main`.
- **Auth:** Git Credential Manager's browser sign-in failed (`fatal: Missing 'state' in response`). Pushing works with a **classic personal access token** (scope `repo`) entered as the password:

```bash
git -c credential.helper= push -u origin main
```

  Username: `shivamr-commits`. Password: the token (starts `ghp_`). Never commit the token.
- **Pi side:** the repo was cloned to `~/ballbot` on the Pi (a private repo needs the token on clone and pull too). Updates: `cd ~/ballbot && git pull`.
- **`.gitignore`** contains `venv/`, `hsv.json`, `__pycache__/` so the venv and the saved colour calibration stay local to each machine.

### Sync workflow (from Stage 2 on)

- The laptop folder is the single source of truth. Edit files (including `config.py` calibration values) on the laptop, push, then `git pull` on the Pi. Do not edit tracked files on the Pi, or the next pull will conflict.
- A zip (`ballbot_stage2.zip`) brought the repo up to date: lgpio `servo_test.py`, `servo_10deg.py`, `servo_cal.py`, `cam_setup.sh`, `ball_detect2.py`, the Stage 2 modules, `.gitignore` and this README.
- `cam_setup.sh` may lose its executable bit via Windows. Run it as `bash cam_setup.sh` rather than `chmod +x` on the Pi (a mode change shows up as a local edit).
- `.gitignore` also ignores `servo_cal.json` and `*.csv` (logs).
- Earlier scripts `cam_test.py` and `ball_detect.py` (the slider-based v1) existed on the Pi but are **not in the repo**.

---

## 6. Servo wiring

### Planned wiring for all three servos

Servos are driven directly from GPIO pins (no driver board), so the Pi sends only the signal.

| Servo | Signal wire (orange/yellow) to | GPIO |
|---|---|---|
| Servo 1 | physical pin 11 | GPIO17 |
| Servo 2 | physical pin 13 | GPIO27 |
| Servo 3 | physical pin 15 | GPIO22 |

| Connection | Goes to |
|---|---|
| All servo red wires | Adapter positive (+5 V) |
| All servo brown/black wires | Adapter negative (-) |
| Shared ground | Adapter negative (-) to **physical pin 9 (GND)** |

Rules:

- The adapter's + never connects to any Pi pin. Only its ground goes to the Pi (pin 9).
- The Pi keeps its own USB-C supply.
- Power the Pi off and unplug it, with the adapter unplugged, before changing any header wire. A slipped wire near pins 1, 2 or 4 can damage the Pi.
- Plug the adapter in last, after rechecking every wire. Keep servo arms free to move.
- Only four wires should touch the header: pins 9, 11, 13, 15.

### Actual current wiring (confirmed)

Only **servo 1** is connected. Ground and power are made with a breadboard rail: the negative rail holds the adapter's negative, the servo's brown wire, and pin 9. The positive rail holds the adapter's positive and the servo's red wire. The servo's signal wire goes to pin 11. The user states the connections have been checked and are fine.

### Power budget warning

An MG996R can draw about 2.5 A when stalled. Three servos under load can exceed the 5 A adapter. Test servos one at a time, keep power wiring short and thick, and consider a 470 uF or larger capacitor across the supply rails (long leg to +, stripe side to -).

---

## 7. Camera setup (Logitech C270)

### Facts (confirmed)

- Max frame rate is **30 FPS at every resolution**. The `60` that older code asked for was never real.
- Formats: `YUYV` and `MJPG`.
- **Use YUYV.** MJPG produced floods of `Corrupt JPEG data: N extraneous bytes before marker 0x..` errors and a very slow `v4l2-ctl` ramp-up, which pointed at USB data loss.
- The `v4l2-ctl --stream-count` fps numbers start low because they are a running average from zero. Read the last value.
- `vcgencmd get_throttled` returned `0x0` (no power problem on the Pi).
- `dmesg` shows `reset high-speed USB device number 5` each time a camera program starts or stops. These looked harmless since frames were clean afterwards.
- Resolution used for tracking: 640x480 (320x240 is the fallback if speed or bandwidth is a problem).

### C270 controls (confirmed from `v4l2-ctl --list-ctrls`)

`brightness`, `contrast`, `saturation`, `white_balance_automatic`, `gain` (default 64, observed 192), `power_line_frequency` (60 Hz), `white_balance_temperature`, `sharpness`, `backlight_compensation`, `auto_exposure` (menu: 1 = manual, 3 = aperture priority, default 3), `exposure_time_absolute` (1 to 10000, units of 0.1 ms, observed 419 = 41.9 ms), `exposure_dynamic_framerate` (bool, default observed 1).

### Why the frame rate and detection degraded

At the defaults (auto exposure about 42 ms, dynamic frame rate on) the camera cannot hold 30 FPS in dim light, blurs the ball, and auto white balance shifts the colours. **All camera settings reset to defaults whenever the camera is unplugged and replugged.** That is why detection got worse after a replug.

### `cam_setup.sh`

Run it after every replug and before detection. It disables dynamic frame rate, switches to manual exposure (150 = 15 ms), sets gain, and locks white balance. Bright lighting on the ball helps more than any setting.

```bash
./cam_setup.sh
python ball_detect2.py 0 640 480
```

If the image is too dark: raise `exposure_time_absolute` (stay below about 330 for 30 FPS) or `gain`, or add light. If the camera is not `/dev/video0`, check `v4l2-ctl --list-devices` and pass the right device.

---

## 8. Ball detection (`ball_detect2.py`)

- Opens the camera in YUYV at 640x480, 30 FPS, buffer size 1.
- **Click on the ball in the "Ball Detect" window** to calibrate. It samples the median HSV of an 11x11 patch around the click, saves `{"center": [h, s, v], "tol": [...]}` to `hsv.json`, and reloads it next run.
- Three trackbars in the "Controls" window set the tolerance around the sampled colour for H, S and V.
- Red hue wraparound (around 0 and 179) is handled in `make_mask`.
- Mask is cleaned with morphological open then close, contours are found, and a blob is accepted only if area is at least 150 and circularity above 0.6. The largest accepted blob is chosen.
- Position is smoothed with an exponential filter (0.6 old, 0.4 new).
- Draws the circle, centre dot, FPS, and prints `dx`, `dy`, `r` on the frame. `dx = cx - W/2`, `dy = cy - H/2`. These are the values intended to drive the servos.
- Press `q` to quit. Recalibrate by clicking again whenever lighting or camera settings change.

The user has not yet reported how v2 performs. The v1 script (hard-coded orange HSV defaults and six sliders) detected the ball but was inaccurate. Ask which failure mode appears: false detections, missed ball, or jumping. Also ask for the ball colour (never stated).

---

## 9. Servo problem: current investigation

### Symptoms (confirmed)

- Servo 1 **buzzes/jitters only while a script is sending pulses, and does not rotate.** It goes silent when the script ends (`Done` printed).
- A hold-at-1500 command produced a tiny movement.
- Turning the horn by hand with power off feels smooth.

### Measurements (confirmed)

- Servo red to brown: **5 V** at rest, **5.75 V** while buzzing (inside the MG996R's 4.8 to 7.2 V range, so the supply is not sagging).
- Signal pin (and orange to brown at the servo): **about 0.3 V**. This is expected. A multimeter averages the pulses: 1500 us in a 20 ms period at 3.3 V averages about 0.25 V. It proves pulses are present, not that their timing is clean.

### Ruled out or unlikely

- Missing signal at the servo, missing power, and voltage sag.
- Stripped gears (horn is smooth, though a clicking internal failure is not fully excluded).

### Still to test (in suggested order)

1. **Ground continuity.** With everything unplugged, use the multimeter's continuity mode: pin 9 to the servo's brown wire, and pin 9 to the adapter negative. Then, with power on and the script running, measure between pin 9 and the adapter negative (should be 0.00 to 0.05 V). Optionally move the ground jumper to pin 14.
2. **Swap in a different servo** on the same wiring. If it sweeps smoothly, servo 1 is faulty. If it also buzzes, suspect pulse timing. (The user was asked whether they have a spare. No answer yet.)
3. **Hardware PWM on pin 12 (GPIO18).** The Pi's software-timed `lgpio.tx_servo` pulses may be too unsteady for an MG996R. Proposed (not yet done or confirmed):
   - Move the signal wire from pin 11 to pin 12 with the Pi powered off.
   - Add `dtoverlay=pwm,pin=18,func=2` to `/boot/firmware/config.txt` and reboot.
   - Drive it through sysfs, for example (20 ms period, duty 1.5 ms for centre):

```bash
echo 0 | sudo tee /sys/class/pwm/pwmchip0/export
echo 20000000 | sudo tee /sys/class/pwm/pwmchip0/pwm0/period
echo 1500000 | sudo tee /sys/class/pwm/pwmchip0/pwm0/duty_cycle
echo 1 | sudo tee /sys/class/pwm/pwmchip0/pwm0/enable
```

   If `export` fails, run `ls /sys/class/pwm/` and check the chip name.
4. If hardware PWM works, only two hardware channels exist on a Pi 400, so three servos on steady pulses points to a **PCA9685 I2C board** (cheap, 16 channels, very steady pulses, ideal for tracking). Buying one is the likely long-term fix.

Do not leave a buzzing servo powered for long. Unplug the adapter between tests.

---

## 10. Pulse-width reference for MG996R

- Servo signal: 50 Hz (20 ms period), pulse 1000 to 2000 us for roughly 90 degrees of travel, 1500 us is centre. Many MG996R units reach about 500 to 2500 us for roughly 180 degrees, but avoid the extremes because the servo strains against its end stops.
- About 11.1 us per degree on the 1000 to 2000 us scale. 10 degrees is about 111 us.
- `lgpio.tx_servo(handle, gpio, pulse_us)` starts pulses, and a pulse of `0` stops them. Always stop pulses in a `finally` block.

---

## 11. History of changes and mistakes (so they are not repeated)

1. Original instructions assumed a PCA9685 and used `adafruit-circuitpython-servokit`. **The user has no PCA9685.**
2. Early scripts forced `MJPG`. The first camera had only YUYV. The C270 has both but MJPG is corrupt-prone here. **Use YUYV.**
3. 60 FPS was requested. **Neither camera can deliver it.** The OpenCV line `@ 60.0 fps` only echoed the request.
4. First camera (Sonix) was replaced because of USB dropouts and no exposure controls.
5. `pigpio` was chosen for servos, then found **unavailable** on this OS. Switched to `lgpio`.
6. Camera settings were lost on replug, which made detection worse. `cam_setup.sh` was created.
7. Pasting heredocs into the Pi terminal corrupted `servo_test.py`. The file was recreated with `nano`.
8. GitHub: the placeholder URL `YOURNAME` was used in the first remote; it was fixed with `git remote set-url`. Browser sign-in failed, token auth worked.
9. A "Pin 1 is the corner nearest..." assumption from standard Pi boards was wrong for the Pi 400. The engraved markings on the case are authoritative.

---

## 12. Code listings (current versions)

### `servo_test.py` (lgpio version, as currently on the Pi)

```python
import lgpio
import time

PIN = 17  # GPIO17 = physical pin 11

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, PIN)

CENTER, LEFT, RIGHT = 1500, 1000, 2000  # pulse width in microseconds


def move(pulse, wait=1.5):
    lgpio.tx_servo(h, PIN, pulse)  # 50 Hz servo pulse
    time.sleep(wait)


try:
    print("Center")
    move(CENTER)
    print("One side")
    move(LEFT)
    print("Other side")
    move(RIGHT)
    print("Back to center")
    move(CENTER)
finally:
    lgpio.tx_servo(h, PIN, 0)  # stop pulses
    lgpio.gpiochip_close(h)
    print("Done")
```

### `servo_10deg.py` (moves servos 1, 2, 3 one by one by 10 degrees)

```python
import lgpio
import time

# Physical pins 11, 13, 15 = GPIO17, GPIO27, GPIO22
PINS = [17, 27, 22]

CENTER = 1500          # microseconds (90 degrees position)
US_PER_DEG = 1000 / 90  # about 11.1 us per degree (1000-2000 us ~ 90 degrees)
DELTA = int(10 * US_PER_DEG)  # pulse change for 10 degrees (about 111 us)

h = lgpio.gpiochip_open(0)
for p in PINS:
    lgpio.gpio_claim_output(h, p)


def hold(pin, pulse, secs=1.0):
    lgpio.tx_servo(h, pin, pulse)
    time.sleep(secs)


try:
    for i, pin in enumerate(PINS, start=1):
        print(f"Servo {i} (GPIO{pin}): center")
        hold(pin, CENTER)
        print(f"Servo {i}: +10 degrees")
        hold(pin, CENTER + DELTA)
        print(f"Servo {i}: back to center")
        hold(pin, CENTER)
        lgpio.tx_servo(h, pin, 0)   # stop pulses to this servo before the next one
finally:
    for p in PINS:
        lgpio.tx_servo(h, p, 0)
    lgpio.gpiochip_close(h)
    print("Done")
```

### `cam_setup.sh`

```bash
#!/bin/bash
# Usage: ./cam_setup.sh [device]   (default /dev/video0)
D=${1:-/dev/video0}

v4l2-ctl -d $D --set-ctrl=exposure_dynamic_framerate=0
v4l2-ctl -d $D --set-ctrl=auto_exposure=1          # manual mode (must come first)
v4l2-ctl -d $D --set-ctrl=exposure_time_absolute=150
v4l2-ctl -d $D --set-ctrl=gain=100
v4l2-ctl -d $D --set-ctrl=white_balance_automatic=0
v4l2-ctl -d $D --set-ctrl=white_balance_temperature=4000

echo "Applied. Current values:"
v4l2-ctl -d $D --get-ctrl=auto_exposure,exposure_time_absolute,exposure_dynamic_framerate,gain,white_balance_automatic
```

Make it executable once: `chmod +x cam_setup.sh`.

### `ball_detect2.py`

```python
import cv2
import numpy as np
import time
import json
import os
import sys

# Usage: python ball_detect2.py [device] [width] [height]
dev = int(sys.argv[1]) if len(sys.argv) > 1 else 0
W = int(sys.argv[2]) if len(sys.argv) > 2 else 640
H = int(sys.argv[3]) if len(sys.argv) > 3 else 480
FILE = "hsv.json"

cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
if not cap.isOpened():
    print(f"Could not open /dev/video{dev}")
    raise SystemExit(1)

center = None            # [h, s, v] sampled from the ball
tol = [10, 70, 70]       # tolerance around the center
if os.path.exists(FILE):
    try:
        d = json.load(open(FILE))
        center, tol = d["center"], d["tol"]
        print("Loaded", FILE, center, tol)
    except Exception:
        pass


def save():
    if center is not None:
        json.dump({"center": center, "tol": tol}, open(FILE, "w"))


latest_hsv = None


def on_click(event, x, y, flags, param):
    global center
    if event == cv2.EVENT_LBUTTONDOWN and latest_hsv is not None:
        patch = latest_hsv[max(y - 5, 0):y + 6, max(x - 5, 0):x + 6].reshape(-1, 3)
        center = [int(v) for v in np.median(patch, axis=0)]
        save()
        print("Calibrated on HSV", center)


def make_mask(hsv, c, t):
    h, s, v = c
    slo, shi = max(s - t[1], 0), min(s + t[1], 255)
    vlo, vhi = max(v - t[2], 0), min(v + t[2], 255)
    hlo, hhi = h - t[0], h + t[0]

    def rng(a, b):
        return cv2.inRange(hsv, np.array([a, slo, vlo]), np.array([b, shi, vhi]))

    if hlo < 0:                      # red wraps around 0
        return rng(0, hhi) | rng(180 + hlo, 179)
    if hhi > 179:
        return rng(hlo, 179) | rng(0, hhi - 180)
    return rng(hlo, hhi)


cv2.namedWindow("Ball Detect")
cv2.setMouseCallback("Ball Detect", on_click)
cv2.namedWindow("Controls")
cv2.createTrackbar("H tol", "Controls", tol[0], 40, lambda x: None)
cv2.createTrackbar("S tol", "Controls", tol[1], 127, lambda x: None)
cv2.createTrackbar("V tol", "Controls", tol[2], 127, lambda x: None)

kernel = np.ones((5, 5), np.uint8)
prev, fps = time.time(), 0.0
sx = sy = None

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to read a frame.")
        break

    now = time.time()
    fps = 0.9 * fps + 0.1 * (1.0 / max(now - prev, 1e-6))
    prev = now

    tol = [cv2.getTrackbarPos("H tol", "Controls"),
           cv2.getTrackbarPos("S tol", "Controls"),
           cv2.getTrackbarPos("V tol", "Controls")]

    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV)
    latest_hsv = hsv
    mask = np.zeros(hsv.shape[:2], np.uint8)

    if center is not None:
        mask = make_mask(hsv, center, tol)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        best, best_area = None, 0
        for c in contours:
            area = cv2.contourArea(c)
            peri = cv2.arcLength(c, True)
            if area < 150 or peri == 0:
                continue
            circularity = 4 * np.pi * area / (peri * peri)
            if circularity > 0.6 and area > best_area:
                best, best_area = c, area

        if best is not None:
            (x, y), r = cv2.minEnclosingCircle(best)
            sx = x if sx is None else 0.6 * sx + 0.4 * x
            sy = y if sy is None else 0.6 * sy + 0.4 * y
            cx, cy = int(sx), int(sy)
            cv2.circle(frame, (cx, cy), int(r), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"dx={cx - W // 2} dy={cy - H // 2} r={int(r)}",
                        (10, H - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 2)
        else:
            sx = sy = None
    else:
        cv2.putText(frame, "Click on the ball to calibrate", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Ball Detect", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

save()
cap.release()
cv2.destroyAllWindows()
```

---

## 13. Suggested roadmap

1. **Fix servo 1** using the tests in section 9 (continuity, spare servo, hardware PWM). Do not wire servos 2 and 3 until servo 1 sweeps cleanly, or three servos will share one unsolved fault.
2. Run `servo_10deg.py` once servo 1 works, then add servos 2 and 3 one at a time and rerun.
3. Run `./cam_setup.sh` then `python ball_detect2.py 0 640 480`, click the ball, and tune the tolerance sliders. Record the FPS, ball colour, and failure mode.
4. Write a tracking script that combines detection and servo output:
   - Normalise the error: `ex = dx / (W/2)`, `ey = dy / (H/2)`, each in [-1, 1].
   - Proportional control with a small gain, a dead band around zero, and clamping each pulse to a safe range (for example 1000 to 2000 us) so the servo never hits its end stops.
   - Add slew limiting (small maximum pulse change per frame) to stop jitter.
   - Decide which servo handles pan and which handles tilt. **The role of the third servo has not been stated by the user** and should be asked.
   - Stop pulses (`tx_servo(..., 0)`) when the ball is lost for some time, to prevent buzzing and heat.
5. Run the tracker headless (no `imshow`) for the final robot, with a start-up script.
6. Strongly consider a PCA9685 board if software PWM stays jittery.

---

## 14. Questions still open for the user

- How does the 5 V adapter physically connect to the breadboard (barrel plug, bare wires, screw terminals, or a jack-to-terminal adapter)?
- Is there a spare servo available for the swap test?
- What colour is the ball, and how does `ball_detect2.py` behave on the Pi (false detections, missed ball, or jumping)?
- What does each of the three servos move (pan, tilt, something else)?
- Do the continuity and ground voltage tests in section 9 pass?
- Is the repo in sync (was the Pi's `servo_test.py` pushed, and are `servo_10deg.py` and this README committed)?

---

## 16. Ball balancing plan (current, supersedes section 13)

Final loop, about 30 times a second: camera finds ball (x, y) in plate coordinates → velocity estimated from frame-to-frame change → PD (later +I) per axis gives a tilt command → tilt clamped (about ±8 degrees) → `plate.tilt_to_pulses` mixes it into 3 servo pulses → slew limited → `Servos.write`.

Axes: stand in front of the robot looking down. +x = right, +y = away from you. Angles anticlockwise from +x (0 right, 90 away, 180 left, 270 towards you). Tilt command `(gx, gy)` in [-1, 1] = the direction the ball should roll.

| Stage | Build | Pass test |
|---|---|---|
| 1 | Servo calibration (`servo_cal.py`, done by user) | Ball at centre stays roughly still |
| 2 | Tilt mixing (`config.py`, `servos.py`, `plate.py`, `plate_test.py`) | Named side goes low for +x, -x, +y, -y; smooth anticlockwise circle; centre does not bob |
| 3 | Vision on the plate (`vision.py`): ball position in plate coordinates, plate centre marked | (x, y) about 0 at centre, about ±1 at edges, FPS near 30 |
| 4 | Sign check: tilt and watch which way the camera says the ball rolls | Camera axes agree with tilt axes; signs fixed in config |
| 5 | Logging position and velocity to CSV | Velocity smooth, not noisy |
| 6 | P control only, low gain (`controller.py`) | Correct reaction; oscillation expected |
| 7 | Add D, tune | Ball settles near centre |
| 8 | Small I, ball-lost levelling, headless, setpoint paths (`balance.py`) | Holds 1 minute, recovers from a nudge |

Stage 2 debugging: one corner wrong → flip its `DIR`; one axis mirrored → swap two `ANGLE` entries; all directions off by about 120 degrees → rotate the `ANGLE` list; moves too small or large → `DEG_PER_UNIT` (10 to 25).

Note: `servo_move.py` stops pulses after each move (servo goes limp). The balancer cannot do that: `Servos` keeps pulses running so the plate is held. If holding causes buzzing, that is software PWM jitter from `lgpio`; the fix is a PCA9685 board.

Stage 3 details (`vision.py`): class `BallTracker(dev)`; `read()` returns `{t, found, u, v, r, x, y}` with x, y in plate units (0 at centre, +1 at the right / far edge). Plate calibration = press `p` and click centre, right edge (+x), far edge (+y); saved to `plate_cal.json` (local, gitignored). The 3-click map handles camera rotation, mirroring and unequal scale. Ball colour = click the ball (saved to `hsv.json`, with tolerance sliders). Processing at half resolution (`SCALE = 0.5`). No smoothing in the tracker: the controller filters. Detection limits are live sliders in the Controls window and are saved in `hsv.json`: `Round %` (min circularity, default 40; a half ball is about 75), `Edge x10` (edge limit in plate units x10, default 20 = 2.0; 0 = no limit; drawn as a grey oval), `Min area` (default 25 processed pixels). The cyan oval is radius 1.0 from the 3-click calibration. User wants half-visible balls at the plate rim detected.

Camera decision (confirmed): the user will use ONLY the "USB 2.0 Camera" (Microdia 0c45:62f1), not the C270. Its only controls are brightness, contrast, saturation, hue, gamma, power_line_frequency (50 Hz), sharpness, backlight_compensation. No exposure, gain or white balance control. Formats: YUYV only, 640x480 / 352x288 / 320x240 / 176x144 / 160x120, each advertising 5 to 60 fps. Measured with `v4l2-ctl --stream-mmap` at 640x480: **19.9 fps** (backlight_compensation=0). `vision.py` measured only 10 fps at 640x480, so frames were being lost in the code path. Use `cam_setup_usb2.sh` (not `cam_setup.sh`). `vision.py` now takes `[device] [width] [height] [fps]`, processes at 320 px wide, prints processing time per frame, and rescales an old plate calibration to the new resolution. Bright, steady lighting is the main way to keep the frame rate up.

`level_cal.py`: holds all three servos and nudges each in degrees (a/d = corner up/down 1 deg, u/j = whole plate up/down); prints the `LEVEL_DEG` line for `config.py`.

### Stage 5: State Estimation & Logging (`estimator.py`, `log_ball.py`)
- `estimator.py`: Implements `BallEstimator` using a continuous-discrete kinematic 2D Kalman filter. Filters spatial quantization and camera noise while computing low-noise velocities $(v_x, v_y)$ with dynamic $\Delta t$ and dropped-frame coasting.
- `log_ball.py`: Testbench for Stage 5. Records $(t, x, y, v_x, v_y, \text{speed})$ to CSV.
```bash
bash cam_setup_usb2.sh
python log_ball.py 0 320 240 30
# Or headless:
python log_ball.py 0 320 240 30 --headless --output test_run.csv
```
Roll the ball manually across the plate. Verify in the GUI/log that velocities are smooth and responsive without high-frequency spikes.

### Stages 6 & 7: Feedback Controller (`controller.py`)
- Implements 2D PD/PID feedback: $g_x = -(K_p \cdot e_x + K_i \cdot \int e_x dt + K_d \cdot v_x)$.
- Features: deadband near center, anti-windup conditional integration, vector magnitude saturation, slew-rate limiting, and dynamic setpoint trajectories.
- **Stage 6 (P-only test):** Run with `--p-only` or press `p` in GUI. Low gain ($K_p \approx 0.4 - 0.5$). The ball will oscillate back and forth around the center.
- **Stage 7 (PD tuning):** Restore PD mode (`d` in GUI). Increase $K_d$ ($0.25 - 0.45$) until oscillations are critically damped and the ball settles at the center.

### Stage 8: Closed-Loop Balancing (`balance.py`)
The primary system application. Connects vision, Kalman estimator, PID controller, kinematics, and servos.
```bash
bash cam_setup_usb2.sh
python balance.py 0 320 240 30
```
- **Failsafe:** If the ball rolls off or is lost for $> 0.35$ s, state transitions to `BALL_LOST` and the plate smoothly returns to level.
- **Hotkeys in GUI:**
  - `q`: Safe shutdown (levels plate, stops pulses, cleans up GPIO).
  - `l`: Force level.
  - `p`: Toggle P-only mode (Stage 6).
  - `d`: Restore PD mode (Stage 7).
  - `s`: Cycle setpoint patterns (`CENTER` $\to$ `CIRCLE` $\to$ `STEP_X` $\to$ `STEP_Y`).
  - `+` / `-`: Increase / decrease $K_p$ by 0.05.
  - `]` / `[`: Increase / decrease $K_d$ by 0.05.
  - `r`: Reset controller integrators.
- **Command-line flags:**
  - `--headless`: Run without OpenCV GUI (for SSH sessions).
  - `--log`: Save full run telemetry to CSV.
  - `--p-only`: Start directly in P-only test mode.
  - `--kp <val>`, `--kd <val>`, `--ki <val>`, `--max-tilt <val>`: Gain overrides.
  - `--mock`: Dry-run without Raspberry Pi hardware.

Open questions: exact servo positions around the plate (`ANGLE`), camera mounting (above, fixed to frame?), plate shape and size, ball colour.

---

## 15. Safety checklist (apply to every wiring change)

1. `sudo shutdown now`, wait for the Pi to stop, then unplug its USB-C.
2. Unplug the 5 V adapter.
3. Make or move wires. Use plastic-ended jumpers, push straight down, never touch pins 1, 2 or 4 area with a loose end.
4. Recheck: only pins 9, 11, 13, 15 used, adapter + goes only to servo red wires, no bare metal touching.
5. Plug in the Pi first and let it boot (Fn + F10 if needed). Plug in the adapter last.
6. If a servo buzzes, grinds, heats, or the Pi reboots: unplug the **adapter first**, then investigate.

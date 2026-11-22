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
MIN_SAT = 80         # eliminates white 3D printed body (white has Saturation < 35)
MAX_AREA = 2500      # rejects large chassis / frame blobs

cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
if not cap.isOpened():
    cap = cv2.VideoCapture(dev)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
if not cap.isOpened():
    print(f"Could not open /dev/video{dev}")
    raise SystemExit(1)

center = None            # [h, s, v] sampled from the ball
tol = [12, 60, 60]       # tolerance around the center
min_sat = MIN_SAT
if os.path.exists(FILE):
    try:
        d = json.load(open(FILE))
        center, tol = d["center"], d["tol"]
        min_sat = d.get("min_sat", MIN_SAT)
        print("Loaded", FILE, center, tol, "Min Sat:", min_sat)
    except Exception:
        pass


def save():
    if center is not None:
        json.dump({"center": center, "tol": tol, "min_sat": min_sat}, open(FILE, "w"))


latest_hsv = None


def on_click(event, x, y, flags, param):
    global center, min_sat
    if event == cv2.EVENT_LBUTTONDOWN and latest_hsv is not None:
        patch = latest_hsv[max(y - 5, 0):y + 6, max(x - 5, 0):x + 6].reshape(-1, 3)
        center = [int(v) for v in np.median(patch, axis=0)]
        # Auto-adjust saturation floor for neon ball
        if center[1] > 70:
            min_sat = max(60, min(140, int(center[1] * 0.55)))
            try:
                cv2.setTrackbarPos("Min Sat", "Controls", min_sat)
            except Exception:
                pass
        save()
        print(f"Calibrated on HSV {center} | Min Saturation floor set to {min_sat}")


def make_mask(hsv, c, t, sat_floor=MIN_SAT):
    h, s, v = c
    # Saturation floor strictly excludes white 3D printed body
    slo, shi = max(s - t[1], sat_floor), min(s + t[1], 255)
    vlo, vhi = max(v - t[2], 50), min(v + t[2], 255)
    hlo, hhi = h - t[0], h + t[0]

    def rng(a, b):
        return cv2.inRange(hsv, np.array([a, slo, vlo]), np.array([b, shi, vhi]))

    if hlo < 0:                      # red/orange wraps around 0
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
cv2.createTrackbar("Min Sat", "Controls", min_sat, 220, lambda x: None)

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
    min_sat = cv2.getTrackbarPos("Min Sat", "Controls")

    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (7, 7), 0), cv2.COLOR_BGR2HSV)
    latest_hsv = hsv
    mask = np.zeros(hsv.shape[:2], np.uint8)

    if center is not None:
        mask = make_mask(hsv, center, tol, sat_floor=min_sat)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        best, best_area = None, 0
        for c in contours:
            area = cv2.contourArea(c)
            peri = cv2.arcLength(c, True)
            if area < 80 or area > MAX_AREA or peri == 0:
                continue
            circularity = 4 * np.pi * area / (peri * peri)
            if circularity > 0.50 and area > best_area:
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

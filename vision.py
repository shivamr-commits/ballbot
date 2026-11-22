"""Stage 3: find the ball and report its position in PLATE coordinates.

Plate coordinates: (0, 0) = plate centre, x = +1 at the right edge,
y = +1 at the far edge (away from you). Same axes as plate.py.

Run directly for calibration and testing:
    bash cam_setup.sh
    python vision.py            (or: python vision.py 1   for /dev/video1)

Keys in the "Vision" window:
    p  = calibrate the plate: click CENTRE, then RIGHT edge, then FAR edge
    click (normal mode) = sample the ball colour
    q  = quit
"""
import cv2
import numpy as np
import json
import math
import sys
import time

CAL_FILE = "plate_cal.json"
HSV_FILE = "hsv.json"
W, H = 640, 480
SCALE = 0.5          # process at half resolution for speed
MIN_AREA = 40        # minimum blob area (in half-resolution pixels)
MIN_CIRC = 0.6       # minimum circularity (1.0 = perfect circle)
EDGE_LIMIT = 1.3     # ignore blobs further than this from the centre (plate units).
                     # Raise to about 2.0 for a triangular plate with far corners.


def load(path, default):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return default


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


class BallTracker:
    def __init__(self, dev=0):
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise SystemExit(f"Could not open /dev/video{dev}")

        c = load(HSV_FILE, {})
        self.center = c.get("center")          # ball colour [h, s, v]
        self.tol = c.get("tol", [10, 70, 70])
        self.kernel = np.ones((3, 3), np.uint8)
        self.frame = self.hsv = self.mask = None
        self.set_plate(load(CAL_FILE, None))

    # ---------- plate calibration ----------
    def set_plate(self, plate):
        """plate = {"c": [u, v], "px": [u, v], "py": [u, v]} in full-res pixels."""
        self.plate, self.M, self.inv = None, None, None
        if not plate:
            return False
        c = np.array(plate["c"], float)
        M = np.column_stack([np.array(plate["px"], float) - c,
                             np.array(plate["py"], float) - c])
        if abs(np.linalg.det(M)) < 1e-3:
            return False                       # clicks were in a straight line
        self.plate, self.c, self.M, self.inv = plate, c, M, np.linalg.inv(M)
        return True

    def to_plate(self, u, v):
        x, y = self.inv @ (np.array([u, v], float) - self.c)
        return float(x), float(y)

    def to_pixel(self, x, y):
        u, v = self.c + self.M @ np.array([x, y], float)
        return int(u), int(v)

    # ---------- one frame ----------
    def read(self):
        """Returns None if the camera failed, else a dict:
        t (time), found, and if found: u, v, r (pixels) and x, y (plate units)."""
        ok, frame = self.cap.read()
        t = time.time()
        if not ok:
            return None
        self.frame = frame
        small = cv2.resize(frame, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_AREA)
        self.hsv = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2HSV)
        res = {"t": t, "found": False}

        if self.center is None:
            self.mask = np.zeros(self.hsv.shape[:2], np.uint8)
            return res

        mask = make_mask(self.hsv, self.center, self.tol)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)
        self.mask = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, best_area = None, 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            peri = cv2.arcLength(cnt, True)
            if area < MIN_AREA or peri == 0:
                continue
            if 4 * np.pi * area / (peri * peri) < MIN_CIRC:
                continue
            (u, v), r = cv2.minEnclosingCircle(cnt)
            u, v, r = u / SCALE, v / SCALE, r / SCALE
            if self.inv is not None:
                x, y = self.to_plate(u, v)
                if math.hypot(x, y) > EDGE_LIMIT:
                    continue                    # off the plate: ignore
            if area > best_area:
                best, best_area = (u, v, r), area

        if best is not None:
            u, v, r = best
            res.update(found=True, u=u, v=v, r=r)
            if self.inv is not None:
                res["x"], res["y"] = self.to_plate(u, v)
        return res

    def save_colour(self):
        if self.center is not None:
            with open(HSV_FILE, "w") as f:
                json.dump({"center": self.center, "tol": self.tol}, f)

    def close(self):
        self.cap.release()


# ================= calibration / test window =================
if __name__ == "__main__":
    dev = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    bt = BallTracker(dev)
    STEPS = ["plate CENTRE", "RIGHT edge (+x)", "FAR edge (+y, away from you)"]
    state = {"mode": "ball", "clicks": []}

    def on_click(event, u, v, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or bt.hsv is None:
            return
        if state["mode"] == "plate":
            state["clicks"].append([u, v])
            if len(state["clicks"]) == 3:
                k = state["clicks"]
                if bt.set_plate({"c": k[0], "px": k[1], "py": k[2]}):
                    with open(CAL_FILE, "w") as f:
                        json.dump(bt.plate, f)
                    print("Plate calibrated:", bt.plate)
                else:
                    print("Those clicks were in a line. Press p and try again.")
                state["mode"] = "ball"
        else:
            su, sv = int(u * SCALE), int(v * SCALE)
            patch = bt.hsv[max(sv - 3, 0):sv + 4, max(su - 3, 0):su + 4].reshape(-1, 3)
            bt.center = [int(x) for x in np.median(patch, axis=0)]
            bt.save_colour()
            print("Ball colour HSV:", bt.center)

    cv2.namedWindow("Vision")
    cv2.setMouseCallback("Vision", on_click)
    cv2.namedWindow("Controls")
    cv2.createTrackbar("H tol", "Controls", bt.tol[0], 40, lambda x: None)
    cv2.createTrackbar("S tol", "Controls", bt.tol[1], 127, lambda x: None)
    cv2.createTrackbar("V tol", "Controls", bt.tol[2], 127, lambda x: None)

    fps, prev, last_print, lost = 0.0, time.time(), 0.0, 0
    font = cv2.FONT_HERSHEY_SIMPLEX
    try:
        while True:
            bt.tol = [cv2.getTrackbarPos(n, "Controls") for n in ("H tol", "S tol", "V tol")]
            res = bt.read()
            if res is None:
                print("Failed to read a frame.")
                break
            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - prev, 1e-6)
            prev = now
            f = bt.frame

            if bt.inv is not None:                      # draw the plate frame
                edge = np.array([bt.to_pixel(math.cos(a), math.sin(a))
                                 for a in np.linspace(0, 2 * math.pi, 48)], np.int32)
                cv2.polylines(f, [edge], True, (255, 255, 0), 1)
                c0 = bt.to_pixel(0, 0)
                cv2.arrowedLine(f, c0, bt.to_pixel(0.5, 0), (0, 0, 255), 2)   # +x red
                cv2.arrowedLine(f, c0, bt.to_pixel(0, 0.5), (0, 255, 0), 2)   # +y green
                cv2.circle(f, c0, 4, (255, 255, 255), -1)

            if res["found"]:
                cv2.circle(f, (int(res["u"]), int(res["v"])), int(res["r"]), (0, 255, 0), 2)
                if "x" in res:
                    txt = f"x={res['x']:+.2f} y={res['y']:+.2f}"
                else:
                    txt = "plate not calibrated (press p)"
                cv2.putText(f, txt, (10, H - 15), font, 0.6, (0, 255, 0), 2)
            else:
                lost += 1
                txt = "LOST"
                cv2.putText(f, "LOST", (10, H - 15), font, 0.6, (0, 0, 255), 2)

            if state["mode"] == "plate":
                cv2.putText(f, "Click " + STEPS[len(state["clicks"])], (10, 60),
                            font, 0.7, (0, 255, 255), 2)
            elif bt.center is None:
                cv2.putText(f, "Click on the ball", (10, 60), font, 0.7, (0, 255, 255), 2)

            cv2.putText(f, f"FPS: {fps:.1f}", (10, 30), font, 1, (0, 255, 0), 2)
            cv2.imshow("Vision", f)
            cv2.imshow("Mask", bt.mask)

            if now - last_print > 0.5:                  # terminal readout, 2 per second
                print(f"FPS {fps:4.1f}  {txt}  (lost frames so far: {lost})")
                last_print = now

            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            if k == ord('p'):
                state["mode"], state["clicks"] = "plate", []
                print("Plate calibration: click", STEPS[0])
    finally:
        bt.save_colour()
        bt.close()
        cv2.destroyAllWindows()

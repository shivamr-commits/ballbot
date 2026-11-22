"""Stage 3: find the ball and report its position in PLATE coordinates.

Plate coordinates: (0, 0) = plate centre, x = +1 at the right edge,
y = +1 at the far edge (away from you). Same axes as plate.py.

Run directly for calibration and testing:
    bash cam_setup_usb2.sh
    python vision.py [device] [width] [height] [fps]
    e.g. python vision.py 0 320 240 30

Cyan oval = plate edge from your clicks (distance 1.0). Grey oval = detection
boundary (Edge limit). Sliders in "Controls": H/S/V tol, Round %, Edge x10, Min area.

Keys in the "Vision" window:
    p  = calibrate the plate: click CENTRE, then RIGHT edge, then FAR edge
    click (normal mode) = sample the ball colour
    q  = quit
"""
try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None
import json
import math
import sys
import time

CAL_FILE = "plate_cal.json"
HSV_FILE = "hsv.json"
PROC_W = 320         # frames are processed at this width (smaller = faster)
# Defaults for the three detection settings. All three have live sliders in the
# "Controls" window and are saved to hsv.json with the colour.
MIN_AREA = 25        # minimum blob area (in processed pixels)
MAX_AREA = 1200      # maximum blob area (rejects large chassis/body parts)
MIN_CIRC = 0.40      # minimum roundness (1.0 = full circle, half a ball is about 0.75)
EDGE_LIMIT = 1.8     # ignore blobs further than this from the centre (1.0 = plate edge)
MIN_SAT = 80         # minimum saturation floor: eliminates white/grey/black 3D printed body!


def load(path, default):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return default


def make_mask(hsv, c, t, min_sat=MIN_SAT):
    """Generates a binary mask for the ball.
    min_sat acts as a hard saturation floor, strictly excluding white/grey/black objects
    (which have S < 40) so only vivid neon/saturated colours are detected.
    """
    h, s, v = c
    # Saturation floor: neon colors have S > 100; white 3D prints have S < 30.
    slo = max(s - t[1], min_sat)
    shi = min(s + t[1], 255)
    vlo = max(v - t[2], 50)     # avoid deep black shadows
    vhi = min(v + t[2], 255)
    hlo, hhi = h - t[0], h + t[0]

    def rng(a, b):
        return cv2.inRange(hsv, np.array([a, slo, vlo]), np.array([b, shi, vhi]))

    if hlo < 0:                      # red/orange hue wraps around 0
        return rng(0, hhi) | rng(180 + hlo, 179)
    if hhi > 179:
        return rng(hlo, 179) | rng(0, hhi - 180)
    return rng(hlo, hhi)


class BallTracker:
    def __init__(self, dev=0, w=640, h=480, fps=30):
        if cv2 is None or np is None:
            raise RuntimeError("OpenCV (cv2) or NumPy is not installed. Run inside the Raspberry Pi virtual environment or install python3-opencv.")

        # Try V4L2 backend first (standard on Linux/Raspberry Pi), fall back to default
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(dev)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise SystemExit(f"Could not open /dev/video{dev}")
        self.w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or w   # what the camera really gave
        self.h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or h
        self.scale = min(1.0, PROC_W / self.w)
        self.proc_ms = 0.0
        print(f"Camera: {self.w}x{self.h}, requested {fps} fps, processing at "
              f"{int(self.w * self.scale)}x{int(self.h * self.scale)}")

        c = load(HSV_FILE, {})
        self.center = c.get("center")          # ball colour [h, s, v]
        self.tol = c.get("tol", [12, 60, 60])
        self.min_sat = c.get("min_sat", MIN_SAT)
        self.min_area = c.get("min_area", MIN_AREA)
        self.max_area = c.get("max_area", MAX_AREA)
        self.min_circ = c.get("min_circ", MIN_CIRC)
        self.edge_limit = c.get("edge_limit", EDGE_LIMIT)
        self.kernel = np.ones((3, 3), np.uint8)
        self.frame = self.hsv = self.mask = None
        self.debug = ""
        self.set_plate(load(CAL_FILE, None))

    # ---------- plate calibration ----------
    def set_plate(self, plate):
        """plate = {"c", "px", "py": [u, v] in camera pixels, "size": [w, h]}."""
        self.plate, self.M, self.inv = None, None, None
        if not plate:
            return False
        sw, sh = plate.get("size", [640, 480])
        if [sw, sh] != [self.w, self.h]:           # calibrated at another resolution: rescale
            fx, fy = self.w / sw, self.h / sh
            plate = {k: [plate[k][0] * fx, plate[k][1] * fy] for k in ("c", "px", "py")}
            plate["size"] = [self.w, self.h]
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
        S = self.scale
        small = frame if S == 1.0 else cv2.resize(frame, None, fx=S, fy=S,
                                                  interpolation=cv2.INTER_AREA)
        self.hsv = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2HSV)
        res = {"t": t, "found": False}

        if self.center is None:
            self.mask = np.zeros(self.hsv.shape[:2], np.uint8)
            self.debug = "no ball colour yet: click on the ball"
            return res

        mask = make_mask(self.hsv, self.center, self.tol, min_sat=self.min_sat)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)
        self.mask = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, best_area = None, 0
        biggest = None                          # for the debug message
        for cnt in contours:
            area = cv2.contourArea(cnt)
            peri = cv2.arcLength(cnt, True)
            circ = 4 * np.pi * area / (peri * peri) if peri > 0 else 0
            (u, v), r = cv2.minEnclosingCircle(cnt)
            u, v, r = u / S, v / S, r / S
            dist = math.hypot(*self.to_plate(u, v)) if self.inv is not None else 0
            if area < self.min_area:
                why = "too small (lower 'Min area')"
            elif area > self.max_area:
                why = "too large (chassis/body, raise 'Max area' if ball)"
            elif circ < self.min_circ:
                why = "not round enough (lower 'Round %')"
            elif self.edge_limit > 0 and dist > self.edge_limit:
                why = "outside edge limit (raise 'Edge x10')"
            else:
                why = "OK"
            if biggest is None or area > biggest[0]:
                biggest = (area, circ, dist, why)
            if why == "OK" and area > best_area:
                best, best_area = (u, v, r), area

        if biggest is None:
            self.debug = "mask empty: colour not matching (click ball again / widen S,V sliders) or ball tiny in image"
        else:
            a, c, d, w = biggest
            self.debug = f"biggest blob: area={a:.0f} round={c:.2f} dist={d:.2f} -> {w}"

        if best is not None:
            u, v, r = best
            res.update(found=True, u=u, v=v, r=r)
            if self.inv is not None:
                res["x"], res["y"] = self.to_plate(u, v)
        self.proc_ms = (time.time() - t) * 1000
        return res

    def save_colour(self):
        with open(HSV_FILE, "w") as f:
            json.dump({"center": self.center, "tol": self.tol,
                       "min_area": self.min_area, "max_area": self.max_area,
                       "min_circ": self.min_circ, "edge_limit": self.edge_limit,
                       "min_sat": self.min_sat}, f)

    def close(self):
        self.cap.release()


# ================= calibration / test window =================
if __name__ == "__main__":
    a = sys.argv[1:]
    bt = BallTracker(int(a[0]) if len(a) > 0 else 0,
                     int(a[1]) if len(a) > 1 else 640,
                     int(a[2]) if len(a) > 2 else 480,
                     int(a[3]) if len(a) > 3 else 30)
    H = bt.h
    STEPS = ["plate CENTRE", "RIGHT edge (+x)", "FAR edge (+y, away from you)"]
    state = {"mode": "ball", "clicks": []}

    def on_click(event, u, v, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or bt.hsv is None:
            return
        if state["mode"] == "plate":
            state["clicks"].append([u, v])
            if len(state["clicks"]) == 3:
                k = state["clicks"]
                if bt.set_plate({"c": k[0], "px": k[1], "py": k[2], "size": [bt.w, bt.h]}):
                    with open(CAL_FILE, "w") as f:
                        json.dump(bt.plate, f)
                    print("Plate calibrated:", bt.plate)
                else:
                    print("Those clicks were in a line. Press p and try again.")
                state["mode"] = "ball"
        else:
            su, sv = int(u * bt.scale), int(v * bt.scale)
            patch = bt.hsv[max(sv - 3, 0):sv + 4, max(su - 3, 0):su + 4].reshape(-1, 3)
            bt.center = [int(x) for x in np.median(patch, axis=0)]
            sampled_sat = bt.center[1]
            # If ball is saturated (neon), set Min Sat well above white body (S<35)
            if sampled_sat > 70:
                bt.min_sat = max(60, min(140, int(sampled_sat * 0.55)))
                try:
                    cv2.setTrackbarPos("Min Sat", "Controls", bt.min_sat)
                except Exception:
                    pass
            bt.save_colour()
            print(f"Ball colour HSV: {bt.center} | Min Saturation floor set to: {bt.min_sat} (white body excluded)")

    cv2.namedWindow("Vision")
    cv2.setMouseCallback("Vision", on_click)
    cv2.namedWindow("Controls")
    cv2.createTrackbar("H tol", "Controls", bt.tol[0], 40, lambda x: None)
    cv2.createTrackbar("S tol", "Controls", bt.tol[1], 127, lambda x: None)
    cv2.createTrackbar("V tol", "Controls", bt.tol[2], 127, lambda x: None)
    cv2.createTrackbar("Min Sat", "Controls", int(bt.min_sat), 220, lambda x: None)
    cv2.createTrackbar("Round %", "Controls", int(bt.min_circ * 100), 100, lambda x: None)
    cv2.createTrackbar("Edge x10", "Controls", int(bt.edge_limit * 10), 40, lambda x: None)
    cv2.createTrackbar("Min area", "Controls", int(bt.min_area), 300, lambda x: None)
    cv2.createTrackbar("Max area", "Controls", int(bt.max_area), 4000, lambda x: None)

    fps, prev, last_print, lost = 0.0, time.time(), 0.0, 0
    font = cv2.FONT_HERSHEY_SIMPLEX
    try:
        while True:
            bt.tol = [cv2.getTrackbarPos(n, "Controls") for n in ("H tol", "S tol", "V tol")]
            bt.min_sat = cv2.getTrackbarPos("Min Sat", "Controls")
            bt.min_circ = cv2.getTrackbarPos("Round %", "Controls") / 100
            bt.edge_limit = cv2.getTrackbarPos("Edge x10", "Controls") / 10
            bt.min_area = cv2.getTrackbarPos("Min area", "Controls")
            bt.max_area = max(bt.min_area + 50, cv2.getTrackbarPos("Max area", "Controls"))
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
                cv2.polylines(f, [edge], True, (255, 255, 0), 1)           # 1.0 = plate edge
                if bt.edge_limit > 0:                                        # detection boundary
                    lim = np.array([bt.to_pixel(bt.edge_limit * math.cos(a),
                                                bt.edge_limit * math.sin(a))
                                    for a in np.linspace(0, 2 * math.pi, 48)], np.int32)
                    cv2.polylines(f, [lim], True, (128, 128, 128), 1)
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
                print(f"FPS {fps:4.1f}  proc {bt.proc_ms:4.1f} ms  {txt:28s} | {bt.debug}")
                last_print = now

            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            if k == ord('p'):
                state["mode"], state["clicks"] = "plate", []
                print("Plate calibration: click", STEPS[0])
            elif k == ord('1'):
                # Preset 1: Neon Green
                bt.center = [55, 200, 200]
                bt.tol = [18, 65, 65]
                bt.min_sat = 85
                cv2.setTrackbarPos("H tol", "Controls", bt.tol[0])
                cv2.setTrackbarPos("Min Sat", "Controls", bt.min_sat)
                print("Applied Preset: NEON GREEN (HSV [55, 200, 200], Min Sat 85)")
            elif k == ord('2'):
                # Preset 2: Neon Yellow
                bt.center = [30, 210, 220]
                bt.tol = [12, 55, 55]
                bt.min_sat = 90
                cv2.setTrackbarPos("H tol", "Controls", bt.tol[0])
                cv2.setTrackbarPos("Min Sat", "Controls", bt.min_sat)
                print("Applied Preset: NEON YELLOW (HSV [30, 210, 220], Min Sat 90)")
            elif k == ord('3'):
                # Preset 3: Neon Orange
                bt.center = [12, 215, 220]
                bt.tol = [10, 50, 50]
                bt.min_sat = 95
                cv2.setTrackbarPos("H tol", "Controls", bt.tol[0])
                cv2.setTrackbarPos("Min Sat", "Controls", bt.min_sat)
                print("Applied Preset: NEON ORANGE (HSV [12, 215, 220], Min Sat 95)")
            elif k == ord('4'):
                # Preset 4: Neon Pink
                bt.center = [170, 190, 210]
                bt.tol = [14, 60, 60]
                bt.min_sat = 80
                cv2.setTrackbarPos("H tol", "Controls", bt.tol[0])
                cv2.setTrackbarPos("Min Sat", "Controls", bt.min_sat)
                print("Applied Preset: NEON PINK (HSV [170, 190, 210], Min Sat 80)")
    finally:
        bt.save_colour()
        bt.close()
        cv2.destroyAllWindows()

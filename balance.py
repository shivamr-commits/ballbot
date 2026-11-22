"""Stage 8: Main Closed-Loop Ball Balancing System.

Integrates real-time vision, Kalman state estimation, 2D PID control,
plate kinematics mixing, and servo actuation.

Usage:
    bash cam_setup_usb2.sh
    python balance.py [device] [width] [height] [fps] [options]
    e.g. python balance.py 0 320 240 30

Options:
    --headless         Run without graphical OpenCV window (ideal for SSH)
    --p-only           Stage 6 mode: run proportional-only control (Kd=0, Ki=0)
    --kp <val>         Override Kp gain (default in config.py)
    --kd <val>         Override Kd gain (default in config.py)
    --ki <val>         Override Ki gain (default in config.py)
    --max-tilt <val>   Override max tilt magnitude in [0, 1] (default in config.py)
    --log              Record telemetry to timestamped CSV
    --mock             Run with simulated servo output (for off-Pi testing)

Interactive Keys in GUI:
    q                  Quit safely (levels plate and shuts down)
    l                  Force plate level
    p                  Toggle P-only mode (Stage 6 test)
    d                  Restore PD mode (Stage 7 test)
    s                  Cycle setpoint pattern (Center -> Circle -> Step X -> Step Y)
    + / -              Increase / decrease Kp gain (+-0.05)
    ] / [              Increase / decrease Kd gain (+-0.05)
    r                  Reset controller integrators and estimator
"""
import argparse
import csv
from datetime import datetime
import math
import os
import signal
import sys
import time

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

import config as C
from controller import BallController
from estimator import BallEstimator
from plate import tilt_to_angles
from servos import Servos
from vision import BallTracker


class MockBallTracker:
    """Simulated ball physics tracker for testing off-Pi without a camera."""
    def __init__(self, w=640, h=480, fps=30):
        self.w, self.h, self.fps = w, h, fps
        self.frame = None
        self.mask = None
        self.inv = True
        self.proc_ms = 0.5
        self.x = 0.45
        self.y = -0.30
        self.vx = 0.0
        self.vy = 0.0
        self.gx = 0.0
        self.gy = 0.0
        self.last_t = time.time()

    def set_tilt(self, gx, gy):
        self.gx, self.gy = gx, gy

    def read(self):
        t = time.time()
        dt = max(0.001, min(0.1, t - self.last_t))
        self.last_t = t
        # Ball acceleration proportional to tilt (gx > 0 rolls +x)
        self.vx += self.gx * 4.0 * dt - 0.2 * self.vx * dt
        self.vy += self.gy * 4.0 * dt - 0.2 * self.vy * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        dist = math.hypot(self.x, self.y)
        if dist > 0.95:  # rim bounce
            self.vx *= -0.4
            self.vy *= -0.4
            self.x = (self.x / dist) * 0.92
            self.y = (self.y / dist) * 0.92
        time.sleep(1.0 / self.fps)
        return {"t": t, "found": True, "x": self.x, "y": self.y, "u": 320, "v": 240, "r": 15}

    def to_pixel(self, x, y):
        return int(320 + x * 200), int(240 - y * 200)

    def close(self):
        pass


def parse_args():
    parser = argparse.ArgumentParser(description="Ballbot: 3-DOF Ball Balancing Robot")
    parser.add_argument("device", nargs="?", type=int, default=getattr(C, "CAM_DEV", 0),
                        help="Camera device index (default: 0)")
    parser.add_argument("width", nargs="?", type=int, default=getattr(C, "CAM_WIDTH", 640),
                        help="Camera frame width (default: 640)")
    parser.add_argument("height", nargs="?", type=int, default=getattr(C, "CAM_HEIGHT", 480),
                        help="Camera frame height (default: 480)")
    parser.add_argument("fps", nargs="?", type=int, default=getattr(C, "CAM_FPS", 30),
                        help="Target camera FPS (default: 30)")
    parser.add_argument("--kp", type=float, default=None, help="Proportional gain")
    parser.add_argument("--kd", type=float, default=None, help="Derivative gain")
    parser.add_argument("--ki", type=float, default=None, help="Integral gain")
    parser.add_argument("--max-tilt", type=float, default=None, help="Max tilt magnitude [0, 1]")
    parser.add_argument("--p-only", action="store_true", help="Start in P-only mode (Kd=0, Ki=0)")
    parser.add_argument("--headless", action="store_true", help="Run without OpenCV GUI window")
    parser.add_argument("--log", action="store_true", help="Log full run telemetry to CSV")
    parser.add_argument("--mock", action="store_true", help="Run with simulated servos")
    return parser.parse_args()


def main():
    args = parse_args()

    # Determine headless mode
    headless = args.headless or (os.name != "nt" and "DISPLAY" not in os.environ and sys.platform != "darwin")

    print("======================================================")
    print("           BALLBOT: BALL BALANCING ROBOT             ")
    print("======================================================")

    # 1. Initialize Servos
    print("Initializing servos...")
    servos = Servos(mock=args.mock)
    print("Holding plate at LEVEL position...")
    servos.level()
    time.sleep(0.5)

    # 2. Initialize Camera & Vision
    if args.mock or cv2 is None:
        print("[MOCK] Running with simulated ball dynamics tracker...")
        bt = MockBallTracker(w=args.width, h=args.height, fps=args.fps)
    else:
        print(f"Initializing camera /dev/video{args.device} ({args.width}x{args.height} @ {args.fps} fps)...")
        bt = BallTracker(dev=args.device, w=args.width, h=args.height, fps=args.fps)

    # 3. Initialize Estimator
    estimator = BallEstimator(
        q=getattr(C, "EST_PROCESS_NOISE", 6.0),
        r=getattr(C, "EST_MEAS_NOISE", 0.0001),
        lost_timeout=getattr(C, "EST_LOST_TIMEOUT", 0.35)
    )

    # 4. Initialize Controller
    controller = BallController(
        kp=args.kp,
        kd=args.kd,
        ki=args.ki,
        max_tilt=args.max_tilt
    )
    if args.p_only:
        controller.set_p_only()
        print("Starting in P-ONLY mode (Stage 6 testing).")

    # 5. Optional CSV Logging
    csv_file = None
    csv_writer = None
    if args.log:
        now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f"balance_log_{now_str}.csv"
        csv_file = open(csv_filename, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            "timestamp", "elapsed_s", "dt",
            "found", "tracking",
            "x", "y", "vx", "vy",
            "sp_x", "sp_y",
            "gx", "gy",
            "s1_deg", "s2_deg", "s3_deg"
        ])
        print(f"Telemetry logging enabled: {csv_filename}")

    # Set up graceful exit signal handler
    shutdown_requested = [False]

    def sig_handler(sig, frame):
        shutdown_requested[0] = True

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    if not headless and cv2 is not None:
        cv2.namedWindow("Ballbot Control")
        print("GUI active. Hotkeys: [q] Quit  [l] Level  [p] P-only  [d] PD  [s] Setpoint  [+/-] Kp  []/[ Kd")
    else:
        print("Running in HEADLESS mode. Press Ctrl+C to stop.")

    t_start = time.time()
    t_prev = t_start
    last_print = t_start
    total_frames = 0
    balanced_frames = 0
    fps_smooth = float(args.fps)
    history = []
    current_angles = list(C.LEVEL_DEG)

    try:
        while not shutdown_requested[0]:
            t_now = time.time()
            dt = max(0.001, min(0.15, t_now - t_prev))
            t_prev = t_now
            elapsed = t_now - t_start

            # Read frame from camera
            res = bt.read()
            if res is None:
                print("Camera failed to deliver a frame.")
                break

            total_frames += 1
            fps_smooth = 0.92 * fps_smooth + 0.08 * (1.0 / dt)

            raw_found = res["found"]
            raw_x = res.get("x")
            raw_y = res.get("y")

            # Update Kalman state estimator
            state = estimator.update(res["t"], raw_found, raw_x, raw_y)

            # Determine robot operational state
            if state.tracking:
                robot_state = "BALANCING"
                balanced_frames += 1
                (gx, gy), (sp_x, sp_y), debug = controller.compute(
                    dt, state.x, state.y, state.vx, state.vy, ball_tracked=True, t=t_now
                )
            else:
                robot_state = "BALL_LOST"
                (gx, gy), (sp_x, sp_y), debug = controller.compute(
                    dt, state.x, state.y, state.vx, state.vy, ball_tracked=False, t=t_now
                )

            # Compute servo target angles via kinematics
            target_angles = tilt_to_angles(gx, gy)

            # Slew rate limit servo angles to prevent mechanical jerk
            # Max rate: ~150 deg/sec
            max_ang_delta = 150.0 * dt
            for i in range(3):
                dang = target_angles[i] - current_angles[i]
                current_angles[i] += max(-max_ang_delta, min(max_ang_delta, dang))

            # Send angles to servos
            servos.write(current_angles)
            if hasattr(bt, "set_tilt"):
                bt.set_tilt(gx, gy)

            # Telemetry logging
            if csv_writer is not None:
                csv_writer.writerow([
                    f"{t_now:.4f}", f"{elapsed:.4f}", f"{dt:.4f}",
                    1 if state.found else 0, 1 if state.tracking else 0,
                    f"{state.x:.4f}", f"{state.y:.4f}", f"{state.vx:.4f}", f"{state.vy:.4f}",
                    f"{sp_x:.4f}", f"{sp_y:.4f}",
                    f"{gx:.4f}", f"{gy:.4f}",
                    f"{current_angles[0]:.1f}", f"{current_angles[1]:.1f}", f"{current_angles[2]:.1f}"
                ])
                if total_frames % 25 == 0:
                    csv_file.flush()

            # Console telemetry output (3 times per second)
            if t_now - last_print >= 0.30:
                last_print = t_now
                kp_val = controller.pid_x.kp
                kd_val = controller.pid_x.kd
                pos_str = f"({state.x:+.2f}, {state.y:+.2f})" if state.tracking else "( ----, ---- )"
                tilt_str = f"({gx:+.2f}, {gy:+.2f})"
                ang_str = f"[{current_angles[0]:.0f}, {current_angles[1]:.0f}, {current_angles[2]:.0f}]"
                print(f"[{robot_state:9s}] FPS:{fps_smooth:4.1f} | Pos:{pos_str} | Tilt:{tilt_str} | "
                      f"Deg:{ang_str} | Kp:{kp_val:.2f} Kd:{kd_val:.2f} | Mode:{controller.mode}")

            # GUI Rendering
            if not headless and cv2 is not None:
                f = bt.frame.copy() if bt.frame is not None else np.zeros((args.height, args.width, 3), np.uint8)

                if bt.inv is not None:
                    # 1. Draw plate perimeter oval (radius = 1.0)
                    edge_pts = np.array([bt.to_pixel(math.cos(a), math.sin(a))
                                         for a in np.linspace(0, 2 * math.pi, 48)], np.int32)
                    cv2.polylines(f, [edge_pts], True, (255, 255, 0), 1)

                    # 2. Draw plate center and axes (+x red, +y green)
                    c0 = bt.to_pixel(0, 0)
                    cv2.arrowedLine(f, c0, bt.to_pixel(0.4, 0), (0, 0, 255), 2)
                    cv2.arrowedLine(f, c0, bt.to_pixel(0, 0.4), (0, 255, 0), 2)
                    cv2.circle(f, c0, 4, (255, 255, 255), -1)

                    # 3. Draw target setpoint (crosshair)
                    spt = bt.to_pixel(sp_x, sp_y)
                    cv2.drawMarker(f, spt, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)

                    # 4. Draw commanded tilt vector (yellow arrow from center)
                    tilt_pt = bt.to_pixel(gx * 0.7, gy * 0.7)
                    cv2.arrowedLine(f, c0, tilt_pt, (0, 255, 255), 2, tipLength=0.2)

                    # 5. Trajectory trail
                    if state.tracking:
                        bp = bt.to_pixel(state.x, state.y)
                        history.append(bp)
                        if len(history) > 30:
                            history.pop(0)

                    for i in range(1, len(history)):
                        alpha = i / len(history)
                        col = (int(50 + 200 * alpha), int(220 * alpha), 50)
                        cv2.line(f, history[i - 1], history[i], col, 2)

                    # 6. Draw tracked ball and velocity vector
                    if state.tracking:
                        bp = bt.to_pixel(state.x, state.y)
                        vel_pt = bt.to_pixel(state.x + state.vx * 0.25, state.y + state.vy * 0.25)
                        cv2.arrowedLine(f, bp, vel_pt, (255, 0, 255), 3, tipLength=0.25)
                        cv2.circle(f, bp, 7, (0, 255, 0), -1)
                        cv2.circle(f, bp, 11, (255, 255, 255), 2)

                # HUD Overlay
                font = cv2.FONT_HERSHEY_SIMPLEX
                status_col = (0, 255, 0) if state.tracking else (0, 0, 255)
                cv2.putText(f, f"STATE: {robot_state}", (10, 25), font, 0.7, status_col, 2)
                cv2.putText(f, f"FPS: {fps_smooth:.1f}  Mode: {controller.mode}", (10, 50), font, 0.55, (255, 255, 255), 1)
                cv2.putText(f, f"Kp: {controller.pid_x.kp:.2f}  Kd: {controller.pid_x.kd:.2f}  Ki: {controller.pid_x.ki:.2f}",
                            (10, 72), font, 0.55, (0, 255, 255), 1)

                cv2.putText(f, f"Pos: ({state.x:+.2f}, {state.y:+.2f})", (10, args.height - 35),
                            font, 0.55, (255, 255, 255), 1)
                cv2.putText(f, f"Tilt: ({gx:+.2f}, {gy:+.2f})  Servos: [{current_angles[0]:.0f}, {current_angles[1]:.0f}, {current_angles[2]:.0f}]",
                            (10, args.height - 15), font, 0.55, (0, 255, 0), 2)

                cv2.imshow("Ballbot Control", f)

                # Process UI Hotkeys
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
                elif k == ord("l"):
                    print("Force leveling plate...")
                    controller.reset()
                elif k == ord("p"):
                    controller.set_p_only()
                    print(f"P-ONLY mode enabled (Kp={controller.pid_x.kp:.2f}, Kd=0, Ki=0)")
                elif k == ord("d"):
                    controller.restore_pd()
                    print(f"PD mode restored (Kp={controller.pid_x.kp:.2f}, Kd={controller.pid_x.kd:.2f})")
                elif k == ord("s"):
                    mode = controller.cycle_mode()
                    print(f"Setpoint mode switched to: {mode}")
                elif k in (ord("+"), ord("=")):
                    controller.set_gains(kp=controller.pid_x.kp + 0.05)
                    print(f"Kp increased to: {controller.pid_x.kp:.2f}")
                elif k in (ord("-"), ord("_")):
                    controller.set_gains(kp=max(0.0, controller.pid_x.kp - 0.05))
                    print(f"Kp decreased to: {controller.pid_x.kp:.2f}")
                elif k == ord("]"):
                    controller.set_gains(kd=controller.pid_x.kd + 0.05)
                    print(f"Kd increased to: {controller.pid_x.kd:.2f}")
                elif k == ord("["):
                    controller.set_gains(kd=max(0.0, controller.pid_x.kd - 0.05))
                    print(f"Kd decreased to: {controller.pid_x.kd:.2f}")
                elif k == ord("r"):
                    controller.reset()
                    estimator.reset()
                    print("Controller and estimator reset.")
    finally:
        print("\nShutting down Ballbot safely...")
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()

        # Safely return plate to level position and stop pulses
        try:
            print("Leveling plate...")
            servos.level()
            time.sleep(0.4)
            servos.close()
            print("Servos stopped.")
        except Exception as e:
            print(f"Error stopping servos: {e}")

        # Release camera
        try:
            bt.close()
        except Exception:
            pass

        if not headless and cv2 is not None:
            cv2.destroyAllWindows()

        bal_pct = (100.0 * balanced_frames / max(1, total_frames))
        total_time = time.time() - t_start
        print("================ Run Summary ================")
        print(f"Total time       : {total_time:.2f} s")
        print(f"Total frames     : {total_frames}")
        print(f"Balanced frames  : {balanced_frames} ({bal_pct:.1f}%)")
        print("Done. Goodbye!")


if __name__ == "__main__":
    main()

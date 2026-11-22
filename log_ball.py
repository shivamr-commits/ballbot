"""Stage 5: Test and log ball position and velocity to CSV.

Verifies that the state estimator produces smooth, low-noise velocity estimates
without jitter or excessive phase lag before running closed-loop control.

Usage:
    bash cam_setup_usb2.sh
    python log_ball.py [device] [width] [height] [fps] [--headless] [--output filename.csv]
    e.g. python log_ball.py 0 320 240 30

Press 'q' in the window or Ctrl+C in terminal to stop logging and save.
"""
import argparse
import csv
from datetime import datetime
import math
import os
import sys
import time

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from estimator import BallEstimator
from vision import BallTracker
import config as C


def parse_args():
    parser = argparse.ArgumentParser(description="Log ball tracking and estimated velocity to CSV.")
    parser.add_argument("device", nargs="?", type=int, default=getattr(C, "CAM_DEV", 0),
                        help="Camera device index (default: 0)")
    parser.add_argument("width", nargs="?", type=int, default=getattr(C, "CAM_WIDTH", 640),
                        help="Capture width (default: 640)")
    parser.add_argument("height", nargs="?", type=int, default=getattr(C, "CAM_HEIGHT", 480),
                        help="Capture height (default: 480)")
    parser.add_argument("fps", nargs="?", type=int, default=getattr(C, "CAM_FPS", 30),
                        help="Requested capture FPS (default: 30)")
    parser.add_argument("--output", "-o", type=str, default=None,
                        help="Custom CSV filename (default: ball_log_<timestamp>.csv)")
    parser.add_argument("--duration", "-d", type=float, default=0.0,
                        help="Auto-stop after N seconds (0 = run until stopped)")
    parser.add_argument("--headless", action="store_true",
                        help="Disable OpenCV GUI window (useful for SSH sessions)")
    parser.add_argument("--mock", action="store_true",
                        help="Run with simulated ball trajectory (for off-Pi testing)")
    return parser.parse_args()


def main():
    args = parse_args()

    # Determine if headless mode should be forced (e.g. no DISPLAY in Linux)
    headless = args.headless or (os.name != "nt" and "DISPLAY" not in os.environ and sys.platform != "darwin")

    # Generate CSV filename if not specified
    if args.output:
        csv_filename = args.output
    else:
        now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f"ball_log_{now_str}.csv"

    if args.mock or cv2 is None:
        from balance import MockBallTracker
        print("[MOCK] Initializing simulated ball dynamics tracker...")
        bt = MockBallTracker(w=args.width, h=args.height, fps=args.fps)
    else:
        print(f"Initializing camera /dev/video{args.device} ({args.width}x{args.height} @ {args.fps} fps)...")
        bt = BallTracker(dev=args.device, w=args.width, h=args.height, fps=args.fps)
    estimator = BallEstimator(
        q=getattr(C, "EST_PROCESS_NOISE", 6.0),
        r=getattr(C, "EST_MEAS_NOISE", 0.0001),
        lost_timeout=getattr(C, "EST_LOST_TIMEOUT", 0.35)
    )

    csv_file = open(csv_filename, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "timestamp", "elapsed_s", "dt_s",
        "found", "tracking",
        "raw_u", "raw_v",
        "raw_x", "raw_y",
        "filt_x", "filt_y",
        "vx", "vy", "speed",
        "proc_ms"
    ])
    csv_file.flush()

    print(f"Logging telemetry to: {csv_filename}")
    if headless:
        print("Running in HEADLESS mode (console only). Press Ctrl+C to stop.")
    else:
        print("GUI active. Press 'q' in the window or Ctrl+C in terminal to stop.")
        cv2.namedWindow("Ball Telemetry")

    t_start = time.time()
    t_prev = t_start
    last_print = t_start
    total_frames = 0
    tracked_frames = 0
    max_speed = 0.0
    fps_smooth = 0.0
    history = []   # list of (x, y) points for visualization trail

    try:
        while True:
            t_now = time.time()
            dt = max(1e-4, t_now - t_prev)
            t_prev = t_now
            elapsed = t_now - t_start

            if args.duration > 0 and elapsed >= args.duration:
                print(f"Target duration {args.duration:.1f}s reached.")
                break

            res = bt.read()
            if res is None:
                print("Failed to read frame from camera.")
                break

            total_frames += 1
            fps_smooth = 0.9 * fps_smooth + 0.1 * (1.0 / dt)

            raw_found = res["found"]
            raw_x = res.get("x")
            raw_y = res.get("y")
            raw_u = res.get("u")
            raw_v = res.get("v")

            # Update Kalman state estimator
            state = estimator.update(res["t"], raw_found, raw_x, raw_y)

            if state.tracking:
                tracked_frames += 1
                if state.speed > max_speed:
                    max_speed = state.speed

            # Write data row to CSV
            writer.writerow([
                f"{state.t:.4f}",
                f"{elapsed:.4f}",
                f"{dt:.4f}",
                1 if state.found else 0,
                1 if state.tracking else 0,
                f"{raw_u:.1f}" if raw_u is not None else "",
                f"{raw_v:.1f}" if raw_v is not None else "",
                f"{raw_x:.4f}" if raw_x is not None else "",
                f"{raw_y:.4f}" if raw_y is not None else "",
                f"{state.x:.4f}",
                f"{state.y:.4f}",
                f"{state.vx:.4f}",
                f"{state.vy:.4f}",
                f"{state.speed:.4f}",
                f"{bt.proc_ms:.2f}"
            ])

            if total_frames % 20 == 0:
                csv_file.flush()

            # Console telemetry (3 times per second)
            if t_now - last_print >= 0.33:
                last_print = t_now
                status_str = "TRACKING" if state.tracking else ("DETECTED" if state.found else "LOST")
                raw_str = f"({raw_x:+.2f}, {raw_y:+.2f})" if raw_x is not None else "( ----, ---- )"
                filt_str = f"({state.x:+.2f}, {state.y:+.2f})"
                vel_str = f"({state.vx:+.2f}, {state.vy:+.2f})"
                print(f"[{elapsed:05.1f}s] FPS:{fps_smooth:4.1f} | {status_str:8s} | "
                      f"Raw:{raw_str} -> Filt:{filt_str} | Vel:{vel_str} | Spd:{state.speed:.2f}")

            # GUI Visualization
            if not headless and cv2 is not None:
                f = bt.frame.copy() if bt.frame is not None else np.zeros((args.height, args.width, 3), np.uint8)

                # Draw plate outline if calibrated
                if bt.inv is not None:
                    edge_pts = np.array([bt.to_pixel(math.cos(a), math.sin(a))
                                         for a in np.linspace(0, 2 * math.pi, 48)], np.int32)
                    cv2.polylines(f, [edge_pts], True, (255, 255, 0), 1)
                    c0 = bt.to_pixel(0, 0)
                    cv2.arrowedLine(f, c0, bt.to_pixel(0.4, 0), (0, 0, 255), 2)   # +x red
                    cv2.arrowedLine(f, c0, bt.to_pixel(0, 0.4), (0, 255, 0), 2)   # +y green
                    cv2.circle(f, c0, 4, (255, 255, 255), -1)

                # Maintain trajectory history trail
                if state.tracking and bt.inv is not None:
                    px, py = bt.to_pixel(state.x, state.y)
                    history.append((px, py))
                    if len(history) > 40:
                        history.pop(0)

                # Draw trajectory trail
                for i in range(1, len(history)):
                    alpha = i / len(history)
                    color = (int(50 + 200 * alpha), int(200 * alpha), int(50 * alpha))
                    cv2.line(f, history[i - 1], history[i], color, 2)

                # Draw current ball and velocity vector
                if state.tracking and bt.inv is not None:
                    bp = bt.to_pixel(state.x, state.y)
                    # Velocity vector arrow (length scaled for visibility)
                    vel_scale = 0.25  # seconds forward
                    vp = bt.to_pixel(state.x + state.vx * vel_scale, state.y + state.vy * vel_scale)
                    cv2.arrowedLine(f, bp, vp, (0, 255, 255), 3, tipLength=0.25)
                    # Filtered position circle
                    cv2.circle(f, bp, 6, (0, 255, 0), -1)
                    cv2.circle(f, bp, 10, (255, 255, 255), 2)

                # Text HUD
                hud_font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(f, f"FPS: {fps_smooth:.1f} | Elapsed: {elapsed:.1f}s", (10, 25),
                            hud_font, 0.6, (0, 255, 0), 2)
                status_color = (0, 255, 0) if state.tracking else (0, 0, 255)
                status_txt = "TRACKING" if state.tracking else "BALL LOST"
                cv2.putText(f, status_txt, (10, 50), hud_font, 0.7, status_color, 2)
                cv2.putText(f, f"Pos: ({state.x:+.2f}, {state.y:+.2f})", (10, args.height - 40),
                            hud_font, 0.55, (255, 255, 255), 1)
                cv2.putText(f, f"Vel: ({state.vx:+.2f}, {state.vy:+.2f}) | Spd: {state.speed:.2f}",
                            (10, args.height - 15), hud_font, 0.55, (0, 255, 255), 2)

                cv2.imshow("Ball Telemetry", f)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
    except KeyboardInterrupt:
        print("\nLogging stopped by user (Ctrl+C).")
    finally:
        csv_file.flush()
        csv_file.close()
        bt.close()
        if not headless and cv2 is not None:
            cv2.destroyAllWindows()

        pct = (100.0 * tracked_frames / max(1, total_frames))
        print("\n================ Logging Summary ================")
        print(f"Saved file      : {csv_filename}")
        print(f"Total frames    : {total_frames}")
        print(f"Tracked frames  : {tracked_frames} ({pct:.1f}%)")
        print(f"Max ball speed  : {max_speed:.3f} plate units/s")
        print(f"Total duration  : {time.time() - t_start:.2f} s")
        print("=================================================")


if __name__ == "__main__":
    main()

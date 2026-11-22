"""Stages 6 & 7: 2D PD/PID Feedback Controller for the Ball Balancing Robot.

Calculates the required plate tilt command (gx, gy) in [-1.0, 1.0] from estimated
ball position and velocity.

Sign convention:
    gx > 0 tilts plate down on the right -> ball rolls right (+x).
    gy > 0 tilts plate down away from you -> ball rolls away (+y).
    Therefore, if ball is at x > setpoint_x, we require gx < 0 (tilt left)
    to roll the ball back toward the setpoint.
    Command: gx = -(Kp * error_x + Ki * int_error_x + Kd * vx)
"""
import math
import time
import config as C


class PIDAxis:
    """1D PID controller with deadband, derivative-on-measurement, and anti-windup."""

    def __init__(self, kp=0.55, kd=0.35, ki=0.02, deadband=0.02,
                 max_integral=0.30, int_active_range=0.35):
        """
        Args:
            kp: Proportional gain.
            kd: Derivative gain (acts on estimated velocity).
            ki: Integral gain.
            deadband: Error deadband (errors smaller than this produce 0 proportional output).
            max_integral: Saturation limit for integral accumulator.
            int_active_range: Only accumulate integral when abs(error) is within this range.
        """
        self.kp = float(kp)
        self.kd = float(kd)
        self.ki = float(ki)
        self.deadband = float(deadband)
        self.max_integral = float(max_integral)
        self.int_active_range = float(int_active_range)

        self.integral = 0.0
        self.prev_error = 0.0
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_u = 0.0

    def reset(self):
        """Reset internal integrator and memory."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0
        self.last_u = 0.0

    def compute(self, dt, error, velocity=None):
        """
        Calculate control command for this axis.

        Args:
            dt: Time delta in seconds.
            error: Current error (pos - setpoint).
            velocity: Measured or estimated velocity. If None, derived from d(error)/dt.

        Returns:
            Control action u (positive tilts roll in + direction; our controller inverts this).
        """
        if dt <= 0:
            dt = 1e-3

        # Apply deadband
        if abs(error) < self.deadband:
            active_error = 0.0
        else:
            # Smooth deadband transition
            active_error = error - math.copysign(self.deadband, error)

        # Proportional term
        self.last_p = self.kp * active_error

        # Derivative term (using estimated velocity)
        if velocity is not None:
            self.last_d = self.kd * velocity
        else:
            self.last_d = self.kd * (error - self.prev_error) / dt
        self.prev_error = error

        # Integral term with anti-windup conditional integration
        if abs(error) < self.int_active_range and self.ki > 0:
            self.integral += active_error * dt
            self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        else:
            # Bleed off integrator if outside active window
            self.integral *= max(0.0, 1.0 - (dt * 5.0))

        self.last_i = self.ki * self.integral

        # Unclamped output
        self.last_u = self.last_p + self.last_i + self.last_d
        return self.last_u


class BallController:
    """2D Plate Balancing Controller combining X and Y axes with output saturation,
    slew-rate limiting, and setpoint trajectory generation.
    """

    SETPOINT_MODES = ("CENTER", "CIRCLE", "STEP_X", "STEP_Y", "CUSTOM")

    def __init__(self, kp=None, kd=None, ki=None,
                 max_tilt=None, deadband=None, max_slew=None):
        """
        Initialize controller with parameters from config.py or explicit overrides.
        """
        _kp = getattr(C, "CTRL_KP", 0.55) if kp is None else float(kp)
        _kd = getattr(C, "CTRL_KD", 0.35) if kd is None else float(kd)
        _ki = getattr(C, "CTRL_KI", 0.02) if ki is None else float(ki)
        _deadband = getattr(C, "CTRL_DEADBAND", 0.02) if deadband is None else float(deadband)

        self.max_tilt = getattr(C, "CTRL_MAX_TILT", 0.70) if max_tilt is None else float(max_tilt)
        self.max_slew = getattr(C, "CTRL_MAX_SLEW", 4.0) if max_slew is None else float(max_slew)

        self.pid_x = PIDAxis(kp=_kp, kd=_kd, ki=_ki, deadband=_deadband)
        self.pid_y = PIDAxis(kp=_kp, kd=_kd, ki=_ki, deadband=_deadband)

        # Output states
        self.cmd_gx = 0.0
        self.cmd_gy = 0.0

        # Setpoint trajectory management
        self.mode = "CENTER"
        self.sp_x = 0.0
        self.sp_y = 0.0
        self.t0 = time.time()
        self.circle_radius = 0.30
        self.circle_period = 6.0    # seconds per rotation
        self.step_amplitude = 0.35
        self.step_period = 4.0

    def set_gains(self, kp=None, kd=None, ki=None):
        """Update controller gains dynamically."""
        if kp is not None:
            self.pid_x.kp = self.pid_y.kp = float(kp)
        if kd is not None:
            self.pid_x.kd = self.pid_y.kd = float(kd)
        if ki is not None:
            self.pid_x.ki = self.pid_y.ki = float(ki)

    def set_p_only(self):
        """Switch to P-only control mode for Stage 6 tuning."""
        self.pid_x.kd = self.pid_y.kd = 0.0
        self.pid_x.ki = self.pid_y.ki = 0.0

    def restore_pd(self, default_kd=None):
        """Restore PD control mode for Stage 7."""
        _kd = getattr(C, "CTRL_KD", 0.35) if default_kd is None else default_kd
        self.pid_x.kd = self.pid_y.kd = float(_kd)

    def set_mode(self, mode, custom_sp=(0.0, 0.0)):
        """Set setpoint generator mode ('CENTER', 'CIRCLE', 'STEP_X', 'STEP_Y', 'CUSTOM')."""
        mode = mode.upper()
        if mode in self.SETPOINT_MODES:
            self.mode = mode
            if mode == "CUSTOM":
                self.sp_x, self.sp_y = custom_sp
            self.t0 = time.time()

    def cycle_mode(self):
        """Cycle through setpoint modes."""
        idx = (self.SETPOINT_MODES.index(self.mode) + 1) % len(self.SETPOINT_MODES)
        self.mode = self.SETPOINT_MODES[idx]
        self.t0 = time.time()
        return self.mode

    def get_setpoint(self, t):
        """Compute the current (sp_x, sp_y) based on active mode."""
        if self.mode == "CENTER":
            self.sp_x, self.sp_y = 0.0, 0.0
        elif self.mode == "CIRCLE":
            theta = 2.0 * math.pi * (t - self.t0) / self.circle_period
            self.sp_x = self.circle_radius * math.cos(theta)
            self.sp_y = self.circle_radius * math.sin(theta)
        elif self.mode == "STEP_X":
            phase = ((t - self.t0) % self.step_period) / self.step_period
            self.sp_x = self.step_amplitude if phase < 0.5 else -self.step_amplitude
            self.sp_y = 0.0
        elif self.mode == "STEP_Y":
            phase = ((t - self.t0) % self.step_period) / self.step_period
            self.sp_x = 0.0
            self.sp_y = self.step_amplitude if phase < 0.5 else -self.step_amplitude
        # CUSTOM retains existing sp_x, sp_y
        return self.sp_x, self.sp_y

    def reset(self):
        """Reset internal controller states."""
        self.pid_x.reset()
        self.pid_y.reset()
        self.cmd_gx = 0.0
        self.cmd_gy = 0.0

    def compute(self, dt, x, y, vx, vy, ball_tracked=True, t=None):
        """
        Compute tilt command (gx, gy) in [-1.0, 1.0].

        Args:
            dt: Time elapsed since last control step (seconds).
            x, y: Filtered ball position in plate coordinates [-1, 1].
            vx, vy: Filtered ball velocity in plate units/s.
            ball_tracked: True if ball is actively tracked; False if ball is lost.
            t: Current time (seconds). If None, uses time.time().

        Returns:
            (gx, gy): Slew-limited, saturated tilt command.
            (sp_x, sp_y): Current target setpoint.
            debug_dict: Dictionary with PID terms for telemetry.
        """
        now = time.time() if t is None else t
        sp_x, sp_y = self.get_setpoint(now)

        if not ball_tracked:
            # Ball is lost: smoothly ramp tilt back to level (0, 0)
            decay = max(0.0, 1.0 - (dt * self.max_slew))
            self.cmd_gx *= decay
            self.cmd_gy *= decay
            self.pid_x.reset()
            self.pid_y.reset()
            debug = {
                "sp_x": sp_x, "sp_y": sp_y,
                "err_x": 0.0, "err_y": 0.0,
                "px": 0.0, "ix": 0.0, "dx": 0.0,
                "py": 0.0, "iy": 0.0, "dy": 0.0,
                "mag": math.hypot(self.cmd_gx, self.cmd_gy)
            }
            return (self.cmd_gx, self.cmd_gy), (sp_x, sp_y), debug

        # Error definition: error = current - target
        err_x = x - sp_x
        err_y = y - sp_y

        # Feedback law: to correct positive error (ball to right of target),
        # we command negative tilt (gx < 0) so plate tilts down to the left.
        raw_gx = -self.pid_x.compute(dt, err_x, vx)
        raw_gy = -self.pid_y.compute(dt, err_y, vy)

        # 1. Vector magnitude saturation (preserves roll direction)
        mag = math.hypot(raw_gx, raw_gy)
        if mag > self.max_tilt and mag > 1e-6:
            scale = self.max_tilt / mag
            target_gx = raw_gx * scale
            target_gy = raw_gy * scale
        else:
            target_gx = raw_gx
            target_gy = raw_gy

        # 2. Slew rate limiter: clamp max change per second
        max_delta = self.max_slew * dt
        dgx = target_gx - self.cmd_gx
        dgy = target_gy - self.cmd_gy

        self.cmd_gx += max(-max_delta, min(max_delta, dgx))
        self.cmd_gy += max(-max_delta, min(max_delta, dgy))

        debug = {
            "sp_x": sp_x, "sp_y": sp_y,
            "err_x": err_x, "err_y": err_y,
            "px": self.pid_x.last_p, "ix": self.pid_x.last_i, "dx": self.pid_x.last_d,
            "py": self.pid_y.last_p, "iy": self.pid_y.last_i, "dy": self.pid_y.last_d,
            "mag": math.hypot(self.cmd_gx, self.cmd_gy)
        }
        return (self.cmd_gx, self.cmd_gy), (sp_x, sp_y), debug

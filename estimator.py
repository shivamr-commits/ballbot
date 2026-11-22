"""State estimator for ball position and velocity in plate coordinates.

Uses an independent 2D Kalman Filter (continuous-discrete acceleration model)
to filter spatial quantization/pixel jitter and compute smooth, latency-compensated
velocity estimates from discrete camera frames.
"""
import math
import time


class KalmanFilter1D:
    """1D Kinematic Kalman Filter for position and velocity.
    State vector: [position, velocity]^T.
    Process model: Constant velocity with discrete white noise acceleration.
    """

    def __init__(self, q=6.0, r=0.0001):
        """
        Args:
            q: Process noise parameter (acceleration disturbance variance).
               Higher = more responsive to sudden maneuvers;
               Lower  = smoother velocity, less sensitive to measurement jitter.
            r: Measurement noise variance (in normalized plate units^2).
               Typical pixel jitter is ~1 px / 320 px -> sigma ~ 0.003 -> r ~ 1e-5 to 1e-4.
        """
        self.q = float(q)
        self.r = float(r)
        self.p = 0.0      # estimated position
        self.v = 0.0      # estimated velocity
        # Covariance matrix P = [[p00, p01], [p01, p11]]
        self.p00 = 1.0
        self.p01 = 0.0
        self.p11 = 10.0
        self.initialized = False

    def reset(self, p=0.0, v=0.0):
        self.p = float(p)
        self.v = float(v)
        self.p00 = 0.05
        self.p01 = 0.0
        self.p11 = 1.0
        self.initialized = True

    def predict(self, dt):
        """Propagate state and covariance forward by dt seconds."""
        if dt <= 0:
            return self.p, self.v

        dt2 = dt * dt
        dt3 = dt2 * dt

        # State transition: p_new = p + v*dt, v_new = v
        self.p += self.v * dt

        # Covariance update: P_prior = F * P * F^T + Q
        # F = [[1, dt], [0, 1]]
        # Q = q * [[dt^3 / 3, dt^2 / 2], [dt^2 / 2, dt]]
        p00 = self.p00 + 2.0 * dt * self.p01 + dt2 * self.p11 + self.q * (dt3 / 3.0)
        p01 = self.p01 + dt * self.p11 + self.q * (dt2 / 2.0)
        p11 = self.p11 + self.q * dt

        self.p00 = p00
        self.p01 = p01
        self.p11 = p11
        return self.p, self.v

    def update(self, z):
        """Incorporate a new position measurement z."""
        if not self.initialized:
            self.reset(z, 0.0)
            return self.p, self.v

        # Measurement residual (innovation)
        y = z - self.p

        # Innovation covariance S = H * P * H^T + R = P00 + R
        s = self.p00 + self.r
        if s <= 0:
            s = 1e-6

        # Kalman gain K = P * H^T / S
        k0 = self.p00 / s
        k1 = self.p01 / s

        # State update
        self.p += k0 * y
        self.v += k1 * y

        # Covariance update: P_post = (I - K*H) * P_prior
        p00 = (1.0 - k0) * self.p00
        p01 = (1.0 - k0) * self.p01
        p11 = self.p11 - k1 * self.p01

        self.p00 = max(1e-7, p00)
        self.p01 = p01
        self.p11 = max(1e-7, p11)
        return self.p, self.v


class BallState:
    """Represents estimated state at a specific point in time."""
    __slots__ = ("t", "found", "tracking", "raw_x", "raw_y", "x", "y", "vx", "vy", "speed", "lost_duration")

    def __init__(self, t, found, tracking, raw_x, raw_y, x, y, vx, vy, lost_duration=0.0):
        self.t = t
        self.found = bool(found)
        self.tracking = bool(tracking)
        self.raw_x = raw_x
        self.raw_y = raw_y
        self.x = float(x)
        self.y = float(y)
        self.vx = float(vx)
        self.vy = float(vy)
        self.speed = math.hypot(self.vx, self.vy)
        self.lost_duration = float(lost_duration)

    def to_dict(self):
        return {
            "t": self.t,
            "found": self.found,
            "tracking": self.tracking,
            "raw_x": self.raw_x,
            "raw_y": self.raw_y,
            "x": self.x,
            "y": self.y,
            "vx": self.vx,
            "vy": self.vy,
            "speed": self.speed,
            "lost_duration": self.lost_duration,
        }

    def __repr__(self):
        return (f"BallState(found={self.found}, pos=({self.x:+.2f}, {self.y:+.2f}), "
                f"vel=({self.vx:+.2f}, {self.vy:+.2f}), speed={self.speed:.2f})")


class BallEstimator:
    """2D Ball State Estimator for the balancing plate.
    Filters camera positions and computes smooth velocities in normalized plate coordinates.
    """

    def __init__(self, q=6.0, r=0.0001, max_coast_time=0.10, lost_timeout=0.35):
        """
        Args:
            q: Process noise covariance parameter.
            r: Measurement noise covariance.
            max_coast_time: Max seconds to coast on prediction when frame is dropped.
            lost_timeout: Max seconds before declaring the ball officially lost.
        """
        self.kf_x = KalmanFilter1D(q=q, r=r)
        self.kf_y = KalmanFilter1D(q=q, r=r)
        self.max_coast_time = float(max_coast_time)
        self.lost_timeout = float(lost_timeout)

        self.last_t = None
        self.lost_t0 = None
        self.last_state = BallState(t=0.0, found=False, tracking=False,
                                    raw_x=None, raw_y=None, x=0.0, y=0.0, vx=0.0, vy=0.0)

    def reset(self):
        self.kf_x = KalmanFilter1D(q=self.kf_x.q, r=self.kf_x.r)
        self.kf_y = KalmanFilter1D(q=self.kf_y.q, r=self.kf_y.r)
        self.last_t = None
        self.lost_t0 = None
        self.last_state = BallState(t=0.0, found=False, tracking=False,
                                    raw_x=None, raw_y=None, x=0.0, y=0.0, vx=0.0, vy=0.0)

    def update(self, t, found, raw_x=None, raw_y=None):
        """
        Update the estimator with a vision reading.

        Args:
            t: Current timestamp (seconds, e.g. time.time()).
            found: Boolean indicating whether ball was detected in this frame.
            raw_x, raw_y: Normalized plate coordinates from vision (or None).

        Returns:
            BallState object containing filtered position and velocity.
        """
        if self.last_t is None:
            dt = 1.0 / 30.0
        else:
            dt = max(0.001, min(0.20, t - self.last_t))
        self.last_t = t

        if found and raw_x is not None and raw_y is not None:
            # Ball detected
            self.lost_t0 = None

            # If re-acquiring after being lost, reset state to measurement with zero velocity
            if not self.last_state.tracking:
                self.kf_x.reset(raw_x, 0.0)
                self.kf_y.reset(raw_y, 0.0)
                fx, vx = raw_x, 0.0
                fy, vy = raw_y, 0.0
            else:
                self.kf_x.predict(dt)
                self.kf_y.predict(dt)
                fx, vx = self.kf_x.update(raw_x)
                fy, vy = self.kf_y.update(raw_y)

            state = BallState(
                t=t,
                found=True,
                tracking=True,
                raw_x=raw_x,
                raw_y=raw_y,
                x=fx,
                y=fy,
                vx=vx,
                vy=vy,
                lost_duration=0.0,
            )
            self.last_state = state
            return state

        # Ball NOT detected
        if self.lost_t0 is None:
            self.lost_t0 = t
        lost_duration = t - self.lost_t0

        if lost_duration <= self.max_coast_time and self.last_state.tracking:
            # Coast for up to max_coast_time using internal prediction model
            fx, vx = self.kf_x.predict(dt)
            fy, vy = self.kf_y.predict(dt)
            state = BallState(
                t=t,
                found=False,
                tracking=True,
                raw_x=None,
                raw_y=None,
                x=fx,
                y=fy,
                vx=vx,
                vy=vy,
                lost_duration=lost_duration,
            )
        else:
            # Officially lost: decay velocity smoothly to 0 so controller does not run wild
            decay = max(0.0, 1.0 - (dt * 10.0))
            vx = self.last_state.vx * decay
            vy = self.last_state.vy * decay
            fx = self.last_state.x
            fy = self.last_state.y
            state = BallState(
                t=t,
                found=False,
                tracking=False,
                raw_x=None,
                raw_y=None,
                x=fx,
                y=fy,
                vx=vx,
                vy=vy,
                lost_duration=lost_duration,
            )

        self.last_state = state
        return state

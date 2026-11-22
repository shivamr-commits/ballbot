try:
    import lgpio
except ImportError:
    lgpio = None

import config as C


def angle_to_pulse(angle):
    """0-180 degrees -> 500-2500 us (same as the working interactive script)."""
    return int(C.MIN_PULSE + (angle / 180.0) * (C.MAX_PULSE - C.MIN_PULSE))


class Servos:
    """Holds all three servos. Unlike the interactive script, pulses keep running
    so the servos hold the plate; they stop only in close()."""

    def __init__(self, mock=False):
        self.mock = bool(mock) or (lgpio is None)
        self.h = None
        self.angles = [None, None, None]
        self.pulses = [None, None, None]

        if self.mock:
            print("[Servos] Running in MOCK mode (simulated servo pulses).")
        else:
            self.h = lgpio.gpiochip_open(0)
            for p in C.PINS:
                lgpio.gpio_claim_output(self.h, p)

    def write(self, angles):
        for i, a in enumerate(angles):
            a = max(C.MIN_DEG[i], min(C.MAX_DEG[i], a))   # never leave the free range
            p = angle_to_pulse(a)
            if p != self.pulses[i]:                        # only resend when it changes
                if not self.mock and self.h is not None:
                    lgpio.tx_servo(self.h, C.PINS[i], p)
                self.pulses[i] = p
            self.angles[i] = round(a, 1)
        return list(self.angles)

    def level(self):
        return self.write(C.LEVEL_DEG)

    def close(self):
        if not self.mock and self.h is not None:
            for p in C.PINS:
                lgpio.tx_servo(self.h, p, 0)
            lgpio.gpiochip_close(self.h)
            self.h = None

import lgpio
import config as C


class Servos:
    def __init__(self):
        self.h = lgpio.gpiochip_open(0)
        for p in C.PINS:
            lgpio.gpio_claim_output(self.h, p)
        self.pulses = [None, None, None]

    def write(self, pulses):
        for i, p in enumerate(pulses):
            p = int(max(C.MIN_US[i], min(C.MAX_US[i], p)))   # never leave the safe range
            if p != self.pulses[i]:                            # only resend when it changes
                lgpio.tx_servo(self.h, C.PINS[i], p)
                self.pulses[i] = p
        return list(self.pulses)

    def level(self):
        return self.write(C.LEVEL)

    def close(self):
        for p in C.PINS:
            lgpio.tx_servo(self.h, p, 0)
        lgpio.gpiochip_close(self.h)

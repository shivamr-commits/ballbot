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

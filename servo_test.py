import lgpio
import time

PIN = 17  # GPIO17 = physical pin 11

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, PIN)

CENTER, LEFT, RIGHT = 1500, 1000, 2000  # pulse width in microseconds


def move(pulse, wait=1.5):
    lgpio.tx_servo(h, PIN, pulse)  # 50 Hz servo pulse
    time.sleep(wait)


try:
    print("Center")
    move(CENTER)
    print("One side")
    move(LEFT)
    print("Other side")
    move(RIGHT)
    print("Back to center")
    move(CENTER)
finally:
    lgpio.tx_servo(h, PIN, 0)  # stop pulses
    lgpio.gpiochip_close(h)
    print("Done")

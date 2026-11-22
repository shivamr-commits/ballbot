import pigpio
import time

# Servo signal wire on GPIO17 (physical pin 11)
PIN = 17

pi = pigpio.pi()
if not pi.connected:
    print("Cannot reach pigpiod. Run: sudo systemctl enable --now pigpiod")
    raise SystemExit(1)

# Pulse width in microseconds: 1500 = center.
# Start with a narrow, safe range (1000-2000) so the servo does not hit its end stops.
CENTER, LEFT, RIGHT = 1500, 1000, 2000


def move(pulse, wait=1.5):
    pi.set_servo_pulsewidth(PIN, pulse)
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
    pi.set_servo_pulsewidth(PIN, 0)   # stop sending pulses
    pi.stop()
    print("Done")

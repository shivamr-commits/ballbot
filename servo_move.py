import lgpio
import time

# Physical pins 11, 13, 15 = GPIO17, GPIO27, GPIO22
PINS = [17, 27, 22]

MIN_PULSE = 500    # microseconds (0 degrees)
MAX_PULSE = 2500   # microseconds (180 degrees)

h = lgpio.gpiochip_open(0)
for p in PINS:
    lgpio.gpio_claim_output(h, p)


def angle_to_pulse(angle):
    """Convert 0-180 degrees to 500-2500 microseconds."""
    return int(MIN_PULSE + (angle / 180.0) * (MAX_PULSE - MIN_PULSE))


def move_servo(pin, angle, hold_secs=0.8):
    pulse = angle_to_pulse(angle)
    lgpio.tx_servo(h, pin, pulse)
    time.sleep(hold_secs)
    lgpio.tx_servo(h, pin, 0)  # stop pulses to prevent jitter


try:
    print("Interactive Servo Control (type 'q' to quit)\n")
    while True:
        servo_input = input(f"Which servo (1-{len(PINS)})? ").strip().lower()
        if servo_input == "q":
            break

        if not servo_input.isdigit() or not (1 <= int(servo_input) <= len(PINS)):
            print(f"Please enter a number between 1 and {len(PINS)}.\n")
            continue

        servo_num = int(servo_input)
        pin = PINS[servo_num - 1]

        angle_input = input(f"Move Servo {servo_num} (GPIO{pin}) to what degree (0-180)? ").strip().lower()
        if angle_input == "q":
            break

        try:
            angle = float(angle_input)
            if not (0 <= angle <= 180):
                print("Please enter an angle between 0 and 180.\n")
                continue
        except ValueError:
            print("Invalid number. Please try again.\n")
            continue

        print(f"Moving Servo {servo_num} (GPIO{pin}) to {angle}°...")
        move_servo(pin, angle)
        print("Done.\n")

except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    for p in PINS:
        lgpio.tx_servo(h, p, 0)
    lgpio.gpiochip_close(h)
    print("GPIO cleaned up. Exiting.")

"""
calibrate.py - measure your servos so servo_led_art.py can match them.
Run it on the ESP32 from Thonny and answer the questions in the Shell.

It finds, for each servo:
    SPAN_DEG   how many degrees it turns from 400 us to 2600 us
    DIRECTION  whether a bigger pulse turns it counterclockwise (+1) or clockwise (-1),
               as seen from the camera
and for the elbow:
    SERVO2_OFFSET  the servo angle at which the second arm is straight

Keep fingers and the arms clear: the servos go to their end stops.
"""

import time
from machine import Pin, PWM

SHOULDER_PIN = 4
ELBOW_PIN = 5
LED_PIN = 27
PULSE_MIN = 400
PULSE_MAX = 2600

Pin(LED_PIN, Pin.OUT, value=0)
servos = {
    "shoulder": PWM(Pin(SHOULDER_PIN), freq=50, duty_u16=0),
    "elbow": PWM(Pin(ELBOW_PIN), freq=50, duty_u16=0),
}


def go(name, us):
    us = min(max(int(us), PULSE_MIN), PULSE_MAX)
    servos[name].duty_ns(us * 1000)
    return us


def ask_number(prompt):
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("  Please type a number.")


def measure(name):
    print("\n=== {} servo ===".format(name.upper()))
    mid = (PULSE_MIN + PULSE_MAX) // 2
    go(name, mid)
    time.sleep(1)
    input("It's at the middle ({} us). Press Enter to move it to {} us...".format(mid, PULSE_MIN))
    go(name, PULSE_MIN)
    time.sleep(1.5)
    input("Mark where the arm points now. Press Enter to move it to {} us...".format(PULSE_MAX))
    go(name, PULSE_MAX)
    time.sleep(1.5)
    span = ask_number("How many degrees did it turn between the two marks? ")
    turn = ""
    while turn not in ("ccw", "cw"):
        turn = input("Seen from the camera, did it turn counterclockwise or clockwise? (ccw/cw) ").strip().lower()
    go(name, mid)
    time.sleep(1)
    return span, (1 if turn == "ccw" else -1)


def find_elbow_straight(span):
    print("\n=== ELBOW: find where the second arm is straight ===")
    print("Type + or - (or ++ / -- for bigger steps) and press Enter;")
    print("type ok when the two arms form one straight line.")
    us = go("elbow", (PULSE_MIN + PULSE_MAX) // 2)
    steps = {"+": 5, "-": -5, "++": 50, "--": -50}
    while True:
        cmd = input("  elbow at {} us > ".format(us)).strip()
        if cmd == "ok":
            return us, (us - PULSE_MIN) / (PULSE_MAX - PULSE_MIN) * span
        if cmd in steps:
            us = go("elbow", us + steps[cmd])


span1, dir1 = measure("shoulder")
span2, dir2 = measure("elbow")
straight_us, offset2 = find_elbow_straight(span2)

print("\n================ Copy these into servo_led_art.py ================")
print("SERVO1_SPAN_DEG = {:.1f}".format(span1))
print("SERVO2_SPAN_DEG = {:.1f}".format(span2))
print("SERVO1_OFFSET, SERVO1_DIRECTION = 0.0, {}   # offset is chosen automatically".format(dir1))
print("SERVO2_OFFSET, SERVO2_DIRECTION = {:.1f}, {}".format(offset2, dir2))
print("(elbow is straight at {} us)".format(straight_us))
print("==================================================================")

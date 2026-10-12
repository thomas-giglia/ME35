"""
draw.py - MicroPython program for the ESP32 that plays back a path made by
servo_led_art.py: two servos move the arm and an LED draws the lines.

Wiring:
    GPIO4  (D4)  -> shoulder servo signal
    GPIO5  (D5)  -> elbow servo signal
    GPIO27 (D27) -> LED (+) through a ~220 ohm resistor, LED (-) -> GND
    Servo power  -> a separate 5 V supply (not the ESP32's 3V3 pin),
                    with its GND connected to the ESP32's GND

Setup in Thonny:
    1. Run servo_led_art.py on your computer. In the logo's folder it makes
       LogoName_esp32.csv.
    2. In Thonny: View > Files. Find that CSV in the top (computer) pane,
       right-click it and choose "Upload to /".
    3. Set PATH_FILE below to the CSV's name, upload/save this file to the
       ESP32 as draw.py, and run it (F5).
    Press Stop in Thonny at any time; the LED turns off.

Also works with the full angles.csv (it looks up the columns by name), but
that file is much larger.
"""

import time
from machine import Pin, PWM

# ======================= Settings =======================
PATH_FILE = "LogoName_esp32.csv"

SHOULDER_PIN = 4
ELBOW_PIN = 5
LED_PIN = 27

PULSE_MIN_US = 400        # never send pulses outside these
PULSE_MAX_US = 2600

MS_PER_POINT = 20         # time per drawing point (points are 0.5 mm apart).
                          # 20 ms = 25 mm/s. Servos update every 20 ms, so lower
                          # values skip points rather than adding detail.
TRAVEL_MS_PER_US = 0.15   # LED-off moves: wait this many ms per us of pulse change
TRAVEL_SETTLE_MS = 150    # ...plus this, so the arm stops wobbling before drawing
END_HOLD_MS = 40          # keep the LED on this long at the end of each line,
                          # since the servos lag slightly behind the commands
START_DELAY_S = 5         # countdown before drawing starts (open the shutter)
LED_ENABLED = True        # False = practice run with the LED kept off
# ========================================================

HAS_DUTY_NS = hasattr(PWM, "duty_ns")


def set_pulse(pwm, us):
    us = min(max(us, PULSE_MIN_US), PULSE_MAX_US)
    if HAS_DUTY_NS:
        pwm.duty_ns(us * 1000)
    else:
        pwm.duty_u16(us * 65535 // 20000)  # 20 ms period at 50 Hz


def make_servo(pin, us):
    """Start the PWM already at the first position, so the servo doesn't
    jump to a random angle first."""
    us = min(max(us, PULSE_MIN_US), PULSE_MAX_US)
    try:
        return PWM(Pin(pin), freq=50, duty_ns=us * 1000)
    except (TypeError, ValueError):
        pwm = PWM(Pin(pin), freq=50)
        set_pulse(pwm, us)
        return pwm


def rows(path):
    """Yield (light, pulse1, pulse2) one line at a time, so even big files
    don't fill the ESP32's memory."""
    with open(path) as f:
        head = f.readline().strip().split(",")
        il = head.index("light")
        i1 = head.index("pulse1_us")
        i2 = head.index("pulse2_us")
        for line in f:
            parts = line.strip().split(",")
            if len(parts) > max(il, i1, i2):
                yield int(parts[il]), int(parts[i1]), int(parts[i2])


def travel_ms(a1, a2, b1, b2):
    change = max(abs(b1 - a1), abs(b2 - a2))
    return int(change * TRAVEL_MS_PER_US) + TRAVEL_SETTLE_MS


def survey(path):
    """One pass through the file: first point, point count, estimated time."""
    first, n, lines, total_ms = None, 0, 0, 0
    prev = None
    for light, p1, p2 in rows(path):
        if first is None:
            first = (p1, p2)
            prev = (p1, p2)
        n += 1
        if light:
            total_ms += MS_PER_POINT
        else:
            lines += 1
            total_ms += travel_ms(prev[0], prev[1], p1, p2) + END_HOLD_MS
        prev = (p1, p2)
    return first, n, lines, total_ms


def main():
    led = Pin(LED_PIN, Pin.OUT, value=0)

    print("Reading", PATH_FILE, "...")
    first, n, lines, total_ms = survey(PATH_FILE)
    if first is None:
        print("The file has no points.")
        return
    print("{} points, {} lines, about {:.0f} s to draw".format(n, lines, total_ms / 1000))

    shoulder = make_servo(SHOULDER_PIN, first[0])
    elbow = make_servo(ELBOW_PIN, first[1])
    time.sleep_ms(1500)  # let the arm reach the start point

    for s in range(START_DELAY_S, 0, -1):
        print("Starting in", s)
        time.sleep(1)
    print("Drawing...")

    start = time.ticks_ms()
    prev1, prev2 = first
    led_on = False
    deadline = time.ticks_ms()
    try:
        for i, (light, p1, p2) in enumerate(rows(PATH_FILE)):
            if light == 0:
                # End of a line (or the very start): finish, LED off, travel.
                if led_on:
                    time.sleep_ms(END_HOLD_MS)
                    led.value(0)
                    led_on = False
                set_pulse(shoulder, p1)
                set_pulse(elbow, p2)
                time.sleep_ms(travel_ms(prev1, prev2, p1, p2))
                deadline = time.ticks_ms()
            else:
                if not led_on and LED_ENABLED:
                    led.value(1)
                    led_on = True
                set_pulse(shoulder, p1)
                set_pulse(elbow, p2)
                # Fixed time per point, however long the file reading took.
                deadline = time.ticks_add(deadline, MS_PER_POINT)
                wait = time.ticks_diff(deadline, time.ticks_ms())
                if wait > 0:
                    time.sleep_ms(wait)
            prev1, prev2 = p1, p2

            if i % 500 == 0:
                print("{:3d}%".format(100 * i // n))
        time.sleep_ms(END_HOLD_MS)
    finally:
        led.value(0)  # always turn the LED off, even if you press Stop

    print("Done in {:.1f} s. Close the shutter.".format(
        time.ticks_diff(time.ticks_ms(), start) / 1000))
    # The servos keep holding their last position. To let them go limp:
    # shoulder.deinit(); elbow.deinit()


main()

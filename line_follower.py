# line_follower.py
# One-sensor EDGE follower for the Cytron Robo ESP32 (MicroPython).
# The color sensor is centered at the front of the car and rides the
# RIGHT edge of the black tape:
#   - sees white  -> tape is to the left  -> turn left
#   - sees black  -> sensor is on the tape -> turn right
#   - sees gray   -> right on the edge     -> drive straight
# Steering is proportional: the whiter or blacker the reading, the
# harder it turns, so it handles angled pieces of tape at the corners.
#
# Colored sections: the car keeps the turn it was making (so a color right
# after a corner doesn't send it off course) and signals the color.
# Line lost: it stops and swings left/right in place, wider each time,
# until it finds the tape again.
#
# Press the onboard button on D35 to start. Press it again to stop
# (then press again to restart).
#
# Needs these files on the ESP32:
#   line_follower_external_hardware.py, knn_color_classifier.py,
#   color_samples.py (made by calibrate_color_sensors.py),
#   lights_and_sound.py (NeoPixels and buzzer),
#   optionally color_messenger.py (sends colors to another ESP32),
#   and main.py (so it runs without the computer plugged in)

from line_follower_external_hardware import make_sensor, make_motors
from knn_color_classifier import ColorClassifier
import lights_and_sound as fx
from machine import Pin
import time

# ----------------------------------------------------------------------
# Tuning
# ----------------------------------------------------------------------
BASE_SPEED    = 0.25      # forward speed, 0 to 1
KP            = 0.25      # steering strength; raise if it misses corners,
                          # lower if it swings past the edge
KD            = 0.3       # damping: eases the turn as the sensor nears the
                          # edge, so it doesn't swing past it (0 = off)
MAX_TURN      = 0.15      # most each wheel's speed may differ from the forward
                          # speed (limits how FAST the car turns)
TARGET        = 0.5       # darkness to hold: 0.5 = halfway on the edge
CORNER_SLOWDOWN = 0.8     # 0-1: how much to slow down in a corner
                          # (0.8 = down to 20% of BASE_SPEED); raise for
                          # tighter corners
CORNER_SMOOTH = 0.3       # 0-1: how quickly a corner is recognized; lower =
                          # needs a longer run of black (or white) to slow down
CORNER_DEADBAND = 0.25    # 0-0.5: averaged error below this = not a corner,
                          # full speed. Raise if it still slows on straights
SHOW_READINGS = False     # True: print darkness and speed to Thonny's Shell
LOST_MS       = 800       # white this long = line lost: stop and search
TURN_SPEED    = 0.22      # spin speed while searching
SWEEP_STEP_MS = 150       # first search swing to each side; grows each pass
SWEEP_MAX_MS  = 1200      # widest swing before spinning all the way around
RIGHT_SWEEP_SCALE = 0.5   # right swings are this many times wider than left
                          # ones (the car follows the right edge of the tape)
PAUSE_MS      = 40        # short stop before changing spin direction
HOLD_TURN     = True      # on colored sections, keep turning the way the car
                          # was just turning (False = drive straight)

COLORS        = ("red", "green", "blue")   # labels that mean "drive straight"
STABLE_COUNT  = 2         # color readings in a row to count as a colored section
COLOR_EXIT_COUNT = 3      # non-color readings in a row to count as leaving it
COLOR_COOLDOWN_MS = 1500  # the same color seen again within this time is the
                          # same strip, so it isn't announced again
PRINT_CHANGES = True      # print to Thonny's Shell when something happens

BUTTON_PIN    = 35        # onboard programmable button (reads 0 when pressed)

# ----------------------------------------------------------------------
# Setup
# ----------------------------------------------------------------------
sensor = make_sensor()
left_motor, right_motor = make_motors()
classifier = ColorClassifier()
button = Pin(BUTTON_PIN, Pin.IN)   # D35 is input-only; the board has its own pull-up

# Black's brightness relative to white, from the calibration samples
_black = [s[3] for s in classifier.samples if s[4] == "black"]
BLACK_RATIO = sum(_black) / len(_black) if _black else 0.1

# Wireless color messages to the other ESP32 (keeps running without it)
try:
    import color_messenger
    color_messenger.start()
except Exception as err:
    print("Color messages disabled:", err)
    color_messenger = None


def drive(left, right):
    left_motor.set(left)
    right_motor.set(right)


def darkness(reading):
    """0 = white paper, 1 = black tape, from the white/clear channel."""
    ratio = reading[3] / classifier.white_c
    d = (1 - ratio) / (1 - BLACK_RATIO)
    return max(0.0, min(1.0, d))


class StopPressed(Exception):
    pass


def pressed():
    """True once per button press (waits for release, ignores bounce)."""
    if button.value() == 0:
        time.sleep_ms(30)
        if button.value() == 0:
            while button.value() == 0:
                time.sleep_ms(10)
            time.sleep_ms(30)
            return True
    return False


def pause():
    drive(0, 0)
    time.sleep_ms(PAUSE_MS)


def spin_until_black(direction, max_ms):
    """Spin in place (+1 = left, -1 = right) until black or gray is seen,
    or max_ms passes. Returns (ms spun, found)."""
    start = time.ticks_ms()
    drive(-direction * TURN_SPEED, direction * TURN_SPEED)
    while True:
        fx.update()
        if pressed():
            raise StopPressed
        elapsed = time.ticks_diff(time.ticks_ms(), start)
        if classifier.classify(sensor.rgbc()) in ("black", "gray"):
            return elapsed, True
        if elapsed >= max_ms:
            return elapsed, False


def search(first_dir):
    """Swings left and right in place, wider each time, until the tape
    is found. Starts with first_dir (+1 = left, -1 = right)."""
    if PRINT_CHANGES:
        print("line lost - searching", "left" if first_dir > 0 else "right", "first")
    pause()
    offset = 0                 # net ms spun; + = left of where it started
    level = 1
    while level * SWEEP_STEP_MS <= SWEEP_MAX_MS:
        for side in (first_dir, -first_dir):
            scale = RIGHT_SWEEP_SCALE if side < 0 else 1.0
            target = side * level * SWEEP_STEP_MS * scale
            direction = 1 if target > offset else -1
            t, found = spin_until_black(direction, int(abs(target - offset)))
            offset += direction * t
            if found:
                pause()
                if PRINT_CHANGES:
                    print("found the line")
                return
            pause()
        level += 1
    # Still nothing: keep spinning right until the tape turns up
    spin_until_black(-1, 10 ** 9)
    pause()


def wait_for_button():
    drive(0, 0)
    fx.quiet()
    print("Press the D35 button to start.")
    while not pressed():
        time.sleep_ms(10)
    print("Driving. Press D35 again to stop.")


# ----------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------
try:
    wait_for_button()
    on_color = False           # True while crossing a red/green/blue section
    color_streak = 0
    other_streak = 0
    white_since = None
    last_error = 0.0
    avg_turn = 0.0             # recent average steering (+ = right, - = left)
    corner_error = 0.0         # recent average error; stays large only in corners
    last_color = None          # last color announced
    last_color_ms = 0          # when the car was last on that color
    loops = 0
    hold = 0.0                 # steering held while on a colored section

    fx.lights_off()
    while True:
        try:
            fx.update()            # keeps any chime playing
            if pressed():
                raise StopPressed

            reading = sensor.rgbc()
            label = classifier.classify(reading)

            # --- colored sections: drive straight across them ---------------
            color_streak = color_streak + 1 if label in COLORS else 0
            other_streak = other_streak + 1 if label not in COLORS else 0

            if not on_color and color_streak >= STABLE_COUNT:
                on_color = True
                hold = avg_turn if HOLD_TURN else 0.0
                now = time.ticks_ms()
                repeat = (label == last_color and
                          time.ticks_diff(now, last_color_ms) < COLOR_COOLDOWN_MS)
                if not repeat:
                    if color_messenger:
                        color_messenger.send_color(label)
                    fx.show_color(label)   # stays lit until the next color
                    fx.chime(label)
                    if PRINT_CHANGES:
                        print("color section:", label)
                last_color = label
                last_color_ms = now
            elif on_color and label in COLORS:
                last_color_ms = time.ticks_ms()   # still on the strip
            elif on_color and other_streak >= COLOR_EXIT_COUNT:
                on_color = False
                if PRINT_CHANGES:
                    print("left the color section")
                if label == "white":
                    # Came off the color onto paper: find the tape right away,
                    # starting on the side the car was turning toward
                    search(-1 if hold > 0 else 1)
                    last_error = corner_error = 0.0
                    white_since = None
                    continue

            if on_color or label in COLORS:
                # Colored section: keep the turn the car was making (or go straight)
                t = hold if on_color else (avg_turn if HOLD_TURN else 0.0)
                drive(max(0.0, BASE_SPEED + t), max(0.0, BASE_SPEED - t))
                white_since = None
                continue

            # --- line lost: white for too long -------------------------------
            if label == "white":
                if white_since is None:
                    white_since = time.ticks_ms()
                elif time.ticks_diff(time.ticks_ms(), white_since) > LOST_MS:
                    search(-1 if avg_turn > 0 else 1)
                    last_error = corner_error = 0.0
                    white_since = None
                    continue
            else:
                white_since = None

            # --- edge following ---------------------------------------------
            # error > 0: too dark (on black) -> turn right
            # error < 0: too light (on white) -> turn left
            error = darkness(reading) - TARGET
            turn_amount = KP * error + KD * (error - last_error)
            last_error = error
            turn_amount = max(-MAX_TURN, min(MAX_TURN, turn_amount))
            avg_turn = 0.8 * avg_turn + 0.2 * turn_amount
            # Corner detection: on straight tape the readings flip between
            # black and white, so their average stays small. In a corner they
            # stay on one side, the average grows, and the car slows down so it
            # curves tighter without turning any faster.
            corner_error += CORNER_SMOOTH * (error - corner_error)
            off = (abs(corner_error) - CORNER_DEADBAND) / (0.5 - CORNER_DEADBAND)
            off = max(0.0, min(1.0, off))
            forward = BASE_SPEED * (1 - CORNER_SLOWDOWN * off)
            drive(forward + turn_amount, forward - turn_amount)

            loops += 1
            if SHOW_READINGS and loops % 5 == 0:
                print("dark %.2f  avg err %+.2f  speed %.2f"
                      % (darkness(reading), corner_error, forward))

        except StopPressed:
            wait_for_button()
            on_color = False
            white_since = None
            last_error = avg_turn = hold = corner_error = 0.0
            color_streak = other_streak = 0

finally:
    drive(0, 0)                # stop motors however the program ends
    fx.quiet()
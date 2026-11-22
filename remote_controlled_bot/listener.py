# listener.py – Listener (central) for the joystick yeller
# Receives "x,y\n" joystick packets (0–1, center 0.5), drives two motors,
# and sends "finish\n" back once the color sensor sees red.
# Import this from main.py and call run(), or run this file directly in Thonny.

from BLE_CEEO import Listen
from machine import Pin, PWM, I2C
import time
import json
import gc

NAME = "Tommy"       # must match the yeller's name exactly

# ---------------- Motors ----------------
LEFT_PINS = (12, 13)     # (forward pin, reverse pin)
RIGHT_PINS = (14, 17)
PWM_FREQ = 1000          # Hz (raise to 20000 if the motors whine)

DEADZONE = 0.08          # ignore small wobble around center
MIN_DUTY = 350           # lowest duty that actually turns the wheels (0–1023)
MAX_DUTY = 1023

# Flip these if the car drives backward or turns the wrong way
INVERT_THROTTLE = False
INVERT_TURN = False

FAILSAFE_MS = 600        # stop if no packet arrives for this long (yeller keepalive is 300 ms)

# ---------------- Finish detection ----------------
SDA_PIN, SCL_PIN = 21, 22
COLOR_PERIOD_MS = 50     # how often to read the color sensor
RED_LABEL = "red"        # must match the label you used in calibrate_color_sensors.py
RED_CONFIRM = 3          # consecutive red readings before calling it a finish
REARM_MS = 2000          # must leave red this long before another finish can fire
STOP_AT_FINISH_MS = 3000 # pause the motors at the finish (0 to keep driving)
DEBUG_COLOR = True       # print sensor readings twice a second for calibration


class Motor:
    def __init__(self, pin_fwd, pin_rev):
        self.fwd = PWM(Pin(pin_fwd), freq=PWM_FREQ, duty=0)
        self.rev = PWM(Pin(pin_rev), freq=PWM_FREQ, duty=0)

    def set(self, duty):
        # Positive = forward, negative = reverse, 0 = coast
        duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
        if duty >= 0:
            self.rev.duty(0)
            self.fwd.duty(duty)
        else:
            self.fwd.duty(0)
            self.rev.duty(-duty)


class TCS34725:
    # Minimal driver for the common TCS34725 RGB color sensor (I2C address 0x29)
    ADDR = 0x29

    def __init__(self, i2c):
        self.i2c = i2c
        chip_id = self._read8(0x12)
        if chip_id not in (0x44, 0x4D):
            raise OSError("unexpected color sensor id: %s" % hex(chip_id))
        self._write8(0x01, 0xF6)   # integration time ~24 ms (fast updates)
        self._write8(0x0F, 0x01)   # gain 4x
        self._write8(0x00, 0x01)   # power on
        time.sleep_ms(3)
        self._write8(0x00, 0x03)   # power on + enable color reading
        time.sleep_ms(30)

    def _write8(self, reg, value):
        self.i2c.writeto_mem(self.ADDR, 0x80 | reg, bytes([value]))

    def _read8(self, reg):
        return self.i2c.readfrom_mem(self.ADDR, 0x80 | reg, 1)[0]

    def rgbc(self):
        # Returns (red, green, blue, clear), the order the KNN classifier expects
        d = self.i2c.readfrom_mem(self.ADDR, 0xA0 | 0x14, 8)
        c = d[0] | d[1] << 8
        r = d[2] | d[3] << 8
        g = d[4] | d[5] << 8
        b = d[6] | d[7] << 8
        return (r, g, b, c)


# Shared state
left = None
right = None
sensor = None
classifier = None
buffer = b""
target = (0.5, 0.5)      # latest joystick reading (x, y)
last_packet = 0


def stop():
    left.set(0)
    right.set(0)


def centered(v):
    # 0–1 joystick value -> -1..1 with a deadzone, rescaled so motion starts smoothly
    v = max(0.0, min(1.0, v)) * 2 - 1
    if abs(v) < DEADZONE:
        return 0.0
    sign = 1 if v > 0 else -1
    return sign * (abs(v) - DEADZONE) / (1 - DEADZONE)


def to_duty(v):
    # -1..1 -> signed duty, jumping past the stall zone once moving
    if v == 0:
        return 0
    duty = int(MIN_DUTY + abs(v) * (MAX_DUTY - MIN_DUTY))
    return duty if v > 0 else -duty


def drive(x_val, y_val):
    turn = centered(x_val)
    throttle = centered(y_val)
    if INVERT_TURN:
        turn = -turn
    if INVERT_THROTTLE:
        throttle = -throttle

    # Arcade-drive mixing: push forward to go, push sideways to turn
    l = max(-1.0, min(1.0, throttle + turn))
    r = max(-1.0, min(1.0, throttle - turn))
    left.set(to_duty(l))
    right.set(to_duty(r))


def callback(data):
    # Only parse and store here. Never block inside the BLE callback.
    global buffer, target, last_packet
    buffer += data
    if len(buffer) > 200:          # garbage protection
        buffer = buffer[-40:]
    while b"\n" in buffer:
        line, buffer = buffer.split(b"\n", 1)
        try:
            text = line.decode().strip()
            if not text:
                continue
            if text.startswith("{"):           # old JSON format still works
                msg = json.loads(text)
                xv, yv = float(msg["x"]), float(msg["y"])
            else:                              # compact "0.512,0.348"
                xs, ys = text.split(",")
                xv, yv = float(xs), float(ys)
            target = (xv, yv)
            last_packet = time.ticks_ms()
        except (ValueError, KeyError, UnicodeError):
            pass                               # skip a damaged line


def send_finish(p):
    # "finish" matters, so retry through a briefly full radio buffer
    for _ in range(10):
        try:
            p.send("finish\n")
            print("Sent finish")
            return True
        except OSError as e:
            if e.args[0] != 12:    # anything other than ENOMEM is a real error
                raise
            time.sleep_ms(30)
    print("Could not send finish")
    return False


def run():
    global left, right, sensor, classifier, buffer, last_packet

    left = Motor(*LEFT_PINS)
    right = Motor(*RIGHT_PINS)
    stop()

    try:
        sensor = TCS34725(I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=400000))
        print("Color sensor ready")
    except Exception as e:
        sensor = None
        print("No color sensor, finish detection off:", e)

    # SystemExit is caught too, so a missing color_samples.py doesn't stop the car
    try:
        from knn_color_classifier import ColorClassifier
        classifier = ColorClassifier()
        print("Color classifier loaded with", len(classifier.samples), "samples")
    except (Exception, SystemExit) as e:
        classifier = None
        print("No color classifier, finish detection off:", e)

    p = None
    try:
        while True:   # reconnect forever
            gc.collect()
            try:
                p = Listen(NAME, verbose=False)
                p.callback = callback

                print("Looking for", NAME, "...")
                if p.connect_up():
                    print("Connected")
                    buffer = b""
                    last_packet = time.ticks_ms()

                    red_count = 0
                    finish_armed = True
                    last_red = time.ticks_ms()
                    pause_until = time.ticks_ms()
                    last_color = time.ticks_ms()
                    last_debug = time.ticks_ms()

                    while p.is_connected:
                        now = time.ticks_ms()

                        # ---- Finish detection ----
                        if sensor and classifier and time.ticks_diff(now, last_color) >= COLOR_PERIOD_MS:
                            last_color = now
                            try:
                                reading = sensor.rgbc()
                                label = classifier.classify(reading)
                            except OSError:
                                reading = None

                            if reading:
                                red = (label == RED_LABEL)
                                if DEBUG_COLOR and time.ticks_diff(now, last_debug) >= 500:
                                    last_debug = now
                                    r, g, b, c = reading
                                    print("R=%d G=%d B=%d C=%d -> %s" % (r, g, b, c, label))

                                if red:
                                    red_count += 1
                                    last_red = now
                                else:
                                    red_count = 0
                                    # Re-arm only after being off red for a while
                                    if not finish_armed and time.ticks_diff(now, last_red) >= REARM_MS:
                                        finish_armed = True

                                if finish_armed and red_count >= RED_CONFIRM:
                                    finish_armed = False
                                    print("Finish line reached")
                                    send_finish(p)
                                    if STOP_AT_FINISH_MS:
                                        pause_until = time.ticks_add(now, STOP_AT_FINISH_MS)

                        # ---- Motors ----
                        if time.ticks_diff(now, pause_until) < 0:
                            stop()                      # celebrating at the finish
                        elif time.ticks_diff(now, last_packet) > FAILSAFE_MS:
                            stop()                      # no recent packets, fail safe
                        else:
                            drive(*target)

                        time.sleep_ms(10)
                print("Lost connection")
            except Exception as e:
                print("Error:", e)
            finally:
                stop()
                if p:
                    p.disconnect()
                p = None
            time.sleep(1)
    finally:
        stop()


# Lets you still run this file directly from Thonny for testing
if __name__ == "__main__":
    run()

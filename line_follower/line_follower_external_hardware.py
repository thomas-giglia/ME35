# line_follower_external_hardware.py
# Shared hardware setup for the line follower (Cytron Robo ESP32, MicroPython).
# Other files import from here, so pins and drivers are defined in one place.

from machine import Pin, PWM, I2C
import time

# ----------------------------------------------------------------------
# Pins
# ----------------------------------------------------------------------
SENSOR_PINS       = (16, 17)   # Grove 1 (use (32, 33) if plugged into Grove 5)
LEFT_MOTOR_PINS   = (12, 13)   # Motor 1
RIGHT_MOTOR_PINS  = (14, 27)   # Motor 2

# Set to True if that wheel spins backward when it should go forward
LEFT_MOTOR_REVERSED  = False
RIGHT_MOTOR_REVERSED = False

PWM_FREQ = 10000               # Hz (motor driver max is 20 kHz)

# ----------------------------------------------------------------------
# Color sensor - detects either chip automatically:
#   VEML6040 at 0x10 (red, green, blue, white; 40 ms per reading)
#   TCS34725 at 0x29 (red, green, blue, clear; 7.2 ms per reading)
# ----------------------------------------------------------------------
VEML6040_ADDR = 0x10
TCS34725_ADDR = 0x29
SUPPORTED = (VEML6040_ADDR, TCS34725_ADDR)


class ColorSensor:
    def __init__(self, bus_id, pins):
        self.i2c = None
        a, b = pins
        # Try both pin orders so you don't have to know which is SDA/SCL
        seen = []
        for scl, sda in ((a, b), (b, a)):
            i2c = I2C(bus_id, scl=Pin(scl), sda=Pin(sda), freq=400000)
            found = i2c.scan()
            seen += [hex(x) for x in found if hex(x) not in seen]
            for addr in SUPPORTED:
                if addr in found:
                    self.i2c = i2c
                    self.addr = addr
                    break
            if self.i2c:
                break
        if self.i2c is None:
            raise OSError("No supported color sensor on pins %d and %d. "
                          "Devices found: %s. Run i2c_check.py for details."
                          % (a, b, seen if seen else "none"))

        if self.addr == VEML6040_ADDR:
            # Config: 40 ms integration, auto mode, sensor enabled
            self.i2c.writeto_mem(self.addr, 0x00, bytes([0x00, 0x00]))
            self.interval_ms = 41
        else:
            w = lambda reg, v: self.i2c.writeto_mem(self.addr, 0x80 | reg, bytes([v]))
            w(0x00, 0x01)        # power on
            time.sleep_ms(3)
            w(0x01, 0xFD)        # integration time: 3 cycles = 7.2 ms
            w(0x0F, 0x01)        # gain: 4x
            w(0x00, 0x03)        # power on + measure continuously
            self.interval_ms = 8
        self._last = time.ticks_add(time.ticks_ms(), -self.interval_ms)
        time.sleep_ms(self.interval_ms + 5)   # let the first reading finish

    def _wait_for_new_reading(self):
        # Reading faster than the sensor measures just returns the same
        # value again, so wait until a fresh one is ready.
        wait = self.interval_ms - time.ticks_diff(time.ticks_ms(), self._last)
        if wait > 0:
            time.sleep_ms(wait)
        self._last = time.ticks_ms()

    def _word(self, reg):
        d = self.i2c.readfrom_mem(self.addr, reg, 2)
        return d[0] | (d[1] << 8)

    def rgbc(self):
        """Returns (red, green, blue, clear/white) raw readings."""
        self._wait_for_new_reading()
        if self.addr == VEML6040_ADDR:
            return (self._word(0x08), self._word(0x09),
                    self._word(0x0A), self._word(0x0B))
        d = self.i2c.readfrom_mem(self.addr, 0xA0 | 0x14, 8)
        c = d[0] | (d[1] << 8)
        r = d[2] | (d[3] << 8)
        g = d[4] | (d[5] << 8)
        b = d[6] | (d[7] << 8)
        return r, g, b, c

    def brightness(self):
        return self.rgbc()[3]


# ----------------------------------------------------------------------
# Motor
# ----------------------------------------------------------------------
class Motor:
    def __init__(self, pins, reversed_):
        a, b = pins
        if reversed_:
            a, b = b, a
        self.a = PWM(Pin(a))
        self.b = PWM(Pin(b))
        for p in (self.a, self.b):
            p.freq(PWM_FREQ)
            p.duty_u16(0)

    def set(self, speed):
        """speed from -1 (full backward) to 1 (full forward)."""
        speed = max(-1.0, min(1.0, speed))
        duty = int(abs(speed) * 65535)
        if speed >= 0:
            self.b.duty_u16(0)
            self.a.duty_u16(duty)
        else:
            self.a.duty_u16(0)
            self.b.duty_u16(duty)


# ----------------------------------------------------------------------
# Helpers for other files
# ----------------------------------------------------------------------
def make_sensor():
    """Returns the color sensor."""
    return ColorSensor(0, SENSOR_PINS)


def make_motors():
    """Returns (left_motor, right_motor)."""
    return (Motor(LEFT_MOTOR_PINS, LEFT_MOTOR_REVERSED),
            Motor(RIGHT_MOTOR_PINS, RIGHT_MOTOR_REVERSED))

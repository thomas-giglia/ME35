# lights_and_sound.py
# Onboard NeoPixels (D15) and piezo buzzer (D23) on the Cytron Robo ESP32.
# Imported by line_follower.py.

from machine import Pin, PWM
import neopixel
import time

NEOPIXEL_PIN = 15
NUM_PIXELS   = 2               # the board has 2 NeoPixels
BRIGHTNESS   = 40              # 0-255; they're very bright at full power
BUZZER_PIN   = 23
VOLUME       = 0.5             # 0-1 (0.5 is the loudest for a piezo)

# A short rising chime for each color: (frequency in Hz, length in ms)
CHIMES = {
    "red":   ((784, 90), (1047, 140)),     # G5 -> C6
    "green": ((880, 90), (1175, 140)),     # A5 -> D6
    "blue":  ((988, 90), (1319, 140)),     # B5 -> E6
}

_COLOR_RGB = {
    "red":   (BRIGHTNESS, 0, 0),
    "green": (0, BRIGHTNESS, 0),
    "blue":  (0, 0, BRIGHTNESS),
}

_pixels = neopixel.NeoPixel(Pin(NEOPIXEL_PIN, Pin.OUT), NUM_PIXELS)
_buzzer = PWM(Pin(BUZZER_PIN))
_buzzer.duty_u16(0)

_notes = []                    # notes still to play
_note_ends = 0                 # when the current note finishes


def show_color(color):
    """Lights both NeoPixels in the color ("red", "green" or "blue")."""
    rgb = _COLOR_RGB.get(color, (0, 0, 0))
    for i in range(NUM_PIXELS):
        _pixels[i] = rgb
    _pixels.write()


def lights_off():
    show_color(None)


def chime(color):
    """Starts the chime for that color. It plays in the background:
    call update() every loop so the car keeps driving meanwhile."""
    global _notes, _note_ends
    _notes = list(CHIMES.get(color, ()))
    _note_ends = time.ticks_ms()
    update()


def update():
    """Moves the chime on to its next note when it's time."""
    global _note_ends
    if time.ticks_diff(time.ticks_ms(), _note_ends) < 0:
        return                 # current note still playing
    if _notes:
        freq, length = _notes.pop(0)
        _buzzer.freq(freq)
        _buzzer.duty_u16(int(VOLUME * 65535))
        _note_ends = time.ticks_add(time.ticks_ms(), length)
    else:
        _buzzer.duty_u16(0)


def quiet():
    """Stops any chime immediately."""
    _notes.clear()
    _buzzer.duty_u16(0)

# Robo ESP32 - MicroPython for Thonny
# Combined program:
#   - Joystick (D36 / D39) -> up / down / left / right inputs (Konami code)
#   - Morse button on D26  -> A and B, using a KNN classifier for dot vs dash
#   - Training button D35 (M2B) -> trains the classifier (5 short + 5 long presses)
#   - Alarm button D32 (external) -> buzzer on D23 sounds until the code is
#     entered correctly, or D32 is pressed again
#   - Correct code -> LED on D22 turns on and the DC motor (D12 / D13) runs
#   - Button D34 -> also runs the motor while held
#   - Encoder on D16 / D17 -> motor position in degrees

import neopixel
from machine import ADC, Pin, PWM
from time import ticks_ms, ticks_diff, sleep_ms

# =========================================================
# Joystick
# =========================================================
x = ADC(Pin(36))
y = ADC(Pin(39))
x.atten(ADC.ATTN_11DB)   # full 0-3.3 V input range
y.atten(ADC.ATTN_11DB)

x_min, x_max = 1280, 3160
y_min, y_max = 1360, 4095

led = Pin(22, Pin.OUT)
led.off()
lights = neopixel.NeoPixel(Pin(15), 2)

joystick_positions = ["center"]
inputs = []
konami = ["up", "up", "down", "down", "left", "right",
          "left", "right", "B", "A"]

def add_input(item):
    inputs.append(item)
    if len(inputs) > 10:
        del inputs[0]
    print(inputs, len(inputs))

# =========================================================
# Buzzer (D23) + alarm button (D32)
# =========================================================
BUZZER_FREQ = 2000          # Hz
BEEP_MS = 300               # beep on / off time while the alarm is active
buzzer = PWM(Pin(23), freq=BUZZER_FREQ, duty_u16=0)

alarm_btn = Pin(32, Pin.IN, Pin.PULL_UP)
ALARM_IDLE = alarm_btn.value()        # level when NOT pressed (read at startup)
alarm_prev = False
alarm_changed_at = 0
alarm_last_level = ALARM_IDLE                 # DEBUG: last raw level seen

alarm_active = False
beep_on = False
last_beep_toggle = 0

def buzzer_set(on):
    buzzer.duty_u16(32768 if on else 0)

def start_alarm():
    global alarm_active, last_beep_toggle, beep_on
    alarm_active = True
    inputs.clear()                    # code must be entered fresh
    beep_on = True
    buzzer_set(True)
    last_beep_toggle = ticks_ms()
    print("\n*** ALARM ON - enter the code to stop it ***")

def stop_alarm(reason):
    global alarm_active, beep_on
    alarm_active = False
    beep_on = False
    buzzer_set(False)
    print("*** ALARM OFF ({}) ***\n".format(reason))

# =========================================================
# Motor (D12 / D13) + run button (D34)
# =========================================================
MOTOR_SPEED = 60                      # percent, 0-100
motor_a = PWM(Pin(12), freq=1000, duty_u16=0)
motor_b = PWM(Pin(13), freq=1000, duty_u16=0)
motor_btn = Pin(34, Pin.IN)           # input-only pin; board supplies the pull-up

def motor_run(speed_percent):
    duty = int(min(abs(speed_percent), 100) * 65535 / 100)
    if speed_percent > 0:
        motor_a.duty_u16(duty); motor_b.duty_u16(0)
    elif speed_percent < 0:
        motor_a.duty_u16(0); motor_b.duty_u16(duty)
    else:
        motor_a.duty_u16(0); motor_b.duty_u16(0)

# =========================================================
# Encoder (D16 / D17)
# =========================================================
COUNTS_PER_REV = 1320                 # set from your calibration
enc_a = Pin(16, Pin.IN, Pin.PULL_UP)
enc_b = Pin(17, Pin.IN, Pin.PULL_UP)
encoder_count = 0
_last_enc = (enc_a.value() << 1) | enc_b.value()
_QUAD = (0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0)

def _encoder_irq(pin):
    global encoder_count, _last_enc
    new = (enc_a.value() << 1) | enc_b.value()
    encoder_count += _QUAD[(_last_enc << 2) | new]
    _last_enc = new

enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_encoder_irq)
enc_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_encoder_irq)

# =========================================================
# Morse button (D26) + training button (D35)
# =========================================================
morse_btn = Pin(26, Pin.IN, Pin.PULL_UP)
MORSE_IDLE = morse_btn.value()        # level when NOT pressed (read at startup)
train_btn = Pin(35, Pin.IN)           # M2B button; board supplies the pull-up

DEBOUNCE_MS = 20
LETTER_GAP_MS = 1000                  # silence that ends a letter
TRAIN_REPS = 5
K = 3

MORSE_TABLE = {
    ".-": "A",
    "-...": "B",
}

# KNN training data: list of (duration_ms, label) with label "." or "-"
samples = []
training = False
train_label = "."

def start_training():
    global training, train_label, samples
    samples = []
    training = True
    train_label = "."
    print("\n--- TRAINING ---")
    print("Press the Morse button SHORT {} times.".format(TRAIN_REPS))

def add_training_sample(duration):
    global training, train_label
    samples.append((duration, train_label))
    count = sum(1 for _, lab in samples if lab == train_label)
    kind = "SHORT" if train_label == "." else "LONG"
    print("  {} press {}/{}: {} ms".format(kind, count, TRAIN_REPS, duration))
    if count >= TRAIN_REPS:
        if train_label == ".":
            train_label = "-"
            print("Now press the Morse button LONG {} times.".format(TRAIN_REPS))
        else:
            training = False
            shorts = [d for d, lab in samples if lab == "."]
            longs = [d for d, lab in samples if lab == "-"]
            print("Training done. Avg short = {} ms, avg long = {} ms".format(
                sum(shorts) // len(shorts), sum(longs) // len(longs)))
            print("Morse: A = .-   B = -...\n")

def knn_classify(duration):
    nearest = sorted(samples, key=lambda s: abs(s[0] - duration))[:K]
    dots = sum(1 for _, lab in nearest if lab == ".")
    return "." if dots > len(nearest) - dots else "-"

symbols = ""

def decode_letter():
    global symbols
    letter = MORSE_TABLE.get(symbols)
    if letter:
        print("Morse '{}' -> {}".format(symbols, letter))
        add_input(letter)
    else:
        print("Morse '{}' -> not A or B, ignored".format(symbols))
    symbols = ""

def handle_morse_press(duration):
    global symbols
    if training:
        add_training_sample(duration)
    elif not samples:
        print("Not trained yet - press D35 (M2B) to train.")
    else:
        sym = knn_classify(duration)
        symbols += sym
        print("  {} ms -> {}   ({})".format(duration, "dot" if sym == "." else "dash", symbols))

# Debounce state
morse_raw = morse_stable = False
morse_changed_at = 0
press_start = 0
last_release = 0

train_prev = train_btn.value()
train_changed_at = 0

# =========================================================
# Main loop
# =========================================================
state = 1          # 0 = blinking plane lights, 1 = joystick mode
last_update_time = 0
blink_interval = 1000
blink_on = True

motor_on = False
code_ok_prev = False
last_count_printed = None
last_deg_print = 0

print("Ready. D32 = alarm, D35 = train Morse, D26 = Morse key, D34 = run motor.")
print("Idle levels: Morse button = {}, alarm button = {}".format(MORSE_IDLE, ALARM_IDLE))

# DEBUG: short test beep so you know the buzzer works
print("[DEBUG] Test beep on D23...")
buzzer_set(True)
sleep_ms(200)
buzzer_set(False)
print("[DEBUG] If you heard nothing, check the buzzer mute switch.")
print(joystick_positions[-1])

try:
    while True:
        now = ticks_ms()

        # ---------- Alarm button (D32) ----------
        alarm_level = alarm_btn.value()
        if alarm_level != alarm_last_level:          # DEBUG: every raw change
            print("[DEBUG] D32 raw level changed to {} (idle is {})".format(
                alarm_level, ALARM_IDLE))
            alarm_last_level = alarm_level
        a = alarm_level != ALARM_IDLE
        if a != alarm_prev and ticks_diff(now, alarm_changed_at) > 200:
            alarm_changed_at = now
            alarm_prev = a
            print("[DEBUG] D32 debounced: {}".format("PRESSED" if a else "released"))
            if a:                                   # just pressed
                if alarm_active:
                    stop_alarm("button pressed again")
                else:
                    start_alarm()

        # Beeping while the alarm is active
        if alarm_active and ticks_diff(now, last_beep_toggle) >= BEEP_MS:
            last_beep_toggle = now
            beep_on = not beep_on
            buzzer_set(beep_on)

        # ---------- Training button (D35, active low) ----------
        t = train_btn.value()
        if t != train_prev and ticks_diff(now, train_changed_at) > 200:
            train_changed_at = now
            train_prev = t
            if t == 0:
                symbols = ""
                start_training()

        # ---------- Morse button (D26) ----------
        raw = morse_btn.value() != MORSE_IDLE
        if raw != morse_raw:
            morse_raw = raw
            morse_changed_at = now
        if ticks_diff(now, morse_changed_at) > DEBOUNCE_MS and morse_raw != morse_stable:
            morse_stable = morse_raw
            if morse_stable:
                press_start = now
            else:
                last_release = now
                handle_morse_press(ticks_diff(now, press_start))

        if symbols and not morse_stable and ticks_diff(now, last_release) > LETTER_GAP_MS:
            decode_letter()

        # ---------- Joystick / lights ----------
        x_voltage = x.read()
        y_voltage = y.read()
        x_norm = (x_voltage - x_min) / (x_max - x_min)
        y_norm = (y_voltage - y_min) / (y_max - y_min)
        stick_pressed = x_voltage > 4000   # stick press drives X to max

        if state == 0:
            if ticks_diff(now, last_update_time) >= blink_interval:
                last_update_time = now
                blink_on = not blink_on
                if blink_on:
                    lights[0] = (0, 20, 2); lights[1] = (0, 0, 0)
                else:
                    lights[0] = (0, 0, 0); lights[1] = (20, 0, 0)
                lights.write()
        else:
            if x_norm > 0.95 and joystick_positions[-1] != "up":
                joystick_positions.append("up")
            elif x_norm < 0.05 and joystick_positions[-1] != "down":
                if not stick_pressed:
                    joystick_positions.append("down")
            elif y_norm < 0.05 and joystick_positions[-1] != "left":
                joystick_positions.append("left")
            elif y_norm > 0.95 and joystick_positions[-1] != "right":
                joystick_positions.append("right")
            elif abs(x_norm - 0.365) < 0.03 and abs(y_norm - 0.215) < 0.03:
                if joystick_positions[-1] != "center":
                    add_input(joystick_positions[-1])
                    joystick_positions.append("center")
            if len(joystick_positions) > 20:
                del joystick_positions[:-1]

        # ---------- Code check ----------
        code_ok = inputs == konami
        if code_ok and not code_ok_prev:
            print(">>> CODE CORRECT <<<")
            if alarm_active:
                stop_alarm("code entered")
        elif not code_ok and code_ok_prev:
            print(">>> code no longer matches - LED and motor off <<<")
        code_ok_prev = code_ok
        led.value(1 if code_ok else 0)

        # ---------- Motor: runs on correct code, or while D34 is held ----------
        want_motor = code_ok or motor_btn.value() == 0
        if want_motor and not motor_on:
            motor_run(MOTOR_SPEED); motor_on = True
            print("Motor ON")
        elif not want_motor and motor_on:
            motor_run(0); motor_on = False
            print("Motor OFF at {:.1f} deg".format(encoder_count * 360.0 / COUNTS_PER_REV))

        # Encoder report, at most 5x per second
        count = encoder_count
        if count != last_count_printed and ticks_diff(now, last_deg_print) >= 200:
            print("Position: {:9.1f} deg".format(count * 360.0 / COUNTS_PER_REV))
            last_count_printed = count
            last_deg_print = now

        sleep_ms(1)
except KeyboardInterrupt:
    motor_run(0)
    buzzer_set(False)
    led.off()
    print("Stopped.")
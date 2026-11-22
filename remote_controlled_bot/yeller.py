# yeller.py – Yeller (peripheral)
# Import this from main.py and call run(), or run this file directly in Thonny.

from BLE_CEEO import Yell
from machine import ADC, Pin, Timer, PWM
import time
import json
import gc

NAME = "Tommy"   # make this unique in a crowded room, and match it on the listener

# Pins
X_PIN, Y_PIN = 36, 39
BUZZER_PIN = 23

# Calibration range for your joystick
X_MIN, X_MAX = 800, 3000
Y_MIN, Y_MAX = 800, 3000

SAMPLE_PERIOD_MS = 100   # 10 readings per second
LOUD = 32768             # 50% duty in the 0–65535 range

# Sending tuning
CHANGE_THRESHOLD = 0.01  # only send when the stick moves at least this much
KEEPALIVE_MS = 300       # resend even if unchanged, so the listener knows we're alive
MIN_SEND_GAP_MS = 60     # never send faster than this (2 BLE connection intervals)
BACKOFF_MS = 150         # pause sending this long after the radio reports a full buffer
DEAD_LINK_MS = 3000      # if nothing gets through for this long, reconnect

# Songs: (frequency in Hz, length in ms), 0 = rest
VICTORY_SONG = [
    (392, 120), (523, 120), (659, 120), (784, 300),
    (0, 60),
    (659, 120), (784, 450),
    (0, 120),
    (523, 100), (587, 100), (659, 100), (698, 100),
    (784, 100), (880, 100), (988, 100), (1047, 600),
]
STARTUP_CHIRP = [(1047, 80), (0, 40), (1319, 80)]   # program started
CONNECTED_CHIRP = [(784, 80), (1047, 120)]          # listener connected
ERROR_BEEP = [(220, 300)]                           # something went wrong

# Shared state (hardware is created in run(), not on import)
x = None
y = None
buzzer = None
song_timer = None
buffer = b""
latest = (0.0, 0.0)      # (x_norm, y_norm), replaced as one tuple so x and y always match
new_sample = False
finished = False         # set by the BLE callback when "finish" arrives

# Background song player state
song = []
song_index = 0
note_on = False


def _song_step(timer):
    # Plays one note (or the short gap after it), then schedules the next step.
    # Runs from a timer, so the main loop keeps sending joystick data during songs.
    global song_index, note_on
    try:
        if note_on:
            # End of a note: short silent gap so repeated notes sound separate
            freq, length = song[song_index - 1]
            buzzer.duty_u16(0)
            note_on = False
            song_timer.init(mode=Timer.ONE_SHOT, period=max(1, length // 10), callback=_song_step)
            return

        if song_index >= len(song):
            buzzer.duty_u16(0)
            return

        freq, length = song[song_index]
        song_index += 1
        if freq == 0:
            buzzer.duty_u16(0)
            song_timer.init(mode=Timer.ONE_SHOT, period=length, callback=_song_step)
        else:
            buzzer.freq(freq)
            buzzer.duty_u16(LOUD)
            note_on = True
            song_timer.init(mode=Timer.ONE_SHOT, period=max(1, length * 9 // 10), callback=_song_step)
    except Exception:
        buzzer.duty_u16(0)


def play_song(new_song):
    # Starts a song in the background and returns immediately
    global song, song_index, note_on
    song_timer.deinit()
    buzzer.duty_u16(0)
    song = new_song
    song_index = 0
    note_on = False
    song_timer.init(mode=Timer.ONE_SHOT, period=1, callback=_song_step)


def clamp(v):
    # Keep values in 0–1 even if the stick reads past the calibration range
    return max(0.0, min(1.0, v))


def sample(timer):
    # Runs on every timer interrupt: read, normalize, round to 3 decimals
    global latest, new_sample
    x_norm = round(clamp((x.read() - X_MIN) / (X_MAX - X_MIN)), 3)
    y_norm = round(clamp((y.read() - Y_MIN) / (Y_MAX - Y_MIN)), 3)
    latest = (x_norm, y_norm)
    new_sample = True


def callback(data):
    # Only set a flag here. Never block inside the BLE callback.
    global buffer, finished
    buffer += data
    while b"\n" in buffer:
        line, buffer = buffer.split(b"\n", 1)
        text = line.decode().strip()

        # Accept both plain text (finish) and JSON ("finish")
        try:
            msg = json.loads(text)
        except ValueError:
            msg = text

        if isinstance(msg, str) and msg.strip().lower() == "finish":
            finished = True
            print("Finish received")
        else:
            print("Received:", msg)


def changed(a, b):
    return b is None or abs(a[0] - b[0]) >= CHANGE_THRESHOLD or abs(a[1] - b[1]) >= CHANGE_THRESHOLD


def check_finished():
    global finished
    if finished:
        finished = False
        print("Success! Playing victory song")
        play_song(VICTORY_SONG)


def run():
    global x, y, buzzer, song_timer, buffer, new_sample

    # Hardware setup
    x = ADC(Pin(X_PIN))
    y = ADC(Pin(Y_PIN))
    x.atten(ADC.ATTN_11DB)   # full 0–3.3V input range
    y.atten(ADC.ATTN_11DB)
    buzzer = PWM(Pin(BUZZER_PIN), freq=1000, duty_u16=0)
    song_timer = Timer(1)

    sampler = Timer(0)
    sampler.init(mode=Timer.PERIODIC, period=SAMPLE_PERIOD_MS, callback=sample)

    play_song(STARTUP_CHIRP)

    try:
        while True:   # reconnect forever
            p = None
            gc.collect()   # free leftovers from the previous connection before starting a new one
            try:
                p = Yell(NAME, interval_us=30000, verbose=False)
                p.callback = callback

                print("Waiting for a listener...")
                if p.connect_up():
                    print("Connected")
                    buffer = b""
                    play_song(CONNECTED_CHIRP)

                    last_sent = None
                    last_ok = time.ticks_ms()        # last time a packet got through
                    last_attempt = time.ticks_ms()   # last time we tried to send
                    backoff_until = time.ticks_ms()
                    dropped = 0

                    while p.is_connected:
                        check_finished()
                        now = time.ticks_ms()

                        ready = (new_sample
                                 and time.ticks_diff(now, backoff_until) >= 0
                                 and time.ticks_diff(now, last_attempt) >= MIN_SEND_GAP_MS)

                        if ready:
                            current = latest   # always the newest reading, never a stale retry
                            if changed(current, last_sent) or time.ticks_diff(now, last_ok) > KEEPALIVE_MS:
                                new_sample = False
                                last_attempt = now
                                # One 12-byte packet, e.g. "0.512,0.348\n"
                                payload = "%.3f,%.3f\n" % current
                                try:
                                    p.send(payload)
                                    last_sent = current
                                    last_ok = now
                                    if dropped:
                                        print("Radio recovered after", dropped, "skipped sends")
                                        dropped = 0
                                except OSError as e:
                                    if e.args[0] != 12:   # anything other than ENOMEM is a real error
                                        raise
                                    dropped += 1
                                    backoff_until = time.ticks_add(now, BACKOFF_MS)

                        # Listener stopped taking packets, so start over instead of spamming
                        if time.ticks_diff(now, last_ok) > DEAD_LINK_MS:
                            print("Link stalled, reconnecting")
                            break

                        time.sleep_ms(5)
                print("Lost connection")
                check_finished()   # still celebrate if "finish" came right before a disconnect
            except Exception as e:
                print("Error:", e)
                play_song(ERROR_BEEP)
            finally:
                if p:
                    p.disconnect()
            time.sleep(1)
    finally:
        sampler.deinit()
        song_timer.deinit()
        buzzer.deinit()


# Lets you still run this file directly from Thonny for testing
if __name__ == "__main__":
    run()

# calibrate_color_sensors.py
# Guided calibration for the color sensor.
#
# Collects samples of white paper, the edge of the black tape (gray),
# and each tape color, then saves them to color_samples.py on the ESP32
# for knn_color_classifier.py. Finishes with a live test that prints
# the color the sensor detects.

from line_follower_external_hardware import make_sensor
from knn_color_classifier import features, knn
import time

SAMPLES_PER_COLOR = 20         # readings per color
SAMPLE_GAP_MS = 100            # 20 x 100 ms = 2 seconds per color

# (label, instruction)
STEPS = (
    ("white", "over WHITE paper"),
    ("black", "over the middle of the BLACK tape"),
    ("gray",  "on the EDGE of the BLACK tape (half on, half off), and slide it along the edge"),
    ("red",   "over the RED tape"),
    ("green", "over the GREEN tape"),
    ("blue",  "over the BLUE tape"),
)


def collect(sensor):
    readings = []
    for _ in range(SAMPLES_PER_COLOR):
        readings.append(sensor.rgbc())
        time.sleep_ms(SAMPLE_GAP_MS)
    return readings


def average_clear(readings):
    return sum(r[3] for r in readings) // len(readings)


sensor = make_sensor()
print("Sensor found.")
print("While each set of readings is taken (2 seconds), slowly slide the")
print("sensor around inside the area, staying away from edges unless asked.\n")

raw = {}
for label, where in STEPS:
    input("Place the sensor %s, then press Enter..." % where)
    raw[label] = collect(sensor)
    print("   done\n")

white_c = average_clear(raw["white"])
black_c = average_clear(raw["black"])
if white_c - black_c < 100:
    print("WARNING: sensor barely tells black from white.")
    print("         Check its height above the floor and its LED.\n")

samples = []
for label, readings in raw.items():
    for r, g, b, c in readings:
        f4 = features(r, g, b, c, white_c)
        samples.append((round(f4[0], 3), round(f4[1], 3),
                        round(f4[2], 3), round(f4[3], 3), label))

with open("color_samples.py", "w") as f:
    f.write("# Written by calibrate_color_sensors.py\n")
    f.write("WHITE_C = %d\n" % white_c)
    f.write("SAMPLES = [\n")
    for s in samples:
        f.write("    %r,\n" % (s,))
    f.write("]\n")

print("Saved color_samples.py on the ESP32.\n")

input("Press Enter for a live test (press Stop in Thonny to end)...")
while True:
    print("sees:", knn(features(*sensor.rgbc(), white_c), samples))
    time.sleep_ms(200)
    

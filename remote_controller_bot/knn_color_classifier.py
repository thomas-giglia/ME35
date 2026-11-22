# knn_color_classifier.py
# Color training AND k-nearest-neighbors color detection in one file.
#
# Training:  run this file directly in Thonny. It walks you through holding the
#            sensor over each color, takes 2 seconds of readings per color, and
#            saves them to color_samples.py on the ESP32.
# Detecting: other files use ColorClassifier().classify(sensor.rgbc()),
#            with the sensor from make_sensor().

from machine import Pin, I2C
import time

K = 5                          # number of neighbors that vote

# Colors to train, in order. White must come first (it sets the brightness scale).
COLORS = ["white", "black", "gray", "green", "blue", "red"]
SAMPLE_SECONDS = 2             # how long to read each color
SAMPLE_PERIOD_MS = 50          # time between readings while training
SAMPLES_FILE = "color_samples.py"

# Sensor wiring and settings (used for both training and detecting)
SDA_PIN, SCL_PIN = 21, 22
INTEGRATION_MS = 40            # 40, 80, 160, 320, 640 or 1280


# ---------------- Sensor ----------------

class VEML6040:
    """Minimal driver for the Seeed Grove VEML6040 color sensor (I2C address 0x10)."""
    ADDR = 0x10
    IT_CODES = {40: 0, 80: 1, 160: 2, 320: 3, 640: 4, 1280: 5}

    def __init__(self, i2c, integration_ms=INTEGRATION_MS):
        self.i2c = i2c
        if self.ADDR not in i2c.scan():
            raise OSError("VEML6040 not found at 0x10")
        config = self.IT_CODES[integration_ms] << 4   # auto mode, sensor enabled
        self.i2c.writeto_mem(self.ADDR, 0x00, bytes([config, 0x00]))
        time.sleep_ms(integration_ms + 10)            # let the first reading finish

    def _read16(self, reg):
        d = self.i2c.readfrom_mem(self.ADDR, reg, 2)
        return d[0] | d[1] << 8

    def rgbc(self):
        """Returns (red, green, blue, white)."""
        return (self._read16(0x08), self._read16(0x09),
                self._read16(0x0A), self._read16(0x0B))


def make_sensor():
    """Creates the color sensor with the shared pins and settings."""
    return VEML6040(I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=400000))


# ---------------- KNN ----------------

def features(r, g, b, c, white_c):
    """Turns a raw reading into 4 numbers that compare well between colors.

    The first three are the color's proportions of red, green and blue
    (so a dim red and a bright red look alike). The fourth is brightness
    relative to white paper, which separates black from the colors.
    """
    if c == 0:
        return (0.0, 0.0, 0.0, 0.0)
    return (r / c, g / c, b / c, c / white_c)


def knn(f, samples, k=K):
    """Returns the most common label among the k closest samples."""
    nearest = []
    for s in samples:
        d = ((f[0] - s[0]) ** 2 + (f[1] - s[1]) ** 2 +
             (f[2] - s[2]) ** 2 + (f[3] - s[3]) ** 2)
        nearest.append((d, s[4]))
    nearest.sort()

    votes = {}
    for _, label in nearest[:k]:
        votes[label] = votes.get(label, 0) + 1
    return max(votes, key=votes.get)


class ColorClassifier:
    """Loads the saved samples. Use .classify(reading), where reading is
    the (r, g, b, c) tuple from sensor.rgbc()."""

    def __init__(self):
        try:
            import color_samples as cs
        except ImportError:
            raise SystemExit("No color samples - run knn_color_classifier.py first to train.")
        self.samples = cs.SAMPLES
        self.white_c = cs.WHITE_C

    def classify(self, reading):
        return knn(features(*reading, self.white_c), self.samples)


# ---------------- Training ----------------

def collect(sensor, seconds=SAMPLE_SECONDS):
    """Reads the sensor for the given time and returns the raw readings."""
    readings = []
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < seconds * 1000:
        readings.append(sensor.rgbc())
        time.sleep_ms(SAMPLE_PERIOD_MS)
    return readings


def train(sensor=None):
    """Walks through each color, then saves every reading as a KNN sample."""
    if sensor is None:
        sensor = make_sensor()

    raw = {}
    for color in COLORS:
        input("Place the sensor over %s and press Enter..." % color.upper())
        raw[color] = collect(sensor)
        print("  got %d readings" % len(raw[color]))

    # White's average brightness becomes the scale for every sample
    whites = raw["white"]
    white_c = sum(r[3] for r in whites) / len(whites)
    if white_c == 0:
        raise SystemExit("White reading was 0. Check the sensor wiring and try again.")

    samples = []
    for color, readings in raw.items():
        for reading in readings:
            f = features(*reading, white_c)
            samples.append((f[0], f[1], f[2], f[3], color))

    save(samples, white_c)
    print("Saved %d samples to %s" % (len(samples), SAMPLES_FILE))
    return samples, white_c


def save(samples, white_c):
    """Writes the samples as a small Python file that ColorClassifier imports."""
    with open(SAMPLES_FILE, "w") as f:
        f.write("# Made by knn_color_classifier.py. Retrain instead of editing by hand.\n")
        f.write("WHITE_C = %.2f\n" % white_c)
        f.write("SAMPLES = [\n")
        for s in samples:
            f.write("    (%.4f, %.4f, %.4f, %.4f, %r),\n" % s)
        f.write("]\n")


def test(seconds=10):
    """After training, prints the detected color twice a second to check it."""
    sensor = make_sensor()
    classifier = ColorClassifier()
    end = time.ticks_add(time.ticks_ms(), seconds * 1000)
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        print(classifier.classify(sensor.rgbc()))
        time.sleep_ms(500)


# Running this file directly trains the colors, then lets you check them
if __name__ == "__main__":
    train()
    print("Move the sensor around to check the colors:")
    test()
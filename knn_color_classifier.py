# knn_color_classifier.py
# K-nearest-neighbors color detection using samples saved by
# calibrate_color_sensors.py (stored in color_samples.py on the ESP32).

K = 5                          # number of neighbors that vote


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
            raise SystemExit("No color samples - run calibrate_color_sensors.py first.")
        self.samples = cs.SAMPLES
        self.white_c = cs.WHITE_C

    def classify(self, reading):
        return knn(features(*reading, self.white_c), self.samples)
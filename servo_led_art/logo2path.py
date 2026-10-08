"""
Turn a logo into strokes.json for strokes2angles.py.

    .svg            -> reads the shapes directly (exact lines and curves)
    .png/.jpg/...   -> groups the pixels into a few flat colors with OpenCV and
                       traces the edges where the colors meet

Either way, edges shared by two shapes are only kept once, so the robot
doesn't trace the same line twice (which would show up brighter in the photo).

Setup:
    pip install opencv-python numpy svgelements

Put the logo next to this script, set INPUT_FILE, and run it. Then run
strokes2angles.py in the same folder.

Outputs:
    strokes.json  - every line as a list of (x, y) points
    outline.png   - preview of the lines that will be drawn
    colors.png    - (image files only) the flat colors the logo was split into;
                    if two colors merged or one got split, change N_COLORS
"""

import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).parent
_T0 = time.time()


def log(msg):
    print(f"[{time.time() - _T0:6.1f}s] {msg}", flush=True)

# ======================= Settings =======================
INPUT_FILE = "jumbo.svg"  # .svg, .png, .jpg, ...

WORK_SIZE = 2000          # the logo is scaled so its longest side is this many units
DEDUP_TOLERANCE = 4       # lines closer than this (in WORK_SIZE units) count as the same line
MIN_STROKE_LENGTH = 15    # drop strokes shorter than this (WORK_SIZE units)
DROP_FRAME = True         # drop a background rectangle around the whole logo

# Image files only:
N_COLORS = 4              # number of flat colors in the logo, INCLUDING the background
SKIP_BACKGROUND = True    # don't outline the background color itself
MIN_REGION_AREA = 200     # ignore color specks smaller than this (pixels at WORK_SIZE)
# ========================================================


# ---------------- SVG ----------------

def svg_strokes(path, step=1.0):
    """Sample every shape in the SVG. Returns a list of (name, points, closed)
    in SVG units; spacing is refined after scaling."""
    from svgelements import SVG, Shape, Path as SvgPath, Move, Close

    log("Parsing SVG...")
    svg = SVG.parse(str(path))  # applies transforms, converts rects/circles/etc.
    shapes = [el for el in svg.elements() if isinstance(el, Shape)]
    log(f"Sampling {len(shapes)} shapes...")
    raw = []
    for el in shapes:
        if not isinstance(el, Shape):
            continue
        p = SvgPath(el)
        if len(p) == 0:
            continue
        name = el.id or type(el).__name__.lower()
        for sub in p.as_subpaths():
            sub = SvgPath(sub)
            pieces, closed = [], False
            for seg in sub:
                if isinstance(seg, Move):
                    continue
                if isinstance(seg, Close):
                    closed = True
                if seg.start is None:
                    continue
                try:
                    length = seg.length(error=1e-4)  # default precision is very slow
                except TypeError:
                    length = seg.length()
                if not length:
                    continue
                pieces.append((seg, length))
            if not pieces:
                continue
            total = sum(L for _, L in pieces)
            pts = []
            for seg, L in pieces:
                # Points per segment proportional to its length (at least 2).
                n = max(2, int(math.ceil(L / total * 4000)))
                pts.append(seg.npoint(np.linspace(0, 1, n))[:-1])
            last = pieces[-1][0].end
            pts.append(np.array([[last.x, last.y]]))
            pts = np.vstack(pts).astype(float)
            if np.linalg.norm(pts[0] - pts[-1]) < 1e-6 * max(1, total):
                closed = True
            if closed and np.linalg.norm(pts[0] - pts[-1]) < 1e-9 * max(1, total):
                pts = pts[:-1]
            if len(pts) >= 2:
                raw.append((name, pts, closed))
    if not raw:
        raise SystemExit("No shapes found in the SVG.")

    # Scale so the longest side is WORK_SIZE, with a small border.
    allpts = np.vstack([p for _, p, _ in raw])
    lo, hi = allpts.min(axis=0), allpts.max(axis=0)
    pad = 10
    scale = (WORK_SIZE - 2 * pad) / max(hi - lo)
    strokes = [(n, (p - lo) * scale + pad, c) for n, p, c in raw]
    size = (int(math.ceil((hi - lo)[0] * scale)) + 2 * pad,
            int(math.ceil((hi - lo)[1] * scale)) + 2 * pad)
    return strokes, size, None


# ---------------- Image ----------------

def image_strokes(path):
    log("Reading image...")
    img = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(f"Couldn't read {path}")
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if img.shape[2] == 4:  # transparent background -> white
        alpha = img[:, :, 3:4].astype(float) / 255
        img = (img[:, :, :3] * alpha + 255 * (1 - alpha)).astype(np.uint8)

    h, w = img.shape[:2]
    s = WORK_SIZE / max(h, w)
    img = cv2.resize(img, (round(w * s), round(h * s)),
                     interpolation=cv2.INTER_CUBIC if s > 1 else cv2.INTER_AREA)
    img = cv2.medianBlur(img, 3)
    h, w = img.shape[:2]

    log(f"Grouping pixels into {N_COLORS} colors...")
    # k-means on a sample of pixels, then assign every pixel to its nearest color.
    pixels = img.reshape(-1, 3).astype(np.float32)
    rng = np.random.default_rng(0)
    sample = pixels[rng.choice(len(pixels), min(60000, len(pixels)), replace=False)]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.5)
    _, _, centers = cv2.kmeans(sample, N_COLORS, None, criteria, 3, cv2.KMEANS_PP_CENTERS)
    labels = np.zeros(len(pixels), np.int32)
    best = np.full(len(pixels), np.inf, np.float32)
    for k, c in enumerate(centers):
        d = ((pixels - c) ** 2).sum(axis=1)
        better = d < best
        labels[better], best[better] = k, d[better]
    labels = labels.reshape(h, w)

    border = np.concatenate([labels[0], labels[-1], labels[:, 0], labels[:, -1]])
    background = int(np.bincount(border, minlength=N_COLORS).argmax())

    log("Tracing color edges...")
    kernel = np.ones((3, 3), np.uint8)
    strokes = []
    for k in range(N_COLORS):
        if SKIP_BACKGROUND and k == background:
            continue
        mask = (labels == k).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # remove specks
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # fill pinholes
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        for c in contours:
            if cv2.contourArea(c) >= MIN_REGION_AREA:
                strokes.append((f"color_{k}", c.reshape(-1, 2).astype(float), True))

    preview = centers[labels].astype(np.uint8)  # what the color grouping saw
    return strokes, (w, h), preview


# ---------------- Clean-up shared by both ----------------

def densify(pts, closed, step=1.0):
    if closed:
        pts = np.vstack([pts, pts[:1]])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    if s[-1] == 0:
        return pts[:1]
    d = np.linspace(0, s[-1], max(2, int(math.ceil(s[-1] / step)) + 1))
    out = np.column_stack([np.interp(d, s, pts[:, 0]), np.interp(d, s, pts[:, 1])])
    return out[:-1] if closed else out


def drop_frame(strokes):
    """Remove a closed stroke that just traces the rectangle around everything."""
    allpts = np.vstack([p for _, p, _ in strokes])
    x0, y0 = allpts.min(axis=0)
    x1, y1 = allpts.max(axis=0)
    tol = DEDUP_TOLERANCE
    kept = []
    for name, p, closed in strokes:
        on_edge = ((np.abs(p[:, 0] - x0) < tol) | (np.abs(p[:, 0] - x1) < tol) |
                   (np.abs(p[:, 1] - y0) < tol) | (np.abs(p[:, 1] - y1) < tol))
        bx0, by0 = p.min(axis=0)
        bx1, by1 = p.max(axis=0)
        full = (bx0 - x0 < tol and by0 - y0 < tol and x1 - bx1 < tol and y1 - by1 < tol)
        if closed and full and on_edge.all() and len(strokes) > 1:
            continue
        kept.append((name, p, closed))
    return kept


def dedup(strokes, size):
    """Keep each edge once: longer strokes go first, and any part of a later
    stroke that runs along an already-kept line is cut out."""
    w, h = size
    drawn = np.zeros((h + 2, w + 2), np.uint8)
    thick = 2 * DEDUP_TOLERANCE + 1
    out = []

    def length(p, closed):
        q = np.vstack([p, p[:1]]) if closed else p
        return np.linalg.norm(np.diff(q, axis=0), axis=1).sum()

    for name, pts, closed in sorted(strokes, key=lambda s: -length(s[1], s[2])):
        dense = densify(pts, closed)
        xi = np.clip(np.round(dense[:, 0]).astype(int), 0, w + 1)
        yi = np.clip(np.round(dense[:, 1]).astype(int), 0, h + 1)
        free = drawn[yi, xi] == 0

        if free.all():
            pieces = [(dense, closed)]
        else:
            if closed:  # start at a used point so free runs don't wrap around
                j = int(np.argmax(~free))
                dense, free = np.roll(dense, -j, axis=0), np.roll(free, -j)
            # Find runs of free points; extend each run into the cut by the
            # tolerance so it meets the line it was cut against (no gap).
            edges = np.diff(np.concatenate([[0], free.astype(np.int8), [0]]))
            starts, ends = np.nonzero(edges == 1)[0], np.nonzero(edges == -1)[0]
            g = DEDUP_TOLERANCE
            pieces = [(dense[max(a - g, 0):min(b + g, len(dense))], False)
                      for a, b in zip(starts, ends)]

        for p, c in pieces:
            if len(p) >= 2 and length(p, c) >= MIN_STROKE_LENGTH:
                out.append((name, p, c))
        for p, c in pieces:
            poly = np.round(p).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(drawn, [poly], c, 1, thick)
    return out


def main():
    path = HERE / INPUT_FILE
    if not path.exists():
        raise FileNotFoundError(f"Couldn't find {path}")

    if path.suffix.lower() == ".svg":
        strokes, size, preview = svg_strokes(path)
    else:
        strokes, size, preview = image_strokes(path)

    log(f"Found {len(strokes)} outlines; removing duplicate edges...")
    if DROP_FRAME:
        strokes = drop_frame(strokes)
    strokes = dedup(strokes, size)
    if not strokes:
        raise SystemExit("No strokes left; try lowering MIN_STROKE_LENGTH or MIN_REGION_AREA.")

    w, h = size
    data = {"image_width": w, "image_height": h, "strokes": [
        {"name": n, "closed": bool(c), "points": np.round(p, 2).tolist()}
        for n, p, c in strokes
    ]}
    with open(HERE / "strokes.json", "w") as f:
        json.dump(data, f)

    outline = np.full((h, w, 3), 255, np.uint8)
    for _, p, c in strokes:
        poly = np.round(p).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(outline, [poly], c, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.imwrite(str(HERE / "outline.png"), outline)
    if preview is not None:
        cv2.imwrite(str(HERE / "colors.png"), preview)

    log(f"Saved strokes.json ({len(strokes)} strokes) and outline.png"
          + (" and colors.png" if preview is not None else ""))


if __name__ == "__main__":
    main()
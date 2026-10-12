"""
servo_led_art.py - turn an SVG logo into servo pulse widths for a two-link
LED-drawing arm (long-exposure light painting).

Usage:
    1. Put this script and your logo (e.g. LogoName.svg) in PythonCode/servo_led_art/
    2. Run:  python servo_led_art.py
    3. Type the file name when asked, e.g.  LogoName.svg

Everything it makes goes into a subfolder named after the logo:
    servo_led_art/LogoName/
        strokes.json   - the logo's outlines as (x, y) points
        outline.png    - preview of the lines that will be drawn
        workspace.png  - the arm's reachable area (gray) with the drawing on top
                         (blue = drawable, red = out of reach)
        preview.png    - the drawing rebuilt from the pulse widths alone (final check)
        LogoName_esp32.csv - just light, pulse1_us, pulse2_us; the small file
                         to copy onto the ESP32 for draw.py
        angles.csv     - one row per point:
                         stroke, light, x_mm, y_mm, theta1_deg, theta2_deg,
                         servo1_deg, servo2_deg, pulse1_us, pulse2_us
                         light = 0: move to this point with the LED OFF (travel)
                         light = 1: move to this point with the LED ON (drawing)

Setup:
    pip install opencv-python numpy svgelements
"""

import csv
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


# ======================= Logo settings =======================
WORK_SIZE = 2000          # logo is scaled so its longest side is this many units
DEDUP_TOLERANCE = 4       # lines closer than this (WORK_SIZE units) count as the same line
MIN_LINE_LENGTH = 15      # drop outline pieces shorter than this (WORK_SIZE units)
DROP_FRAME = True         # drop a background rectangle around the whole logo

# ======================= Robot settings =======================
L1 = 150.0   # mm, shoulder servo -> elbow servo
L2 = 150.0   # mm, elbow servo -> LED

# Servo pulse widths (microseconds) and how far each servo actually turns
# between them. To measure the span: send PULSE_MIN, mark the arm's position,
# send PULSE_MAX, and measure the angle between the two positions.
SERVO1_PULSE = (400.0, 2600.0)   # shoulder: (us at 0 deg, us at SPAN deg)
SERVO2_PULSE = (400.0, 2600.0)   # elbow
SERVO1_SPAN_DEG = 180.0          # degrees turned from PULSE_MIN to PULSE_MAX
SERVO2_SPAN_DEG = 180.0

# Servo angles used below run from 0 (at PULSE_MIN) to SPAN (at PULSE_MAX).
SERVO1_RANGE = (0.0, SERVO1_SPAN_DEG)
SERVO2_RANGE = (0.0, SERVO2_SPAN_DEG)

# How each servo's reading relates to the arm's joint angle:
#   servo = OFFSET + DIRECTION * joint_angle
# Joint angles used here:
#   theta1 = angle of the first arm, measured from the +x axis (counterclockwise +)
#   theta2 = angle of the second arm relative to the first (0 = arm straight)
# Recommended mounting (gives by far the largest drawing area):
#   elbow servo 0 = second arm straight out, 180 = folded back over the first arm.
#   (Stack the second arm above the first so it can fold over the shoulder servo.)
# The shoulder offset is chosen automatically when AUTO_PLACE is True; the script
# prints which way the shoulder servo's 0 position should point.
# Use DIRECTION = -1 if a servo turns the opposite way when its value increases.
SERVO1_OFFSET, SERVO1_DIRECTION = 0.0, 1
SERVO2_OFFSET, SERVO2_DIRECTION = 0.0, 1

# Keep the LED at least this far from the shoulder axis, so it doesn't run into
# the shoulder servo, and because near the axis tiny moves need huge shoulder turns.
MIN_RADIUS = 30.0   # mm

# Which of the two arm poses to use for each point: "up", "down", or "auto"
# ("auto" picks whichever fits the servo limits, one choice per stroke so the
# arm never flips partway through a line).
ELBOW = "auto"

# ======================= Drawing placement =======================
# Robot frame: origin at the shoulder servo axis, +x to the right, +y up
# (as seen by the camera). The drawing is kept upright in this frame.
# AUTO_PLACE = True finds the largest drawing that fits (and the shoulder servo
# offset that allows it) and ignores the three values below.
AUTO_PLACE = True
MARGIN = 3.0                    # mm of safety margin from the edge of the reach
DRAWING_CENTER = (0.0, 150.0)   # mm, center of the drawing's lines
DRAWING_WIDTH = 100.0           # mm, width of the drawing's lines
DRAWING_ROTATION_DEG = 0.0

# ======================= Path settings =======================
POINT_SPACING = 0.5      # mm between output points (equal spacing = even brightness)
SMOOTH_WINDOW = 5        # moving-average window in points (1 = no smoothing)
MIN_STROKE_LENGTH = 2.0  # mm; shorter strokes are dropped
# ==============================================================

ELBOW_SIGNS = {"up": [-1], "down": [1], "auto": [-1, 1]}


# ======================= Logo -> strokes =======================

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
            if len(p) >= 2 and length(p, c) >= MIN_LINE_LENGTH:
                out.append((name, p, c))
        for p, c in pieces:
            poly = np.round(p).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(drawn, [poly], c, 1, thick)
    return out



def logo_to_strokes(svg_path, out_dir):
    strokes, size, _ = svg_strokes(svg_path)
    log(f"Found {len(strokes)} outlines; removing duplicate edges...")
    if DROP_FRAME:
        strokes = drop_frame(strokes)
    strokes = dedup(strokes, size)
    if not strokes:
        raise SystemExit("No lines left; try lowering MIN_LINE_LENGTH.")

    w, h = size
    raw = [{"name": n, "closed": bool(c), "points": np.round(p, 2).tolist()}
           for n, p, c in strokes]
    with open(out_dir / "strokes.json", "w") as f:
        json.dump({"image_width": w, "image_height": h, "strokes": raw}, f)

    outline = np.full((h, w, 3), 255, np.uint8)
    for _, p, c in strokes:
        poly = np.round(p).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(outline, [poly], c, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.imwrite(str(out_dir / "outline.png"), outline)
    log(f"Saved strokes.json ({len(raw)} strokes) and outline.png")
    return raw


# ======================= Strokes -> servo angles =======================

# ---------------- 1. Pixels -> workspace: the transformation matrix ----------------

def drawing_bounds(raw):
    """Bounding box (x0, y0, x1, y1) of all stroke points, in pixels."""
    allpts = np.vstack([np.array(s["points"], float) for s in raw])
    return (*allpts.min(axis=0), *allpts.max(axis=0))


def pixel_to_workspace_matrix(bounds, center, width, rotation_deg):
    """3x3 homogeneous matrix mapping (px, py, 1) -> (x_mm, y_mm, 1).
    Applied right to left: move the drawing's center to the origin -> scale to mm
    and flip y (images count y downward) -> rotate -> move to `center`."""
    x0, y0, x1, y1 = bounds
    s = width / (x1 - x0)
    a = math.radians(rotation_deg)
    cx, cy = center
    T_center = np.array([[1, 0, -(x0 + x1) / 2], [0, 1, -(y0 + y1) / 2], [0, 0, 1]])
    S_flip = np.array([[s, 0, 0], [0, -s, 0], [0, 0, 1]])
    R = np.array([[math.cos(a), -math.sin(a), 0],
                  [math.sin(a), math.cos(a), 0],
                  [0, 0, 1]])
    T_place = np.array([[1, 0, cx], [0, 1, cy], [0, 0, 1]])
    return T_place @ R @ S_flip @ T_center


def apply_matrix(M, pts):
    homog = np.column_stack([pts, np.ones(len(pts))])
    return (homog @ M.T)[:, :2]


# ---------------- 2. Polylines -> smooth functions x(t), y(t) ----------------

def smooth(pts, closed, window):
    if window <= 1 or len(pts) < window:
        return pts
    kernel = np.ones(window) / window
    pad = window // 2
    if closed:
        padded = np.vstack([pts[-pad:], pts, pts[:pad]])
    else:
        padded = np.vstack([np.repeat(pts[:1], pad, 0), pts, np.repeat(pts[-1:], pad, 0)])
    out = np.column_stack([np.convolve(padded[:, i], kernel, "valid") for i in (0, 1)])
    if not closed:
        out[0], out[-1] = pts[0], pts[-1]
    return out


def path_function(pts, closed):
    """Return (f, length): f(t) gives (x, y) for t in [0, 1], with t
    proportional to distance along the curve."""
    if closed:
        pts = np.vstack([pts, pts[:1]])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    keep = np.concatenate([[True], seg > 1e-9])
    pts = pts[keep]
    s = np.concatenate([[0], np.cumsum(seg[seg > 1e-9])])
    length = s[-1]

    def f(t):
        d = np.asarray(t) * length
        return np.column_stack([np.interp(d, s, pts[:, 0]), np.interp(d, s, pts[:, 1])])

    return f, length


def resample(f, length, spacing):
    n = max(2, int(math.ceil(length / spacing)) + 1)
    return f(np.linspace(0, 1, n))


# ---------------- 3. Stroke ordering ----------------

def order_strokes(strokes):
    """Greedy nearest-neighbour ordering; open strokes may be reversed and
    closed strokes may start at (roughly) whichever point is nearest.
    Vectorized: all candidate start points are checked in one numpy step."""
    if not strokes:
        return []
    cand_pts, cand_id, cand_idx = [], [], []
    for i, (_, pts, closed) in enumerate(strokes):
        if closed:
            idx = np.unique(np.linspace(0, len(pts) - 1, min(len(pts), 64)).astype(int))
        else:
            idx = np.array([0, len(pts) - 1])
        cand_pts.append(pts[idx])
        cand_id.append(np.full(len(idx), i))
        cand_idx.append(idx)
    cand_pts = np.vstack(cand_pts)
    cand_id = np.concatenate(cand_id)
    cand_idx = np.concatenate(cand_idx)
    active = np.ones(len(cand_pts), bool)

    ordered = []
    pos = strokes[0][1][0]
    for _ in range(len(strokes)):
        d = np.einsum("ij,ij->i", cand_pts - pos, cand_pts - pos)
        d[~active] = np.inf
        c = int(np.argmin(d))
        i = cand_id[c]
        name, pts, closed = strokes[i]
        if closed:
            # refine: exact nearest point on this loop
            j = int(np.argmin(np.einsum("ij,ij->i", pts - pos, pts - pos)))
            pts = np.roll(pts, -j, axis=0)
            pos = pts[0]
        else:
            if cand_idx[c] != 0:
                pts = pts[::-1]
            pos = pts[-1]
        ordered.append((name, pts, closed))
        active[cand_id == i] = False
    return ordered


# ---------------- 4. Kinematics and servo limits ----------------

def ik(xy, elbow_sign):
    """Two-link inverse kinematics. Returns (theta1, theta2) in radians,
    with NaN where the point is too far away (or too close) to reach."""
    x, y = xy[:, 0], xy[:, 1]
    c2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    c2 = np.where(np.abs(c2) <= 1 + 1e-12, np.clip(c2, -1, 1), np.nan)
    t2 = elbow_sign * np.arccos(c2)
    t1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(t2), L1 + L2 * np.cos(t2))
    return t1, t2


def fk(t1, t2):
    x = L1 * np.cos(t1) + L2 * np.cos(t1 + t2)
    y = L1 * np.sin(t1) + L2 * np.sin(t1 + t2)
    return np.column_stack([x, y])


def joints_to_servos(t1, t2, s1_offset=None):
    off = SERVO1_OFFSET if s1_offset is None else s1_offset
    s1 = off + SERVO1_DIRECTION * np.degrees(t1)
    s2 = SERVO2_OFFSET + SERVO2_DIRECTION * np.degrees(t2)
    return s1, s2


def servos_to_joints(s1, s2):
    t1 = np.radians((np.asarray(s1) - SERVO1_OFFSET) / SERVO1_DIRECTION)
    t2 = np.radians((np.asarray(s2) - SERVO2_OFFSET) / SERVO2_DIRECTION)
    return t1, t2


def shift_into_range(s, rng):
    """Add/subtract whole turns so angles sit as close to the range as possible
    (the same arm direction can be written as a, a+360, a-360...)."""
    mid = (rng[0] + rng[1]) / 2
    return s - 360 * np.round((s - mid) / 360)


def servo_to_pulse(s, pulse, span):
    return pulse[0] + (pulse[1] - pulse[0]) * np.asarray(s) / span


def in_range(s, rng, tol=1e-6):
    return (s >= rng[0] - tol) & (s <= rng[1] + tol)


def solve_stroke(pts):
    """Find servo angles for a whole stroke using one elbow pose throughout.
    Returns (t1, t2, s1, s2) or (None, reason)."""
    reasons = []
    for sign in ELBOW_SIGNS[ELBOW]:
        t1, t2 = ik(pts, sign)
        if np.isnan(t2).any():
            i = int(np.argmax(np.isnan(t2)))
            r = math.hypot(*pts[i])
            reasons.append(f"point ({pts[i][0]:.1f}, {pts[i][1]:.1f}) is {r:.0f} mm from the "
                           f"shoulder; arms only reach {abs(L1 - L2):.0f}-{L1 + L2:.0f} mm")
            continue
        t1 = np.unwrap(t1)
        s1, s2 = joints_to_servos(t1, t2)
        # Shift the whole stroke by whole turns as one, so it stays continuous.
        mid1 = sum(SERVO1_RANGE) / 2
        s1 = s1 - 360 * np.round((np.mean(s1) - mid1) / 360)
        t1 = servos_to_joints(s1, s2)[0]
        ok1, ok2 = in_range(s1, SERVO1_RANGE), in_range(s2, SERVO2_RANGE)
        if ok1.all() and ok2.all():
            return t1, t2, s1, s2
        i = int(np.argmax(~(ok1 & ok2)))
        which, val, rng = ("shoulder", s1[i], SERVO1_RANGE) if not ok1[i] else \
                          ("elbow", s2[i], SERVO2_RANGE)
        pose = {-1: "up", 1: "down"}[sign]
        reasons.append(f"elbow-{pose}: at ({pts[i][0]:.1f}, {pts[i][1]:.1f}) the {which} servo "
                       f"would need {val:.1f} deg (range {rng[0]:.0f}-{rng[1]:.0f})")
    return None, "; ".join(reasons)


def reachable_mask(xy, s1_offset=None):
    """Pointwise: can the LED reach each point with either allowed pose?"""
    ok = np.zeros(len(xy), bool)
    for sign in ELBOW_SIGNS[ELBOW]:
        t1, t2 = ik(xy, sign)
        valid = ~np.isnan(t2)
        s1, s2 = joints_to_servos(np.nan_to_num(t1), np.nan_to_num(t2), s1_offset)
        s1 = shift_into_range(s1, SERVO1_RANGE)
        ok |= valid & in_range(s1, SERVO1_RANGE) & in_range(s2, SERVO2_RANGE)
    return ok & (np.hypot(xy[:, 0], xy[:, 1]) >= MIN_RADIUS)


def find_best_placement(aspect, step=1.5):
    """Search shoulder offsets for the largest upright w x h rectangle
    (h = aspect * w) that fits inside the reachable area.
    Returns (shoulder_offset, center_xy, width_mm)."""
    R = L1 + L2
    xs = np.arange(-R, R + step, step)
    gx, gy = np.meshgrid(xs, xs[::-1])
    grid = np.column_stack([gx.ravel(), gy.ravel()])
    m = max(1, int(round(MARGIN / step)))
    disk = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * m + 1, 2 * m + 1))

    def evaluate(off):
        mask = reachable_mask(grid, off).reshape(gx.shape).astype(np.uint8)
        mask = cv2.erode(mask, disk)  # safety margin
        lo, hi = 0, mask.shape[1]
        while hi - lo > 1:  # binary search on width
            w = (lo + hi) // 2
            h = max(1, int(round(w * aspect)))
            if cv2.erode(mask, np.ones((h, w), np.uint8)).any():
                lo = w
            else:
                hi = w
        if lo == 0:
            return 0, None
        h = max(1, int(round(lo * aspect)))
        ok = cv2.erode(mask, np.ones((h, lo), np.uint8))
        rows, cols = np.nonzero(ok)
        j = np.argmin((rows - rows.mean())**2 + (cols - cols.mean())**2)
        # erode() anchors at the kernel center; correct for even sizes
        cx = xs[cols[j]] - (0.5 * step if lo % 2 == 0 else 0)
        cy = xs[::-1][rows[j]] + (0.5 * step if h % 2 == 0 else 0)
        return lo * step, (cx, cy)

    results = {}
    for n, off in enumerate(range(0, 360, 15)):     # coarse search
        results[off] = evaluate(off)
        print(f"\r  coarse search {n + 1}/24", end="", flush=True)
    print()
    best_off = max(results, key=lambda o: results[o][0])
    fine = [o % 360 for o in range(best_off - 12, best_off + 13, 2) if o % 360 not in results]
    for n, o in enumerate(fine):                     # refine to 2 degrees
        results[o] = evaluate(o)
        print(f"\r  fine search {n + 1}/{len(fine)}", end="", flush=True)
    print()
    best_off = max(results, key=lambda o: results[o][0])
    width, center = results[best_off]
    return float(best_off), center, width


# ---------------- 5. Images ----------------

def save_workspace(strokes, path, px_per_mm=2):
    R = L1 + L2 + 10
    n = int(2 * R * px_per_mm)
    xs = np.linspace(-R, R, n)
    gx, gy = np.meshgrid(xs, xs[::-1])
    grid = np.column_stack([gx.ravel(), gy.ravel()])
    ok = reachable_mask(grid).reshape(n, n)

    img = np.full((n, n, 3), 255, np.uint8)
    img[ok] = (215, 215, 215)

    def to_px(p):
        return (int((p[0] + R) * px_per_mm), int((R - p[1]) * px_per_mm))

    cv2.circle(img, to_px((0, 0)), 5, (0, 0, 0), -1)  # shoulder
    cv2.line(img, to_px((-R, 0)), to_px((R, 0)), (170, 170, 170), 1)
    cv2.line(img, to_px((0, -R)), to_px((0, R)), (170, 170, 170), 1)
    cv2.putText(img, "shoulder", (to_px((0, 0))[0] + 8, to_px((0, 0))[1] + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
    for _, pts, closed in strokes:
        good = reachable_mask(pts)
        for i in range(1, len(pts)):
            color = (200, 100, 0) if good[i] and good[i - 1] else (0, 0, 230)
            cv2.line(img, to_px(pts[i - 1]), to_px(pts[i]), color, 1, cv2.LINE_AA)
    cv2.imwrite(str(path), img)


def save_preview(rows, path, px_per_mm=4, margin=10):
    """Rebuild the drawing from the rounded pulse widths only
    (pulse -> servo angle -> joint angle -> position)."""
    p1 = np.array([r[8] for r in rows], float)
    p2 = np.array([r[9] for r in rows], float)
    s1 = (p1 - SERVO1_PULSE[0]) / (SERVO1_PULSE[1] - SERVO1_PULSE[0]) * SERVO1_SPAN_DEG
    s2 = (p2 - SERVO2_PULSE[0]) / (SERVO2_PULSE[1] - SERVO2_PULSE[0]) * SERVO2_SPAN_DEG
    back = fk(*servos_to_joints(s1, s2))
    xy = np.array([[r[2], r[3]] for r in rows])
    err = np.max(np.linalg.norm(back - xy, axis=1))

    lo = back.min(axis=0) - margin
    hi = back.max(axis=0) + margin
    size = ((hi - lo) * px_per_mm).astype(int) + 1
    img = np.full((size[1], size[0], 3), 255, np.uint8)

    def to_px(p):
        q = (p - lo) * px_per_mm
        return int(q[0]), int(size[1] - 1 - q[1])

    for i in range(1, len(rows)):
        color = (0, 0, 0) if rows[i][1] else (200, 200, 255)
        cv2.line(img, to_px(back[i - 1]), to_px(back[i]), color, 1, cv2.LINE_AA)
    cv2.imwrite(str(path), img)
    return err



def strokes_to_angles(raw, out_dir):
    global SERVO1_OFFSET
    bounds = drawing_bounds(raw)
    center, width, rotation = DRAWING_CENTER, DRAWING_WIDTH, DRAWING_ROTATION_DEG

    if AUTO_PLACE:
        aspect = (bounds[3] - bounds[1]) / (bounds[2] - bounds[0])
        log("Searching for the largest drawing that fits...")
        SERVO1_OFFSET, center, width = find_best_placement(aspect)
        if center is None:
            raise SystemExit("Nothing fits; check the arm lengths and servo ranges.")
        width *= 0.98  # tiny extra margin for smoothing/rounding
        rotation = 0.0
        zero_dir = (-SERVO1_OFFSET / SERVO1_DIRECTION) % 360
        print(f"  SERVO1_OFFSET = {SERVO1_OFFSET:.0f}  (mount the shoulder servo so its 0 "
              f"position points the first arm {zero_dir:.0f} deg counterclockwise from +x)")
        print(f"  DRAWING_CENTER = ({center[0]:.1f}, {center[1]:.1f}) mm")
        print(f"  DRAWING_WIDTH = {width:.1f} mm  (height {width * aspect:.1f} mm)")

    M = pixel_to_workspace_matrix(bounds, center, width, rotation)

    strokes = []
    for s in raw:
        pts = apply_matrix(M, np.array(s["points"], float))
        pts = smooth(pts, s["closed"], SMOOTH_WINDOW)
        f, length = path_function(pts, s["closed"])
        if length < MIN_STROKE_LENGTH:
            continue
        pts = resample(f, length, POINT_SPACING)
        if s["closed"]:
            pts = pts[:-1]
        strokes.append((s.get("name", "stroke"), pts, s["closed"]))

    log(f"Prepared {len(strokes)} strokes; drawing workspace.png...")
    save_workspace(strokes, out_dir / "workspace.png")
    log("Ordering strokes and solving servo angles...")

    rows, failures = [], []
    for k, (name, pts, closed) in enumerate(order_strokes(strokes)):
        if closed:
            pts = np.vstack([pts, pts[:1]])
        sol = solve_stroke(pts)
        if sol[0] is None:
            failures.append(f"  stroke {k} ({name}): {sol[1]}")
            continue
        t1, t2, s1, s2 = sol
        p1 = servo_to_pulse(s1, SERVO1_PULSE, SERVO1_SPAN_DEG)
        p2 = servo_to_pulse(s2, SERVO2_PULSE, SERVO2_SPAN_DEG)
        for i in range(len(pts)):
            rows.append([k, 0 if i == 0 else 1,
                         round(pts[i][0], 3), round(pts[i][1], 3),
                         round(math.degrees(t1[i]), 4), round(math.degrees(t2[i]), 4),
                         round(s1[i], 4), round(s2[i], 4),
                         int(round(p1[i])), int(round(p2[i]))])

    if failures:
        print("Some strokes can't be drawn with these servo limits:")
        print("\n".join(failures))
        print("See workspace.png (gray = reachable). Move DRAWING_CENTER, shrink "
              "DRAWING_WIDTH, adjust the servo settings, or set AUTO_PLACE = True.")
        raise SystemExit(1)

    with open(out_dir / "angles.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["stroke", "light", "x_mm", "y_mm",
                    "theta1_deg", "theta2_deg", "servo1_deg", "servo2_deg",
                    "pulse1_us", "pulse2_us"])
        w.writerows(rows)

    # Slim copy for the ESP32: only what the board needs, in a small file.
    esp_name = f"{out_dir.name}_esp32.csv"
    with open(out_dir / esp_name, "w", newline="") as f:
        f.write("light,pulse1_us,pulse2_us\n")
        for r in rows:
            f.write(f"{r[1]},{r[8]},{r[9]}\n")
    log(f"Saved {esp_name} (copy this one to the ESP32)")

    log("Drawing preview.png...")
    err = save_preview(rows, out_dir / "preview.png")
    p1s = [r[8] for r in rows]
    p2s = [r[9] for r in rows]
    log(f"Saved angles.csv: {len(strokes)} strokes, {len(rows)} points")
    print(f"  shoulder pulses: {min(p1s)} to {max(p1s)} us")
    print(f"  elbow pulses:    {min(p2s)} to {max(p2s)} us")
    print(f"  max position error from 1 us pulse rounding: {err:.3f} mm")


# ======================= Main =======================

def ask_for_svg():
    """Ask for the logo's file name until it names an existing .svg.
    A bare name is looked for next to this script; a full path also works.
    The .svg ending can be left off."""
    while True:
        name = input("SVG file name (e.g. LogoName.svg): ").strip().strip('"').strip("'")
        if not name:
            continue
        path = Path(name)
        if not path.suffix:
            path = path.with_name(path.name + ".svg")
        if path.suffix.lower() != ".svg":
            print("  Only .svg files are supported.")
            continue
        if not path.is_absolute():
            path = HERE / path
        if path.exists():
            return path
        print(f"  Couldn't find {path.name} in {path.parent}. Check the spelling.")


def main():
    global _T0
    svg_path = ask_for_svg()
    out_dir = HERE / svg_path.stem
    out_dir.mkdir(exist_ok=True)
    print(f"Saving everything to {out_dir}")
    _T0 = time.time()

    raw = logo_to_strokes(svg_path, out_dir)
    strokes_to_angles(raw, out_dir)
    print(f"Done. Files are in {out_dir}")


if __name__ == "__main__":
    main()

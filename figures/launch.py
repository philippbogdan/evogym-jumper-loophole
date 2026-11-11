#!/usr/bin/env python3
"""Render the launch as one continuous shot.

One continuous shot, drawn from the simulator's own state: a two-cell robot
falls into the floor, sits inside solid ground, is thrown out by the floor's
contact forces and climbs while the camera pulls back until the whole climb is
in frame. The best robot found with the contact gap closed shares the floor,
the scale and the clock. Every number shown is read from the data.

    python figures/launch.py [--data DIR] [--out DIR] [--jobs N] [--ffmpeg PATH]

For each theme (light, dark) it writes launch-THEME.gif, launch-THEME.mp4 and
launch-THEME.png (the closing frame) to figures/, from the traces in
figures/data/. Needs numpy, matplotlib, Pillow and ffmpeg.
"""

import argparse
import csv
import multiprocessing
import os
import shutil
import subprocess
import tempfile

import numpy as np
import matplotlib

matplotlib.use("Agg")
from matplotlib.font_manager import FontProperties  # noqa: E402
from matplotlib.backends.backend_agg import FigureCanvasAgg  # noqa: E402
from matplotlib.collections import LineCollection, PolyCollection  # noqa: E402
from matplotlib.colors import to_rgb  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from matplotlib.transforms import offset_copy  # noqa: E402
from PIL import Image  # noqa: E402

# ------------------------------------------------------------------ world
DT = 1e-4                  # one integration step, s
CELL = 0.1                 # cell size, m
FLOOR_TOP = 0.1            # the floor fills 0 <= x <= 6, 0 <= y <= 0.1
FLOOR_X = (0.0, 6.0)
WALL_TOP = 0.8
HONEST_DX = 1.0            # the honest jumper is drawn 1 m to the right
S_CLEAR = 6078             # first step with the launch robot clear of the floor
S_END = 15000              # last step of the episode
ARROW_SCALE = 0.12 / 2.14e5  # force arrows: metres of arrow per newton

# ------------------------------------------------------------------ shot
FPS = 25
W_PX, H_PX = 1660, 1108    # twice an 830 px wide README column
S_FIRST = 5976             # first step of the story: two steps per frame
S_STILL = 6052             # the robot comes to rest inside the floor: one step per frame
S_MOVE = 6074              # the camera starts to pull back
S_PULL = 6086              # time starts to speed up
INSIDE_LABEL = (6028, 6068)  # steps at which "inside the floor" fades in and out
FORCE_FADE = 6081          # step at which the force label starts to fade
RAMP_S = 4.0               # seconds over which the speed ramps up
STEPS_MAX = 400            # steps per frame at the end of the ramp: real time
POSTER_S = 1.6             # the closing frame, shown first
DIP_S = 0.32               # dip to the background between poster and story
LAND_S = 1.6               # the closing frame after the story (labels fade in)
CAM0 = dict(x=3.45, yb=-0.15, h=0.50)      # close-up, centred on the robot
CAM1 = dict(h=58.0, yb_frac=0.045, robot_x=0.36)
ROBOT_FRAC = 0.80          # height in the frame the climbing robot is kept at
SETTLE = 0.72              # ease onto the last framing from this fraction of its height

# 2.5D: each cell is extruded back by one cell; depth is drawn up and right.
DEPTH = 0.40 * CELL * np.array([np.cos(np.radians(50)), np.sin(np.radians(50))])

# Text uses the DejaVu Sans files that ship inside matplotlib, loaded by path,
# so the picture is the same on every machine.
FONT_DIR = os.path.join(matplotlib.get_data_path(), "fonts", "ttf")
FONT_FILES = {"regular": "DejaVuSans.ttf", "bold": "DejaVuSans-Bold.ttf"}

# One colour, vermilion (accent), marks the throw: the floor's four forces at the
# release, the path they send the robot along and the height it reaches.
# Everything else is ink and neutral greys.
THEMES = {
    "light": dict(bg="#ffffff", text="#232323", muted="#626262", rule="#bbbbbb",
                  floor="#d7d7d7", seam="#c3c3c3", far="#ababab",
                  rigid="#626262", soft="#a9a9a9", hmus="#383838", vmus="#a8a8a8",
                  accent="#e73121", shadow="#333333", shadow_alpha=0.30,
                  top_mix=0.32, side_mix=0.24, edge_mix=0.45),
    "dark": dict(bg="#0d1117", text="#ececec", muted="#979797", rule="#434343",
                 floor="#3c3c3c", seam="#2f2f2f", far="#606060",
                 rigid="#959595", soft="#666666", hmus="#cfcfcf", vmus="#5e5e5e",
                 accent="#e73121", shadow="#000000", shadow_alpha=0.45,
                 top_mix=0.16, side_mix=0.32, edge_mix=0.22),
}
MATERIAL = {1: "rigid", 2: "soft", 3: "hmus", 4: "vmus"}   # cell types, as in the data


# ------------------------------------------------------------------ helpers
def mix(c, other, f):
    a, b = np.array(to_rgb(c)), np.array(to_rgb(other))
    return tuple(a * (1.0 - f) + b * f)


def smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def clip_poly(poly, y0, keep_above):
    """Clip a polygon to one side of the line y = y0 (Sutherland-Hodgman)."""
    if poly is None:
        return None
    out = []
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        ina = a[1] >= y0 if keep_above else a[1] <= y0
        inb = b[1] >= y0 if keep_above else b[1] <= y0
        if ina:
            out.append(a)
        if ina != inb:
            out.append(a + (y0 - a[1]) / (b[1] - a[1]) * (b - a))
    return np.array(out) if len(out) >= 3 else None


def clip_segment(a, b, y0, keep_above):
    ina = a[1] >= y0 if keep_above else a[1] <= y0
    inb = b[1] >= y0 if keep_above else b[1] <= y0
    if ina and inb:
        return a, b
    if not (ina or inb):
        return None
    m = a + (y0 - a[1]) / (b[1] - a[1]) * (b - a)
    return (a, m) if ina else (m, b)


# ------------------------------------------------------------------ data
class Run:
    """One episode: every point's position at every step, and the cells."""

    def __init__(self, path, dx=0.0):
        d = np.load(path)
        self.X = d["X"] + dx
        self.Y = d["Y"]
        self.cells = d["cells"].astype(int)
        self.com0 = float(self.Y[0].mean())
        self.edges = self._boundary_edges()

    def _boundary_edges(self):
        """Outer edges (used by one cell only), as (cell, point, point)."""
        count = {}
        for row in self.cells:
            for k in range(4):
                e = tuple(sorted((row[1 + k], row[1 + (k + 1) % 4])))
                count[e] = count.get(e, 0) + 1
        out = []
        for ci, row in enumerate(self.cells):
            for k in range(4):      # corners run clockwise: TL, TR, BR, BL
                i, j = row[1 + k], row[1 + (k + 1) % 4]
                if count[tuple(sorted((i, j)))] == 1:
                    out.append((ci, i, j))
        return out

    def at(self, t):
        """Point positions at time t, in steps. Whole steps are the simulator's
        own state; between two steps the positions are linearly interpolated."""
        i = min(int(np.floor(t)), len(self.X) - 1)
        a = t - i
        if a < 1e-9 or i == len(self.X) - 1:
            return self.X[i], self.Y[i]
        return (self.X[i] + a * (self.X[i + 1] - self.X[i]),
                self.Y[i] + a * (self.Y[i + 1] - self.Y[i]))

    def com(self, t):
        X, Y = self.at(t)
        return float(X.mean()), float(Y.mean())

    def rise(self, t):
        return self.com(t)[1] - self.com0

    def peak(self):
        """Step at which the centre of mass is highest."""
        return int(np.argmax(self.Y.mean(axis=1)))


def load_forces(path):
    """Upward contact forces on the launch robot's bottom corners, by step.

    Returns {step: [(corner, force N), ...]}, corner 4 (left) or 5 (right).
    Each bottom corner is pushed up directly, and each floor corner swallowed
    by the robot pushes the bottom edge (points 5-4) up beside it. Floor
    corners are numbered left to right, so the lower index is by corner 4.
    """
    forces, swallowed = {}, {}
    with open(path) as fh:
        for r in csv.DictReader(fh):
            s, fy = int(r["step"]), float(r["normal_force_y_N"])
            if r["contact"].startswith("robot point") and r["point"] in ("4", "5"):
                forces.setdefault(s, []).append((int(r["point"]), fy))
            elif r["contact"].startswith("floor point") and r["edge"] == "4":
                swallowed.setdefault(s, []).append((int(r["point"]), -fy))
    for s, rows in swallowed.items():
        for corner, (_, fy) in zip((4, 5), sorted(rows)):
            forces.setdefault(s, []).append((corner, fy))
    return forces


# ------------------------------------------------------------------ timeline
def zoom_progress(h):
    return smoothstep(np.log(h / CAM0["h"]) / np.log(CAM1["h"] / CAM0["h"]))


def camera(h):
    """Bottom edge and centre for a frame of height h (both in metres)."""
    u = zoom_progress(h)
    yb_frac = (-CAM0["yb"] / CAM0["h"]) * (1 - u) + CAM1["yb_frac"] * u
    robot_x = 0.5 * (1 - u) + CAM1["robot_x"] * u
    w = h * W_PX / H_PX
    return -yb_frac * h, CAM0["x"] + (0.5 - robot_x) * w


def build_timeline(launch):
    """Frames as dicts: simulator time t (in steps), playback rate (steps per
    frame), camera (centre xc, bottom yb, height h), and the closing phase."""
    ts, rates = [], []
    t = float(S_FIRST)
    while t < S_PULL:                       # whole simulator steps only
        r = 2.0 if t < S_STILL else 1.0
        ts.append(t)
        rates.append(r)
        t += r
    k = 0
    while t < S_END:                        # then time speeds up smoothly
        r = float(np.exp(np.log(STEPS_MAX) * smoothstep(k / (RAMP_S * FPS))))
        ts.append(t)
        rates.append(r)
        t += r
        k += 1
    ts.append(float(S_END))
    rates.append(rates[-1])
    ts, n = np.array(ts), len(ts)

    # Camera. It holds still through the launch until S_MOVE, then starts
    # from rest and eases the robot to ROBOT_FRAC of the frame height
    # (a critically damped approach), and finally settles on the closing
    # framing of height CAM1["h"].
    top = np.array([launch.at(t)[1].max() for t in ts]) + DEPTH[1]
    still = (top - CAM0["yb"]) / CAM0["h"]
    c = int(np.searchsorted(ts, S_MOVE))
    f0, m0, lam = still[c], still[c] - still[c - 1], 0.22
    k = np.arange(n) - c
    frac = ROBOT_FRAC + (f0 - ROBOT_FRAC + (m0 + lam * (f0 - ROBOT_FRAC)) * k) * np.exp(-lam * k)
    h = np.full(n, CAM0["h"])
    for i in range(c + 1, n):
        hi = h[i - 1]
        for _ in range(50):
            yb, _ = camera(hi)
            hi = (top[i] - yb) / frac[i]
        h[i] = max(hi, CAM0["h"])
    # settle: from the frame where the tracking passes SETTLE of the closing
    # height, ease (cubic Hermite in log height, matching the slope) onto it
    s0 = int(np.argmax(h >= SETTLE * CAM1["h"]))
    a, b = np.log(h[s0]), np.log(CAM1["h"])
    u = (np.arange(n - s0)) / (n - 1 - s0)
    m = (np.log(h[s0 + 1]) - a) * (n - 1 - s0) / (b - a)
    h[s0:] = np.exp(a + (b - a) * (m * (u ** 3 - 2 * u ** 2 + u) + 3 * u ** 2 - 2 * u ** 3))
    h = np.minimum(h, CAM1["h"])

    story = []
    for i in range(n):
        yb, xc = camera(h[i])
        story.append(dict(t=float(ts[i]), rate=float(rates[i]), xc=float(xc), yb=float(yb),
                          h=float(h[i]), end=0.0, fade=0.0))
    final = dict(story[-1])
    frames = [dict(final, end=1.0) for _ in range(int(round(POSTER_S * FPS)))]
    n_dip = int(round(DIP_S * FPS / 2))
    frames += [dict(final, end=1.0, fade=(i + 1) / n_dip) for i in range(n_dip)]
    frames += [dict(story[0], fade=1.0 - (i + 1) / n_dip) for i in range(n_dip)]
    frames += story
    frames += [dict(final, end=min(1.0, (i + 1) / (0.7 * FPS)))
               for i in range(int(round(LAND_S * FPS)))]
    return frames


# ------------------------------------------------------------------ drawing
class Painter:
    def __init__(self, theme, launch, honest, forces):
        self.t = THEMES[theme]
        self.launch, self.honest, self.forces = launch, honest, forces
        self.fig = Figure(figsize=(W_PX / 100, H_PX / 100), dpi=100)
        self.canvas = FigureCanvasAgg(self.fig)
        self.fig.patch.set_facecolor(self.t["bg"])
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.face = {}
        for m in ("rigid", "soft", "hmus", "vmus", "floor"):
            c = self.t[m]
            self.face[m] = dict(front=to_rgb(c), top=mix(c, "#ffffff", self.t["top_mix"]),
                                right=mix(c, "#000000", self.t["side_mix"]),
                                edge=mix(c, "#ffffff", self.t["edge_mix"]))

    # -- geometry helpers
    def px(self):
        """Metres per pixel in the current frame."""
        return self.f["h"] / H_PX

    def side_colour(self, m, n):
        base = np.array(self.face[m]["front"])
        c = base + (np.array(self.face[m]["top"]) - base) * max(n[1], 0.0)
        c = c + (np.array(self.face[m]["right"]) - base) * max(n[0], 0.0)
        return tuple(np.clip(c, 0, 1))

    # -- scene
    def floor(self):
        ax, fc, d = self.ax, self.face["floor"], DEPTH
        x0, x1 = FLOOR_X
        back = lambda p: (p[0] + d[0], p[1] + d[1])  # noqa: E731
        polys, cols = [], []

        def box(xa, xb, ya, yt):
            polys.append([(xa, yt), (xb, yt), back((xb, yt)), back((xa, yt))])
            cols.append(fc["top"])
            polys.append([(xb, ya), (xb, yt), back((xb, yt)), back((xb, ya))])
            cols.append(fc["right"])

        box(x0, x1, 0.0, FLOOR_TOP)
        ax.add_collection(PolyCollection(polys, facecolors=cols, edgecolors=cols, linewidths=0.5,
                                         zorder=1))
        w = self.f["h"] * W_PX / H_PX
        xl, xr = self.f["xc"] - w / 2 - 0.2, self.f["xc"] + w / 2 + 0.2
        xs = np.arange(1, 60) * CELL
        xs = xs[(xs > xl) & (xs < xr)]
        seam_w = float(np.clip((CELL / self.px() - 6) / 30, 0, 1)) * 1.6
        if seam_w > 0:
            ax.add_collection(LineCollection([[(x, FLOOR_TOP), back((x, FLOOR_TOP))] for x in xs],
                                             colors=[self.t["seam"]], linewidths=seam_w,
                                             zorder=1.1))
        polys, cols = [], []
        for wx in (x0, x1 - CELL):
            box(wx, wx + CELL, FLOOR_TOP, WALL_TOP)
        ax.add_collection(PolyCollection(polys, facecolors=cols, edgecolors=cols, linewidths=0.5,
                                         zorder=1.5))
        fronts = [[(x0, 0.0), (x1, 0.0), (x1, FLOOR_TOP), (x0, FLOOR_TOP)]]
        fronts += [[(wx, FLOOR_TOP), (wx + CELL, FLOOR_TOP), (wx + CELL, WALL_TOP), (wx, WALL_TOP)]
                   for wx in (x0, x1 - CELL)]
        ax.add_collection(PolyCollection(fronts, facecolors=[fc["front"]], edgecolors=[fc["front"]],
                                         linewidths=0.5, zorder=8))
        lines = []
        if seam_w > 0:
            lines += [[(x, 0.0), (x, FLOOR_TOP)] for x in xs]
            for wx in (x0, x1 - CELL):
                lines += [[(wx, y), (wx + CELL, y)] for y in np.arange(2, 8) * CELL]
            ax.add_collection(LineCollection(lines, colors=[self.t["seam"]], linewidths=seam_w,
                                             zorder=8.1))
        # a lit edge where the floor's front meets its top
        ax.plot([x0, x1], [FLOOR_TOP, FLOOR_TOP], color=fc["edge"], lw=1.6, zorder=8.2,
                solid_capstyle="butt")
        # far away, keep the floor and walls visible with a thin outline
        far = 1.0 - float(np.clip((CELL / self.px() - 2.0) / 4.0, 0, 1))
        if far > 0:
            outline = [(x0, WALL_TOP), (x0, 0.0), (x1, 0.0), (x1, WALL_TOP), (x1 - CELL, WALL_TOP),
                       (x1 - CELL, FLOOR_TOP), (x0 + CELL, FLOOR_TOP), (x0 + CELL, WALL_TOP),
                       (x0, WALL_TOP)]
            xs_, ys_ = zip(*outline)
            ax.plot(xs_, ys_, color=self.t["far"], lw=2.2, alpha=far, zorder=8.3,
                    solid_joinstyle="miter")

    def shadow(self, run, t):
        """Soft shadow of a robot on the floor's top face, light from above."""
        X, Y = run.at(t)
        lift = max(float(Y.min()) - FLOOR_TOP, 0.0)
        if lift > 3.0:
            return
        x0, x1 = float(X.min()), float(X.max())
        spread = 0.012 + 0.35 * lift
        strength = self.t["shadow_alpha"] / (1.0 + lift / 0.15) ** 2
        n = int(np.clip(spread / self.px() / 2.0, 3, 32))
        polys = []
        for i in range(n):
            e = spread * ((i + 1) / n - 0.5)
            a, b = x0 - e, x1 + e
            polys.append([(a, FLOOR_TOP), (b, FLOOR_TOP), (b + DEPTH[0], FLOOR_TOP + DEPTH[1]),
                          (a + DEPTH[0], FLOOR_TOP + DEPTH[1])])
        alpha = 1.0 - (1.0 - strength) ** (1.0 / n)     # n layers add up to strength
        self.ax.add_collection(PolyCollection(polys, facecolors=[self.t["shadow"]],
                                              edgecolors="none", alpha=alpha, zorder=1.2))

    def robot(self, run, t, z):
        """Draw a robot at time t; return its parts hidden inside the floor."""
        pts = np.stack(run.at(t), 1)
        side_v, side_c, rims = [], [], []
        for ci, i, j in run.edges:
            a, b = pts[i], pts[j]
            d = b - a
            n = np.array([-d[1], d[0]]) / (np.hypot(*d) + 1e-12)
            if n @ DEPTH <= 0:
                continue
            m = MATERIAL[run.cells[ci][0]]
            for seg in (clip_segment(a, b, FLOOR_TOP, True), clip_segment(a, b, 0.0, False)):
                if seg is None:
                    continue
                p, q = seg
                side_v.append([p, q, q + DEPTH, p + DEPTH])
                side_c.append(self.side_colour(m, n))
                if n[1] > 0.7:
                    rims.append(([p, q], self.face[m]["edge"]))
        if side_v:
            self.ax.add_collection(PolyCollection(side_v, facecolors=side_c, edgecolors=side_c,
                                                  linewidths=0.5, zorder=z))
        front_v, front_c, hidden = [], [], []
        for row in run.cells:
            m = MATERIAL[row[0]]
            poly = pts[row[1:]]
            for part in (clip_poly(poly, FLOOR_TOP, True), clip_poly(poly, 0.0, False)):
                if part is not None:
                    front_v.append(part)
                    front_c.append(self.face[m]["front"])
            mid = clip_poly(clip_poly(poly, FLOOR_TOP, False), 0.0, True)
            if mid is not None:
                hidden.append((mid, m))
        if front_v:
            self.ax.add_collection(PolyCollection(front_v, facecolors=front_c, edgecolors=front_c,
                                                  linewidths=0.5, zorder=z + 0.3))
        size_px = CELL / self.px()
        if rims and size_px > 8:
            lw = 1.4 * float(np.clip((size_px - 8) / 30, 0, 1))
            self.ax.add_collection(LineCollection([r[0] for r in rims], colors=[r[1] for r in rims],
                                                  linewidths=lw, zorder=z + 0.4, capstyle="butt"))
        return hidden

    def hidden(self, parts):
        for poly, m in parts:
            fill = mix(self.t[m], self.t["floor"], 0.62)
            self.ax.fill(poly[:, 0], poly[:, 1], color=fill, lw=0, zorder=9)
            ring = np.vstack([poly, poly[:1]])
            self.ax.plot(ring[:, 0], ring[:, 1], color=self.t[m], lw=2.2, ls=(0, (2.6, 1.8)),
                         zorder=9.5)

    def arrows(self):
        s = int(self.f["t"])
        if s != self.f["t"] or s not in self.forces:
            return
        X, Y = self.launch.at(s)
        p = self.px()
        shaft, head_w, head_l, gap = 5 * p, 20 * p, 22 * p, 9 * p
        polys = []
        by_corner = {4: [], 5: []}
        for corner, fy in self.forces[s]:
            by_corner[corner].append(fy)
        for corner, fys in by_corner.items():
            for k, fy in enumerate(sorted(fys)):
                if fy <= 0:
                    continue
                length = fy * ARROW_SCALE
                x, tip = X[corner] + (2 * k - 1) * gap, Y[corner]
                hl = min(head_l, 0.6 * length)
                polys.append([(x - shaft / 2, tip - length), (x + shaft / 2, tip - length),
                              (x + shaft / 2, tip - hl), (x + head_w / 2, tip - hl), (x, tip),
                              (x - head_w / 2, tip - hl), (x - shaft / 2, tip - hl)])
        self.ax.add_collection(PolyCollection(polys, facecolors=[self.t["accent"]],
                                              edgecolors=[self.t["accent"]], linewidths=0.4,
                                              zorder=12))

    def trail(self):
        """Path of the centre of mass since the robot cleared the floor, with a
        dot every 50 ms. It stops short of the robot so the robot stays visible
        however small it gets."""
        t = self.f["t"]
        if t <= S_CLEAR:
            return
        L = self.launch
        end = float(L.at(t)[1].min()) - 10 * self.px()
        ss = np.arange(S_CLEAR, int(np.floor(t)) + 1, 20)
        cx, cy = L.X[ss].mean(1), L.Y[ss].mean(1)
        keep = cy < end
        if keep.sum() >= 1:
            cx = np.append(cx[keep], cx[keep][-1])
            cy = np.append(cy[keep], max(end, cy[keep][-1]))
            self.ax.plot(cx, cy, color=self.t["accent"], lw=3.0, zorder=2.5,
                         solid_capstyle="butt")
        g = np.arange(S_CLEAR + 500, t, 500).astype(int)
        gx, gy = L.X[g].mean(1), L.Y[g].mean(1)
        keep = gy < end - 4 * self.px()
        if keep.any():
            self.ax.scatter(gx[keep], gy[keep], s=(7.0 * 0.72) ** 2, marker="o",
                            color=self.t["accent"], linewidths=0, zorder=2.6)

    # -- text
    def text(self, x, y, s, size, colour, alpha=1.0, bold=False, ha="left", va="baseline",
             fig_coords=False, dx=0.0, dy=0.0):
        """Text of `size` pixels; dx, dy are pixel offsets from (x, y)."""
        if alpha <= 0.01:
            return
        base = self.fig.transFigure if fig_coords else self.ax.transData
        tr = offset_copy(base, fig=self.fig, x=dx, y=dy, units="dots")
        face = FONT_FILES["bold" if bold else "regular"]
        font = FontProperties(fname=os.path.join(FONT_DIR, face), size=size * 72.0 / 100.0)
        self.ax.text(x, y, s, transform=tr, fontproperties=font, color=colour, alpha=alpha,
                     ha=ha, va=va, zorder=20)

    def labels(self):
        f, t, s = self.f, self.t, self.f["t"]
        L = self.launch
        # the simulator's clock and the playback speed
        self.text(0.035, 0.94, "t = %.4f s" % (s * DT), 32, t["text"], fig_coords=True)
        if f["end"] > 0:
            speed = "end of episode"
        else:
            slower = 1.0 / (f["rate"] * DT * FPS)
            if slower >= 100:
                slower = 10 * round(slower / 10)
            if slower < 1.05:
                speed = "real time"
            else:
                speed = ("%.0f" % slower if slower >= 9.95 else "%.1f" % slower) + "\u00d7 slower"
        self.text(0.035, 0.94, speed, 25, t["muted"], fig_coords=True, dy=-40)
        # the moment inside the floor
        a = smoothstep((s - INSIDE_LABEL[0]) / 12) * (1 - smoothstep((s - INSIDE_LABEL[1]) / 8))
        self.text(3.5 + 0.035, 0.5 * FLOOR_TOP, "inside the floor", 30, t["text"], alpha=a,
                  va="center", dx=18)
        onset = max(self.forces, key=lambda k: max(force for _, force in self.forces[k]))    # the release: the largest forces
        a = (s >= onset) * (1 - smoothstep((s - FORCE_FADE) / 10))
        peak = [force for _, force in self.forces[onset]]
        label = "%d \u00d7 %s N" % (len(peak), format(int(round(max(peak), -3)), ","))
        self.text(3.5 + 0.035, -0.065, label, 34, t["accent"], alpha=a, va="center",
                  bold=True, dx=18)
        # name the honest jumper while it is large enough to see
        H, w = self.honest, f["h"] * W_PX / H_PX
        hx, hy = H.at(s)
        in_view = smoothstep((f["xc"] + w / 2 - hx.max()) / (0.15 * w))
        a = in_view * (1 - smoothstep((f["h"] - 5) / 3))
        self.text(0.5 * (hx.min() + hx.max() + DEPTH[0]), hy.max() + DEPTH[1],
                  "best honest jumper", 28,
                  t["muted"], alpha=a * (1 - f["end"]), ha="center", va="bottom", dy=16)
        # live height on the climbing robot
        end = f["end"]
        if s > S_CLEAR and end < 1:
            a = smoothstep((L.rise(s) - 0.2) / 0.4) * (1 - end)
            self.text(L.at(s)[0].max() + DEPTH[0], L.com(s)[1], "%.2f m" % max(L.rise(s), 0.0), 29,
                      t["text"], alpha=a, va="center", dx=20)
        if end > 0:
            self.closing(end)

    def closing(self, a):
        """A level rule from each robot's highest point to its rise, at the true
        height. A robot whose peak came before the last step is outlined there."""
        t = self.t
        w = self.f["h"] * W_PX / H_PX
        x_lab = self.f["xc"] - w / 2 + 0.70 * w
        p = self.px()
        for run, small, colour in ((self.launch, "thrown out of the floor", t["accent"]),
                                   (self.honest, "best honest jump", t["text"])):
            k = run.peak()
            if k != self.f["t"]:
                for row in run.cells:
                    ring = np.stack([run.X[k][row[1:]], run.Y[k][row[1:]]], 1)
                    ring = np.vstack([ring, ring[:1]])
                    self.ax.plot(ring[:, 0], ring[:, 1], color=t["muted"], lw=1.2, alpha=a,
                                 zorder=3)
            cy = run.com(k)[1]
            x0 = run.X[k].max() + DEPTH[0] + 14 * p
            self.ax.plot([x0, x_lab - 22 * p], [cy, cy], color=t["rule"], lw=1.6, alpha=a,
                         zorder=3, solid_capstyle="butt")
            self.text(x_lab, cy, "%.2f m" % run.rise(k), 56, colour, alpha=a, bold=True,
                      va="center")
            self.text(x_lab, cy, small, 26, t["muted"], alpha=a, va="center", dy=-50)

    # -- frame
    def draw(self, f):
        self.f = f
        ax = self.ax
        ax.clear()
        ax.set_axis_off()
        w = f["h"] * W_PX / H_PX
        ax.set_xlim(f["xc"] - w / 2, f["xc"] + w / 2)
        ax.set_ylim(f["yb"], f["yb"] + f["h"])
        s = f["t"]
        self.floor()
        self.shadow(self.honest, s)
        self.shadow(self.launch, s)
        self.trail()
        parts = self.robot(self.honest, s, z=3)
        parts += self.robot(self.launch, s, z=5)
        self.hidden(parts)
        self.arrows()
        self.labels()
        self.canvas.draw()
        img = np.asarray(self.canvas.buffer_rgba())[:, :, :3].astype(np.float32)
        if f["fade"] > 0:
            a = smoothstep(f["fade"])
            img = img * (1 - a) + np.array(to_rgb(self.t["bg"])) * 255 * a
        return np.round(img).astype(np.uint8)


# ------------------------------------------------------------------ output
def encode(frames_dir, out_base, ffmpeg="ffmpeg"):
    pattern = os.path.join(frames_dir, "f%04d.png")
    src = [ffmpeg, "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", pattern]
    subprocess.run(src + ["-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", "-preset", "slow",
                          "-threads", "1", "-movflags", "+faststart", "-map_metadata", "-1",
                          "-fflags", "+bitexact", "-flags:v", "+bitexact", out_base + ".mp4"],
                   check=True)
    palette = os.path.join(frames_dir, "palette.png")
    subprocess.run(src + ["-vf", "palettegen=max_colors=256:stats_mode=full", palette], check=True)
    subprocess.run(src + ["-i", palette, "-lavfi", "paletteuse=dither=none:diff_mode=rectangle",
                          "-loop", "0", out_base + ".gif"], check=True)


def load(data):
    launch = Run(os.path.join(data, "launch.npz"))
    honest = Run(os.path.join(data, "best_closed.npz"), dx=HONEST_DX)
    forces = load_forces(os.path.join(data, "release_contacts.csv"))
    return launch, honest, forces


def render_part(job):
    """Render some frames of one theme to PNG files (one worker process)."""
    data, theme, indices, folder, names = job
    launch, honest, forces = load(data)
    timeline = build_timeline(launch)
    painter = Painter(theme, launch, honest, forces)
    for i in indices:
        img = painter.draw(timeline[i])
        Image.fromarray(img).save(os.path.join(folder, names % i), compress_level=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--data", default=os.path.join(here, "data"),
                    help="folder with launch.npz, best_closed.npz and release_contacts.csv")
    ap.add_argument("--out", default=here, help="folder for the outputs")
    ap.add_argument("--themes", nargs="+", default=["light", "dark"], choices=sorted(THEMES))
    ap.add_argument("--jobs", type=int, default=1, help="frames rendered in parallel processes")
    ap.add_argument("--ffmpeg", default="ffmpeg", help="ffmpeg executable")
    ap.add_argument("--frames", nargs="*", type=int,
                    help="write only these frames, as PNGs, and skip the animations")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    n = len(build_timeline(load(args.data)[0]))

    for theme in args.themes:
        if args.frames:
            render_part((args.data, theme, args.frames, args.out, "frame-" + theme + "-%04d.png"))
            continue
        tmp = tempfile.mkdtemp(prefix="frames-", dir=args.out)
        try:
            jobs = [(args.data, theme, range(k, n, args.jobs), tmp, "f%04d.png")
                    for k in range(args.jobs)]
            if args.jobs > 1:
                with multiprocessing.get_context("spawn").Pool(args.jobs) as pool:
                    pool.map(render_part, jobs)
            else:
                render_part(jobs[0])
            Image.open(os.path.join(tmp, "f0000.png")).save(
                os.path.join(args.out, "launch-%s.png" % theme), optimize=True)
            encode(tmp, os.path.join(args.out, "launch-%s" % theme), args.ffmpeg)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main()

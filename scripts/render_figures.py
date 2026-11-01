"""Render the launch animation and three figures from the exported CSV data.

Run from the repository root with ``python scripts/render_figures.py``. NumPy,
Matplotlib and Pillow are required; the animation also uses ffmpeg. Input checks
validate geometry, snapshot alignment and work accumulation before rendering.
"""

import argparse
import csv
import os
import shutil
import subprocess
import tempfile

import numpy as np
import matplotlib

# Render without a display or an interactive plotting session.
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import colors as mcolors, ticker
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrow, Patch, Polygon, Rectangle

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

S = 30
DT_MS = 0.1
CELL = 0.1
FLOOR_TOP = 0.1
GRID_COLS = 60
GRID_ROWS = 8
RELEASE_FIRST = 6066
RELEASE_LAST = 6077
LOCAL_FIRST = 6030
LOCAL_LAST = 6110
ACTIONS = 500
FPS = 24
GIF_FPS = 12
MP4_SIZE = (1600, 900)
GIF_SIZE = (960, 540)
# Map video-frame intervals to simulation steps, slowing the release to eight frames per step.
SEGMENTS = [(0, 72, 0, LOCAL_FIRST), (72, 120, LOCAL_FIRST, RELEASE_FIRST),
            (120, 216, RELEASE_FIRST, RELEASE_LAST + 1), (216, 240, RELEASE_LAST + 1, LOCAL_LAST),
            (240, 360, LOCAL_LAST, ACTIONS * S)]
FRAMES = 384

INK = '#1a1a1a'
SOFT_INK = '#4a4a4a'
BLUE = '#1d5fd1'
BLUE_EDGE = '#0c2f73'
GREY = '#a6a6a6'
GREY_EDGE = '#262626'
ORANGE = '#d9620b'
ORANGE_FAINT = '#f2c3a0'
FLOOR_FILL = '#efece6'
FLOOR_LINE = '#b3ada2'
GREEN = '#117733'
PURPLE = '#aa3377'
NEUTRAL = '#666666'
GRID = '#e6e6e6'

MATERIALS = {'horizontal muscle': ('Horizontal muscle', BLUE, BLUE_EDGE),
             'rigid': ('Rigid', GREY, GREY_EDGE)}

COM_SERIES = [('best design, stock', 'Stock winner, stock', INK, '-', True),
              ('best design, loophole closed', 'Stock winner, closed', GREEN, (0, (5, 2.2)), False),
              ('best loophole-closed design, loophole closed', 'Closed winner, closed', PURPLE,
               (0, (5, 1.6, 1.2, 1.6)), False)]

TERMS = {'ground normal': 'Ground normal', 'ground damping': 'Ground damping',
         'ground friction': 'Ground friction', 'ground penalty (y<0)': 'Penalty below y = 0',
         'muscle spring work': 'Muscle springs', 'passive spring work': 'Passive springs', 'drag': 'Drag',
         'gravity work': 'Gravity', 'potential change in RK4 step': 'Potential change, RK4'}
TERM_HEAD = ['ground normal', 'ground damping', 'ground friction', 'ground penalty (y<0)',
             'muscle spring work', 'passive spring work', 'drag']
TERM_TAIL = ['gravity work', 'potential change in RK4 step']

RULES = {'base': 'Released task', 'box7': 'Box 7×7', 'no_rigid': 'No rigid cells', 'no_soft': 'No soft cells',
         'no_hmuscle': 'No horizontal muscles', 'no_vmuscle': 'No vertical muscles',
         'joins': 'Any subset of joins', 'symmetric': 'Mirror symmetry',
         'range': 'Muscle range [0.475, 1.725]', 'control': '1000 actions, 15 steps', 'horizon': '1000 actions',
         'sideways': 'No sideways penalty', 'instability': 'No instability check',
         'strain': 'Strain 50%, rigid 6%', 'dt': 'Step 5e-5 s', 'contact': 'Contact at every RK stage'}


def style():
    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.sans-serif': ['Helvetica Neue', 'Helvetica', 'Arial', 'DejaVu Sans'],
        'font.size': 9.5,
        'axes.labelsize': 10,
        'xtick.labelsize': 9,
        'ytick.labelsize': 9,
        'legend.fontsize': 9.5,
        'text.color': INK,
        'axes.labelcolor': INK,
        'axes.edgecolor': SOFT_INK,
        'axes.linewidth': 0.8,
        'xtick.color': SOFT_INK,
        'ytick.color': SOFT_INK,
        'xtick.labelcolor': INK,
        'ytick.labelcolor': INK,
        'xtick.major.width': 0.8,
        'ytick.major.width': 0.8,
        'xtick.major.size': 3.5,
        'ytick.major.size': 3.5,
        'axes.spines.top': False,
        'axes.spines.right': False,
        'axes.unicode_minus': True,
        'legend.frameon': False,
        'figure.facecolor': 'white',
        'axes.facecolor': 'white',
        'savefig.facecolor': 'white',
        'svg.fonttype': 'path',
        'svg.hashsalt': 'evogym-jumper-loophole',
        'path.simplify': False,
    })


def read(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


class Data:
    def __init__(self, folder):
        self.folder = folder
        p = lambda name: os.path.join(folder, name)

        self.cells = []
        for r in read(p('body_cells.csv')):
            assert r['material'] in MATERIALS, r['material']
            corners = [int(r[k]) for k in ('top_left_point', 'top_right_point', 'bottom_right_point', 'bottom_left_point')]
            self.cells.append((int(r['cell']), r['material'], corners))
        self.n = 1 + max(max(c[2]) for c in self.cells)

        ep = read(p('episode_points_per_action.csv'))
        a = np.array([int(r['action']) for r in ep])
        q = np.array([int(r['point']) for r in ep])
        self.actions = a.max() + 1
        assert self.actions == ACTIONS + 1 and q.max() + 1 == self.n
        self.ax = np.full((self.actions, self.n), np.nan)
        self.ay = np.full((self.actions, self.n), np.nan)
        self.ax[a, q] = [float(r['x_m']) for r in ep]
        self.ay[a, q] = [float(r['y_m']) for r in ep]
        assert np.isfinite(self.ax).all() and np.isfinite(self.ay).all()

        rp = read(p('release_points_per_step.csv'))
        st = np.array([int(r['step']) for r in rp])
        q = np.array([int(r['point']) for r in rp])
        assert all(int(r['action']) == int(r['step']) // S for r in rp)
        self.steps = np.arange(st.min(), st.max() + 1)
        assert self.steps[0] == LOCAL_FIRST and self.steps[-1] == LOCAL_LAST
        k = st - self.steps[0]
        shape = (len(self.steps), self.n)
        self.sx, self.sy, self.svx, self.svy = (np.full(shape, np.nan) for _ in range(4))
        self.sx[k, q] = [float(r['x_m']) for r in rp]
        self.sy[k, q] = [float(r['y_m']) for r in rp]
        self.svx[k, q] = [float(r['vx_m_per_s']) for r in rp]
        self.svy[k, q] = [float(r['vy_m_per_s']) for r in rp]
        assert np.isfinite(self.sx).all() and np.isfinite(self.sy).all()
        # Both snapshot tables record the state before the indexed update.
        for s in self.steps[self.steps % S == 0]:
            assert np.allclose(self.sx[s - self.steps[0]], self.ax[s // S], atol=1e-9)
            assert np.allclose(self.sy[s - self.steps[0]], self.ay[s // S], atol=1e-9)

        gw = read(p('ground_normal_work_per_step.csv'))
        self.wstep = np.array([int(r['step']) for r in gw])
        assert np.array_equal(self.wstep, np.arange(len(gw))) and len(gw) == ACTIONS * S
        self.work = np.array([float(r['ground_normal_work_J']) for r in gw])
        self.cum = np.array([float(r['cumulative_J']) for r in gw])
        assert np.allclose(np.cumsum(self.work), self.cum, rtol=1e-9, atol=1e-6)
        release = self.work[RELEASE_FIRST:RELEASE_LAST + 1]
        assert (release > 0).all() and release.sum() > 0.9 * self.work[self.work > 0].sum()

        cr = read(p('com_height_per_action.csv'))
        assert [int(r['action']) for r in cr] == list(range(self.actions))
        self.com = {key: np.array([float(r[key]) for r in cr]) for key, *_ in COM_SERIES}

        self.contacts = []
        for r in read(p('release_contacts.csv')):
            s = int(r['step'])
            rec = dict(step=s, robot=r['contact'].startswith('robot point'), point=int(r['point']),
                       cell=int(r['cell_grid_index']), edge=int(r['edge']),
                       a=int(r['edge_point_a']) if r['edge_point_a'] else None,
                       b=int(r['edge_point_b']) if r['edge_point_b'] else None,
                       depth=float(r['depth_m']), f=np.array([float(r['normal_force_x_N']), float(r['normal_force_y_N'])]))
            assert r['contact'].startswith('robot point') or r['contact'].startswith('floor point')
            ys = np.array([float(r[f'robot_point_{k}_y_m']) for k in range(self.n)])
            assert np.allclose(ys, self.sy[s - self.steps[0]], atol=1e-12)
            self.contacts.append(rec)
        self.resolve_contacts()

        self.terms = [(r['term'], float(r['work_J'])) for r in read(p('energy_terms_best_design.csv'))]
        self.rules = [(r['rules'], float(r['best_found']), float(r['best_found_loophole_closed']),
                       float(r['best_design_with_loophole_closed'])) for r in read(p('rule_variants.csv'))]

    def state(self, step):
        i = step - self.steps[0]
        return self.sx[i], self.sy[i]

    def cell_polygon(self, corners, x, y):
        return np.column_stack([x[corners], y[corners]])

    def robot_cell_of(self, grid_index):
        for cell, material, corners in self.cells:
            if cell == grid_index:
                return corners
        return None

    def resolve_contacts(self):
        # Recover fixed floor geometry from grid indices, force directions and logged depths.
        floor_candidates = {}
        for c in self.contacts:
            x, y = self.state(c['step'])
            if c['robot']:
                col, row = c['cell'] % GRID_COLS, c['cell'] // GRID_COLS
                x0, x1 = col * CELL, (col + 1) * CELL
                y0, y1 = (GRID_ROWS - 1 - row) * CELL, (GRID_ROWS - row) * CELL
                px, py = x[c['point']], y[c['point']]
                assert x0 <= px <= x1 and y0 <= py <= y1
                n = c['f'] / np.linalg.norm(c['f'])
                sides = {(0, 1): ((x0, y1), (x1, y1)), (0, -1): ((x0, y0), (x1, y0)),
                         (1, 0): ((x1, y0), (x1, y1)), (-1, 0): ((x0, y0), (x0, y1))}
                key = min(sides, key=lambda d: -np.dot(d, n))
                (ex0, ey0), (ex1, ey1) = sides[key]
                dist = abs((ey1 - py) if key[1] else (ex1 - px))
                assert abs(dist - c['depth']) < 1e-6, (c, dist)
                c['segment'] = np.array([[ex0, ey0], [ex1, ey1]])
                c['loads'] = [(c['point'], c['f'])]
            else:
                corners = self.robot_cell_of(c['cell'])
                assert corners is not None and c['a'] is not None and c['b'] is not None
                assert c['a'] in corners and c['b'] in corners
                poly = self.cell_polygon(corners, x, y)
                pa, pb = np.array([x[c['a']], y[c['a']]]), np.array([x[c['b']], y[c['b']]])
                cands = set()
                for col in range(int(poly[:, 0].min() / CELL) - 1, int(poly[:, 0].max() / CELL) + 2):
                    for vy in (0.0, FLOOR_TOP):
                        v = np.array([col * CELL, vy])
                        if not inside(poly, v):
                            continue
                        e, u = pb - pa, v - pa
                        d = abs(e[0] * u[1] - e[1] * u[0]) / np.linalg.norm(e)
                        if abs(d - c['depth']) < 2e-6:
                            cands.add((round(col * CELL, 9), vy))
                floor_candidates.setdefault(c['point'], []).append(cands)
                c['segment'] = np.array([pa, pb])
                # A force on a floor point reacts equally onto the robot edge's endpoints.
                c['loads'] = [(c['a'], -c['f'] / 2), (c['b'], -c['f'] / 2)]
        self.floor_points = {}
        for point, sets in floor_candidates.items():
            # A fixed floor point must have one location consistent with every recorded contact.
            common = set.intersection(*sets)
            assert len(common) == 1, (point, sets)
            self.floor_points[point] = np.array(common.pop())

    def release_state(self):
        for s in range(RELEASE_FIRST, RELEASE_LAST + 1):
            rows = [c for c in self.contacts if c['step'] == s]
            if any(c['robot'] for c in rows) and any(not c['robot'] for c in rows):
                return s
        raise AssertionError('no release state with both contact directions')

    def approach_action(self):
        a = (RELEASE_FIRST - 1) // S
        while a > 0 and self.ay[a].min() <= FLOOR_TOP:
            a -= 1
        assert self.ay[a].min() > FLOOR_TOP
        return a


def inside(poly, v):
    # An odd number of horizontal ray crossings puts the point inside the polygon.
    x, y = v
    c = False
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        if (y0 > y) != (y1 > y):
            if x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
                c = not c
    return c


def frac(fig, l, b, w, h):
    W, H = fig.get_size_inches()
    return [l / W, b / H, w / W, h / H]


def floor_patches(ax, x0, x1, tx=lambda v: v, z=1, lw=0.8, line=FLOOR_LINE):
    for k in range(int(np.floor(x0 / CELL)) - 1, int(np.ceil(x1 / CELL)) + 1):
        l, r = tx(k * CELL), tx((k + 1) * CELL)
        ax.add_patch(Rectangle((l, 0), r - l, FLOOR_TOP, facecolor=FLOOR_FILL, edgecolor='none', zorder=z))
    xs = [tx(k * CELL) for k in range(int(np.floor(x0 / CELL)) - 1, int(np.ceil(x1 / CELL)) + 2)]
    for v in xs:
        ax.plot([v, v], [0, FLOOR_TOP], color=line, lw=lw, zorder=z + 2.5, solid_capstyle='butt')
    ax.plot([xs[0], xs[-1]], [0, 0], color=line, lw=lw, zorder=z + 2.5)
    ax.plot([xs[0], xs[-1]], [FLOOR_TOP, FLOOR_TOP], color=line, lw=lw, zorder=z + 2.5)


def robot_patches(ax, data, x, y, tx=lambda v: v, alpha=0.9, lw=1.2, z=3):
    out = []
    for cell, material, corners in data.cells:
        _, face, edge = MATERIALS[material]
        poly = data.cell_polygon(corners, x, y)
        poly[:, 0] = tx(poly[:, 0])
        fill = Polygon(poly, closed=True, facecolor=face, edgecolor='none', alpha=alpha, zorder=z)
        line = Polygon(poly, closed=True, facecolor='none', edgecolor=edge, lw=lw, zorder=z + 1, joinstyle='miter')
        ax.add_patch(fill)
        ax.add_patch(line)
        out.append((corners, fill, line))
    return out


def material_handles():
    return [Patch(facecolor=MATERIALS[m][1], edgecolor=MATERIALS[m][2], lw=1.0, label=MATERIALS[m][0])
            for m in ('horizontal muscle', 'rigid')]


def sigma(f):
    for f0, f1, s0, s1 in SEGMENTS:
        if f0 <= f < f1:
            return s0 + (s1 - s0) * (f - f0) / (f1 - f0)
    return float(ACTIONS * S)


class Hero:
    def __init__(self, data):
        self.d = data
        d = data
        ta = np.arange(d.actions) * S
        lo, hi = ta < LOCAL_FIRST, ta > LOCAL_LAST
        # Use per-step positions near release and per-action positions elsewhere.
        self.t = np.concatenate([ta[lo], d.steps, ta[hi]]).astype(float)
        self.px = np.vstack([d.ax[lo], d.sx, d.ax[hi]])
        self.py = np.vstack([d.ay[lo], d.sy, d.ay[hi]])
        assert np.all(np.diff(self.t) > 0)

        W, H = 8.5, 8.5 * 9 / 16
        self.fig = fig = plt.figure(figsize=(W, H), dpi=MP4_SIZE[0] / W * (1 + 1e-12))

        xl, yl = (3.25, 3.65), (-0.05, 0.55)
        gl, gb = 0.54, 1.74
        gh = H - 0.12 - gb
        gw = gh * (xl[1] - xl[0]) / (yl[1] - yl[0])
        g = self.g = fig.add_axes(frac(fig, gl, gb, gw, gh))
        g.set_xlim(*xl)
        g.set_ylim(*yl)
        g.set_aspect('equal', adjustable='box')
        g.set_xticks([3.3, 3.4, 3.5, 3.6])
        g.set_yticks([0, 0.1, 0.2, 0.3, 0.4, 0.5])
        g.set_xlabel('x (m)', labelpad=3)
        g.set_ylabel('y (m)', labelpad=4)
        floor_patches(g, *xl)
        x0, y0 = self.pos(0.0)
        self.trail = g.scatter(np.zeros(1), np.zeros(1), s=6, linewidths=0, zorder=6)
        self.cells = robot_patches(g, d, x0, y0)
        self.points, = g.plot(x0, y0, 'o', ms=3.8, color=INK, zorder=7)

        cl = gl + gw + 0.66
        c = self.c = fig.add_axes(frac(fig, cl, gb, W - 0.15 - cl, gh))
        top = max(v.max() for v in d.com.values())
        c.set_xlim(0, ACTIONS)
        c.set_ylim(-0.03 * top, top * 1.04)
        c.set_xticks(range(0, ACTIONS + 1, 100))
        c.set_xlabel('Action', labelpad=3)
        c.set_ylabel('Centre-of-mass height (m)', labelpad=6)
        c.yaxis.grid(True, color=GRID, lw=0.7)
        c.set_axisbelow(True)
        self.cursor = c.axvline(0, color='#9a9a9a', lw=0.9, zorder=1)
        self.lines, self.dots = [], []
        handles = []
        for key, label, colour, ls, filled in COM_SERIES:
            ln, = c.plot([], [], color=colour, ls=ls, lw=1.7, zorder=3 if filled else 4, solid_capstyle='round')
            dot, = c.plot([], [], 'o', ms=6, color=colour, mfc=colour if filled else 'white', mew=1.4, zorder=5)
            self.lines.append((key, ln))
            self.dots.append((key, dot))
            handles.append(Line2D([], [], color=colour, ls=ls, lw=1.7, marker='o', ms=6,
                                  mfc=colour if filled else 'white', mew=1.4, label=label))
        first = c.legend(handles=material_handles(), loc='upper left', bbox_to_anchor=(0.012, 1.0), ncol=2,
                         handlelength=1.1, handleheight=0.9, columnspacing=1.4, handletextpad=0.5, borderaxespad=0.2)
        c.add_artist(first)
        c.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.012, 0.90), handlelength=3.0,
                 labelspacing=0.5, borderaxespad=0.2)

        sb, sh = 0.42, 0.62
        w = self.w = fig.add_axes(frac(fig, gl, sb, W - 0.15 - gl, sh))
        self.bars = []
        steps = np.arange(LOCAL_FIRST, LOCAL_LAST)
        kj = d.work[steps] / 1e3
        for s, v in zip(steps, kj):
            if v == 0:
                continue
            t0 = (s - RELEASE_FIRST) * DT_MS
            r = Rectangle((t0, min(0, v)), DT_MS, abs(v), facecolor='none', edgecolor=ORANGE_FAINT, lw=0.7, zorder=2)
            w.add_patch(r)
            self.bars.append((s, r))
        w.set_xlim((LOCAL_FIRST - RELEASE_FIRST) * DT_MS, (LOCAL_LAST - RELEASE_FIRST) * DT_MS)
        w.set_ylim(min(0, kj.min()) - 0.04 * kj.max(), kj.max() * 1.1)
        w.axhline(0, color=SOFT_INK, lw=0.6, zorder=1)
        w.set_xlabel('Time from release (ms)', labelpad=3)
        w.set_xticks(np.arange(np.ceil(w.get_xlim()[0]), w.get_xlim()[1] + 1e-9, 1.0))
        w.yaxis.set_major_locator(ticker.MaxNLocator(3, integer=True))
        fig.text((gl - 0.42) / W, (sb + sh + 0.07) / H, 'Ground normal work (kJ)',
                 fontsize=plt.rcParams['axes.labelsize'], ha='left', va='bottom')
        self.wcursor = w.axvline(0, color='#9a9a9a', lw=0.9, zorder=1)

    def pos(self, s):
        # Interpolation supplies animation frames between the recorded states.
        x = np.array([np.interp(s, self.t, self.px[:, p]) for p in range(self.d.n)])
        y = np.array([np.interp(s, self.t, self.py[:, p]) for p in range(self.d.n)])
        return x, y

    def frame(self, f):
        s = sigma(f)
        d = self.d
        x, y = self.pos(s)
        for corners, fill, line in self.cells:
            poly = d.cell_polygon(corners, x, y)
            fill.set_xy(poly)
            line.set_xy(poly)
        self.points.set_data(x, y)

        k = np.searchsorted(self.t, s, side='right')
        idx = np.arange(max(0, k - 10), k)
        if len(idx):
            age = (k - 1 - idx) / 10.0
            alpha = 0.42 * (1 - age) ** 1.5
            xy = np.column_stack([self.px[idx].ravel(), self.py[idx].ravel()])
            rgba = np.zeros((len(xy), 4))
            rgba[:, :3] = mcolors.to_rgb(SOFT_INK)
            rgba[:, 3] = np.repeat(alpha, d.n)
            self.trail.set_offsets(xy)
            self.trail.set_facecolors(rgba)
        a = s / S
        self.cursor.set_xdata([a, a])
        grid = np.arange(0, int(np.floor(a)) + 1)
        for (key, ln), (_, dot) in zip(self.lines, self.dots):
            v = d.com[key]
            ya = np.interp(a, np.arange(d.actions), v)
            ln.set_data(np.append(grid, a), np.append(v[grid], ya))
            dot.set_data([a], [ya])
        for st, r in self.bars:
            # Work for step st appears only after the update to state st + 1.
            done = s >= st + 1 - 1e-9
            r.set_facecolor(ORANGE if done else 'none')
            r.set_edgecolor('none' if done else ORANGE_FAINT)
        t = (s - RELEASE_FIRST) * DT_MS
        lo, hi = self.w.get_xlim()
        self.wcursor.set_xdata([t, t])
        self.wcursor.set_visible(lo <= t <= hi)

    def rgb(self, f):
        self.frame(f)
        self.fig.canvas.draw()
        buf = np.asarray(self.fig.canvas.buffer_rgba())[:, :, :3]
        assert buf.shape[1] == MP4_SIZE[0] and buf.shape[0] == MP4_SIZE[1], buf.shape
        return np.ascontiguousarray(buf)


def render_hero(data, out):
    ff = os.environ.get('FFMPEG') or shutil.which('ffmpeg') or '/opt/homebrew/bin/ffmpeg'
    hero = Hero(data)
    raw = ['-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{MP4_SIZE[0]}x{MP4_SIZE[1]}']
    scale = f'scale={GIF_SIZE[0]}:{GIF_SIZE[1]}:flags=lanczos'
    mp4 = os.path.join(out, 'launch.mp4')
    gif = os.path.join(out, 'launch.gif')
    step = FPS // GIF_FPS
    with tempfile.TemporaryDirectory() as tmp:
        # The first pass encodes the MP4 and builds one palette for the whole GIF.
        palette = os.path.join(tmp, 'palette.png')
        vp = subprocess.Popen([ff, '-y', '-loglevel', 'error', *raw, '-r', str(FPS), '-i', '-', '-an',
                               '-vf', 'scale=out_color_matrix=bt709:out_range=tv,format=yuv420p',
                               '-c:v', 'libx264', '-preset', 'medium', '-crf', '12', '-tune', 'animation',
                               '-profile:v', 'high', '-colorspace', 'bt709', '-color_primaries', 'bt709',
                               '-color_trc', 'bt709', '-color_range', 'tv', '-movflags', '+faststart', mp4],
                              stdin=subprocess.PIPE)
        pp = subprocess.Popen([ff, '-y', '-loglevel', 'error', *raw, '-r', str(GIF_FPS), '-i', '-',
                               '-vf', scale + ',palettegen=max_colors=256:stats_mode=full:reserve_transparent=0',
                               '-frames:v', '1', '-update', '1', palette], stdin=subprocess.PIPE)
        for f in range(FRAMES):
            buf = hero.rgb(f).tobytes()
            vp.stdin.write(buf)
            if f % step == 0:
                pp.stdin.write(buf)
        for proc in (vp, pp):
            proc.stdin.close()
            assert proc.wait() == 0
        # Reuse that palette on the second pass to keep GIF colours stable across frames.
        gp = subprocess.Popen([ff, '-y', '-loglevel', 'error', *raw, '-r', str(GIF_FPS), '-i', '-', '-i', palette,
                               '-lavfi', f'[0:v]{scale}[x];[x][1:v]paletteuse=dither=none:diff_mode=rectangle',
                               '-loop', '0', gif], stdin=subprocess.PIPE)
        for f in range(0, FRAMES, step):
            gp.stdin.write(hero.rgb(f).tobytes())
        gp.stdin.close()
        assert gp.wait() == 0
    plt.close(hero.fig)


ARROW = dict(width=0.0075, head_width=0.021, head_length=0.015)


def arrow_stack(ax, tip, loads, scale, z=9):
    # Stack direct and reverse loads at the same corner using a shared force scale.
    for vec, filled in loads:
        if np.linalg.norm(vec) * scale < ARROW['head_length']:
            continue
        tail = tip - vec * scale
        ax.add_patch(FancyArrow(tail[0], tail[1], *(tip - tail), length_includes_head=True,
                                facecolor=ORANGE if filled else 'white', edgecolor=ORANGE, lw=1.1, zorder=z,
                                joinstyle='miter', **ARROW))
        tip = tail


def side_chain(d, j):
    rigid = next(c[2] for c in d.cells if c[1] == 'rigid')
    chain = [rigid[3 - j], rigid[j]]
    while True:
        nxt = [c[2][j] for c in d.cells if c[2][3 - j] == chain[-1]]
        if not nxt:
            return chain
        chain.append(nxt[0])


def crossing(xx, yy, chain):
    for p0, p1 in zip(chain[:-1], chain[1:]):
        if (yy[p0] - FLOOR_TOP) * (yy[p1] - FLOOR_TOP) <= 0 and yy[p0] != yy[p1]:
            return xx[p0] + (FLOOR_TOP - yy[p0]) / (yy[p1] - yy[p0]) * (xx[p1] - xx[p0])
    return None


def contact_figure(data, out):
    d = data
    W, H = 8.5, 5.1
    fig = plt.figure(figsize=(W, H), dpi=3000 / W)
    rel = d.release_state()
    sunk = RELEASE_FIRST - 1
    app = d.approach_action()
    states = [(d.ax[app], d.ay[app]), d.state(sunk), d.state(rel)]
    rows = [c for c in d.contacts if c['step'] == rel]
    segments = [np.array(seg) for seg in sorted({tuple(map(tuple, np.round(c['segment'], 12))) for c in rows})]

    scale = 0.08 / 400000
    loads = {}
    for c in rows:
        for p, vec in c['loads']:
            loads.setdefault(p, []).append((vec, c['robot']))
    deepest = min(states[2][1][p] - sum(v[1] for v, _ in ls if v[1] > 0) * scale for p, ls in loads.items())

    left, right = 0.56, 0.10
    vb, vh = 2.36, 2.40
    yl = (deepest - 0.016, max(st[1].max() for st in states) + 0.018)
    xspan = 0.42
    vw = vh * xspan / (yl[1] - yl[0])
    gap = (W - left - right - 3 * vw) / 2
    xl = (3.45 - xspan / 2, 3.45 + xspan / 2)
    views = []
    fp = np.array(list(d.floor_points.values()))
    for i, (x, y) in enumerate(states):
        ax = fig.add_axes(frac(fig, left + i * (vw + gap), vb, vw, vh))
        ax.set_xlim(*xl)
        ax.set_ylim(*yl)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xticks([3.3, 3.4, 3.5, 3.6])
        ax.set_yticks([0, 0.1, 0.2, 0.3])
        if i:
            ax.tick_params(labelleft=False)
        else:
            ax.set_ylabel('y (m)', labelpad=4)
        if i == 1:
            ax.set_xlabel('x (m)', labelpad=3)
        floor_patches(ax, *xl)
        robot_patches(ax, d, x, y)
        ax.plot(x, y, 'o', ms=3.6, color=INK, zorder=8)
        top = fp[np.isclose(fp[:, 1], FLOOR_TOP)]
        ax.plot(top[:, 0], top[:, 1], 's', ms=4.8, mfc='white', mec=INK, mew=1.1, zorder=9)
        views.append(ax)

    g = views[2]
    x, y = states[2]
    for seg in segments:
        g.plot(seg[:, 0], seg[:, 1], color=ORANGE, alpha=0.4, lw=5.5, solid_capstyle='butt', zorder=5)
    for p, ls in loads.items():
        up = sorted([(v, r) for v, r in ls if v[1] > 0], key=lambda t: not t[1])
        arrow_stack(g, np.array([x[p], y[p]]), up, scale)

    cb, chh = 0.46, 1.30
    cgap = 0.62
    cw = (W - left - right - cgap) / 2
    xs, ys = d.state(rel)
    xs0, ys0 = d.state(sunk)
    k_rel = rel - d.steps[0]
    for j in (0, 1):
        chain = side_chain(d, j)
        corner = chain[0]
        ref = round(xs[corner] / CELL) * CELL
        # Stretch x into micrometre offsets from the floor boundary; y stays in metres.
        tx = lambda v, ref=ref: (np.asarray(v) - ref) * 1e6
        vals = [0.0]
        for xx, yy in ((xs0, ys0), (xs, ys)):
            vals.append(tx(xx[corner]))
            cr = crossing(xx, yy, chain)
            if cr is not None:
                vals.append(tx(cr))
        span = max(vals) - min(vals)
        lo, hi = 10 * np.floor((min(vals) - 0.5 * span) / 10), 10 * np.ceil((max(vals) + 0.5 * span) / 10)
        ax = fig.add_axes(frac(fig, left + j * (cw + cgap), cb, cw, chh))
        ax.set_xlim(lo, hi)
        ax.set_ylim(-0.012, 0.124)
        ax.set_yticks([0, 0.05, 0.1])
        ax.xaxis.set_major_locator(ticker.MultipleLocator(50))
        ax.set_xlabel('x offset (µm)', labelpad=3)
        if j == 0:
            ax.set_ylabel('y (m)', labelpad=4)
        floor_patches(ax, ref - 0.2, ref + 0.2, tx=tx, lw=1.0, line='#8f887c')
        robot_patches(ax, d, xs, ys, tx=tx, alpha=0.32, lw=1.5, z=3)
        first = int(np.argmin(d.sy[:k_rel, corner]))
        # Fade the side-edge history from the deepest sunk state to the step before release.
        hist = range(first, k_rel)
        for i in hist:
            age = (k_rel - 1 - i) / max(1, k_rel - 1 - first)
            alpha = 0.10 + 0.62 * (1 - age) ** 2
            ax.plot(tx(d.sx[i][chain]), d.sy[i][chain], color=INK, lw=0.85, alpha=alpha, zorder=4.5,
                    solid_joinstyle='miter')
        for seg in segments:
            ax.plot(tx(seg[:, 0]), seg[:, 1], color=ORANGE, alpha=0.4, lw=5.5, solid_capstyle='butt', zorder=5)
        ax.plot(tx(xs), ys, 'o', ms=4.2, color=INK, zorder=9)
        ax.plot(tx(fp[:, 0]), fp[:, 1], 's', ms=5.4, mfc='white', mec=INK, mew=1.2, zorder=10)

    handles = material_handles() + [
        Line2D([], [], ls='none', marker='o', ms=4.5, color=INK, label='Robot point'),
        Line2D([], [], ls='none', marker='s', ms=5, mfc='white', mec=INK, mew=1.1, label='Floor point'),
        Line2D([], [], color=ORANGE, alpha=0.4, lw=5.5, solid_capstyle='butt', label='Selected edge')]
    leg = fig.legend(handles=handles, loc='upper left', bbox_to_anchor=((left - 0.08) / W, 1 - 0.05 / H), ncol=5,
                     handlelength=1.4, handleheight=0.9, columnspacing=1.5, handletextpad=0.5, borderaxespad=0,
                     borderpad=0)
    key_len = 400000 * scale * vw / xspan
    ky = H - 0.05 - 0.085
    kx = W - right - 0.32 - key_len
    k = fig.add_axes(frac(fig, kx, ky - 0.10, key_len, 0.20))
    k.set_xlim(0, 400000 * scale)
    k.set_ylim(-0.1 * 400000 * scale / key_len, 0.1 * 400000 * scale / key_len)
    k.axis('off')
    k.add_patch(FancyArrow(0, 0, 400000 * scale, 0, length_includes_head=True, facecolor=ORANGE,
                           edgecolor=ORANGE, lw=1.1, joinstyle='miter', **ARROW))
    fig.text(kx / W, (ky - 0.085) / H, '0', ha='center', va='top', fontsize=plt.rcParams['xtick.labelsize'])
    fig.text((kx + key_len) / W, (ky - 0.085) / H, '400,000', ha='center', va='top',
             fontsize=plt.rcParams['xtick.labelsize'])
    fig.text((kx - 0.14) / W, ky / H, 'Normal force (N)', ha='right', va='center',
             fontsize=plt.rcParams['legend.fontsize'])
    save(fig, out, 'contact-mechanism', (3000, 1800))


def energy_figure(data, out):
    d = data
    W, H = 8.5, 5.1
    fig = plt.figure(figsize=(W, H), dpi=3000 / W)
    known = dict(d.terms)
    order = [t for t in TERM_HEAD if t in known] + [t for t, _ in d.terms if t not in TERM_HEAD + TERM_TAIL] + \
            [t for t in TERM_TAIL if t in known]
    assert sorted(order) == sorted(known)
    vals = np.array([known[t] / 1e3 for t in order])
    labels = [TERMS.get(t, t) for t in order]
    colours = [ORANGE if t == 'ground normal' else BLUE if t == 'muscle spring work' else NEUTRAL for t in order]

    ub, uh = 2.70, 2.30
    ul = 1.62
    u = fig.add_axes(frac(fig, ul, ub, W - 0.20 - ul, uh))
    lim = np.abs(vals).max() * 1.08
    u.set_xlim(-lim, lim)
    yy = np.arange(len(order))[::-1]
    u.barh(yy, vals, height=0.62, color=colours, edgecolor='none', zorder=3)
    small = np.abs(vals) < 0.012 * lim
    for v, y_, c in zip(vals[small], yy[small], np.array(colours)[small]):
        u.plot([v], [y_], 'o', ms=4.6, color=c, mec='white', mew=0.6, zorder=5)
    u.axvline(0, color=INK, lw=0.9, zorder=4)
    u.set_yticks(yy)
    u.set_yticklabels(labels)
    u.tick_params(axis='y', length=0, pad=6)
    u.spines['left'].set_visible(False)
    u.xaxis.grid(True, color=GRID, lw=0.7)
    u.set_axisbelow(True)
    u.set_ylim(-0.6, len(order) - 0.4)
    u.set_xlabel('Work (kJ)', labelpad=3)
    u.xaxis.set_major_locator(ticker.MultipleLocator(20))

    s0, s1 = RELEASE_FIRST - 6, RELEASE_LAST + 6
    steps = np.arange(s0, s1 + 1)
    t0 = (steps - RELEASE_FIRST) * DT_MS
    ll = 0.62
    bb, bh = 1.42, 0.78
    cb_, ch = 0.45, 0.62
    b = fig.add_axes(frac(fig, ll, bb, W - 0.20 - ll, bh))
    c = fig.add_axes(frac(fig, ll, cb_, W - 0.20 - ll, ch), sharex=b)
    kj = d.work[steps] / 1e3
    b.bar(t0, kj, width=DT_MS, align='edge', color=ORANGE, edgecolor='white', lw=0.6, zorder=3)
    b.axhline(0, color=INK, lw=0.7, zorder=4)
    b.set_ylim(min(0, kj.min()) * 1.1, kj.max() * 1.12)
    b.yaxis.set_major_locator(ticker.MaxNLocator(4, integer=True))
    b.tick_params(labelbottom=False)
    b.yaxis.grid(True, color=GRID, lw=0.7)
    b.set_axisbelow(True)
    edges = np.append(t0, t0[-1] + DT_MS)
    # Cumulative work at a state boundary includes only updates already completed.
    held = np.array([d.cum[s - 1] if s > 0 else 0.0 for s in np.append(steps, s1 + 1)]) / 1e3
    c.step(edges, held, where='post', color=ORANGE, lw=1.8, zorder=3)
    c.plot(edges, held, 'o', ms=2.8, color=ORANGE, zorder=4)
    c.set_ylim(0, d.cum.max() / 1e3 * 1.12)
    c.yaxis.set_major_locator(ticker.MaxNLocator(3, integer=True))
    c.yaxis.grid(True, color=GRID, lw=0.7)
    c.set_axisbelow(True)
    c.set_xlim(edges[0], edges[-1])
    c.set_xlabel('Time from release (ms)', labelpad=3)
    c.xaxis.set_major_locator(ticker.MultipleLocator(0.5))
    for ax_, label in [(b, 'Ground normal work (kJ)'), (c, 'Cumulative work (kJ)')]:
        pos = ax_.get_position()
        fig.text(pos.x0 - 0.40 / W, pos.y1 + 0.07 / H, label, ha='left', va='bottom',
                 fontsize=plt.rcParams['axes.labelsize'])
    save(fig, out, 'energy-account', (3000, 1800))


def rules_figure(data, out):
    d = data
    W, H = 8.5, 6.8
    fig = plt.figure(figsize=(W, H), dpi=3000 / W)
    codes = [r[0] for r in d.rules]
    assert codes[0] == 'base' and codes[-1] == 'contact' and len(codes) == len(RULES) and set(codes) == set(RULES)
    l = 2.05
    ax = fig.add_axes(frac(fig, l, 0.55, W - 0.25 - l, H - 0.55 - 0.55))
    n = len(d.rules)
    y = np.arange(n)[::-1]
    best = np.array([r[1] for r in d.rules])
    closed = np.array([r[2] for r in d.rules])
    own = np.array([r[3] for r in d.rules])
    for yy, a, b_ in zip(y, best, closed):
        ax.plot([b_, a], [yy, yy], color='#b5b5b5', lw=0.9, zorder=2, solid_capstyle='butt')
    ax.plot(best, y, 'o', ms=7.5, color=INK, ls='none', zorder=4, label='Best found, open')
    ax.plot(closed, y, 'o', ms=7.5, mfc='white', mec=PURPLE, mew=1.5, ls='none', zorder=4, label='Best found, closed')
    ax.plot(own, y, 'x', ms=5.5, mew=1.5, color=GREEN, ls='none', zorder=5, label='Open winner, closed')
    ax.axvline(0, color=SOFT_INK, lw=0.8, zorder=3)
    lo = min(own.min(), closed.min(), 0)
    hi = max(best.max(), 0)
    ax.set_xlim(lo - 0.035 * (hi - lo), hi + 0.035 * (hi - lo))
    ax.set_ylim(-0.6, n - 0.4)
    ax.set_yticks(y)
    ax.set_yticklabels([RULES[c] for c in codes])
    ax.tick_params(axis='y', length=0, pad=8)
    ax.spines['left'].set_visible(False)
    ax.xaxis.grid(True, color=GRID, lw=0.7)
    ax.set_axisbelow(True)
    ax.xaxis.set_major_locator(ticker.MultipleLocator(100))
    ax.set_xlabel('Return', labelpad=4)
    for yy in y[1::2]:
        ax.axhspan(yy - 0.5, yy + 0.5, color='#f6f6f6', zorder=0, lw=0)
    ax.legend(loc='lower left', bbox_to_anchor=(0, 1.012), ncol=3, borderaxespad=0, columnspacing=2.0,
              handletextpad=0.4)
    save(fig, out, 'rule-variants', (3000, 2400))


def save(fig, out, name, size):
    png = os.path.join(out, name + '.png')
    fig.savefig(png, dpi=size[0] / fig.get_size_inches()[0] * (1 + 1e-12), metadata={'Software': None})
    fig.savefig(os.path.join(out, name + '.svg'), metadata={'Date': None, 'Creator': None})
    plt.close(fig)
    from PIL import Image
    with Image.open(png) as im:
        assert im.size == size, (name, im.size)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data', default=os.path.join(ROOT, 'figures', 'data'),
                   help='input CSV directory (default: figures/data/ in the repository)')
    p.add_argument('--out', default=os.path.join(ROOT, 'figures'),
                   help='output directory; replaces selected visuals (default: figures/ in the repository)')
    p.add_argument('--only', nargs='*', default=['hero', 'contact', 'energy', 'rules'],
                   help='visuals to render: hero contact energy rules (default: all four)')
    a = p.parse_args()
    style()
    data = Data(a.data)
    os.makedirs(a.out, exist_ok=True)
    renders = {'hero': render_hero, 'contact': contact_figure, 'energy': energy_figure, 'rules': rules_figure}
    for name in a.only:
        renders[name](data, a.out)


if __name__ == '__main__':
    main()

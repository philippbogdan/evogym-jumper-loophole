#!/usr/bin/env python3
"""Render three instants of the launch: a two-cell robot that falls into the floor and is fired out.

In Evolution Gym's jumping task this robot, a horizontal muscle (dark grey) on
top of a rigid cell (vermilion), lands on the floor, passes through its surface,
sits inside a floor cell and is then thrown out at 110 m/s. Three instants are
drawn side by side at the same scale: it lands; it is inside the floor; it is
fired out. Vermilion marks the cell that goes into the floor.

What is data and what is drawing:
  * The front face of every block is the simulation's plane. Every robot corner
    on it is where the simulator recorded it. The instants, the times and the
    speeds in the labels are all found in the recorded states (find_instants,
    time_step); none is typed in.
  * The simulation is two-dimensional. The depth of the blocks, the floor shown
    as a short piece with cut ends, the light, the shadows and the rounded edges
    are drawing choices.
  * Each robot also leaves a fading trail made of its own 24 previous recorded
    states (2.4 ms), as a long photographic exposure ending in a flash would.

Usage:
    python figures/inside.py [--data DIR] [--out DIR] [--draft]

Reads figures/data/launch.npz and writes inside-light.png and inside-dark.png
to figures/. Needs numpy, numba, Pillow and matplotlib (for its font).
"""
import argparse
import math
import os

import numpy as np
from numba import njit, prange
from PIL import Image, ImageDraw, ImageFont

# ----------------------------------------------------------------------------
# The simulated world (from the simulator's world definition; the floor is
# fixed, so it is not part of the recorded states)
# ----------------------------------------------------------------------------
CELL = 0.1           # cell size in metres
FLOOR_TOP = 0.1      # the floor fills y from 0 to 0.1; its underside is y = 0
TRAIL = 24           # recorded states drawn behind each instant as its trail


def time_step(data):
    """Seconds per integration step, recovered from the recorded motion: over
    each step a point moves by its mean velocity times the step."""
    Y, VY = data["Y"], data["VY"]
    dy = Y[1:] - Y[:-1]
    v = 0.5 * (VY[1:] + VY[:-1])
    ok = np.abs(v) > 1.0
    return float(np.median(dy[ok] / v[ok]))


def find_instants(data):
    """The instants drawn, found in the recorded states (row s is the state
    before integration step s).

    strike: the step with the largest jump in the centre of mass's vertical
        speed, when the floor finds the robot deep inside it and throws it out
    land: the last state before the strike with every point above the floor
    inside: the last state before the robot's corners reach the floor's underside
    out: TRAIL steps after the strike, so that its trail starts at the strike
    """
    Y, VY = data["Y"], data["VY"]
    strike = int(np.argmax(np.diff(VY.mean(1))))
    low = Y.min(1)
    land = int(np.nonzero(low[:strike] > FLOOR_TOP)[0][-1])
    reach = land + int(np.nonzero(low[land:strike] <= 0.0)[0][0])
    return dict(land=land, inside=reach - 1, strike=strike, out=strike + TRAIL)


# ----------------------------------------------------------------------------
# Small deterministic random number generator usable inside numba kernels
# ----------------------------------------------------------------------------


@njit(inline="always")
def _mix(x):
    x = x + np.uint64(0x9E3779B97F4A7C15)
    x = (x ^ (x >> np.uint64(30))) * np.uint64(0xBF58476D1CE4E5B9)
    x = (x ^ (x >> np.uint64(27))) * np.uint64(0x94D049BB133111EB)
    return x ^ (x >> np.uint64(31))


@njit(inline="always")
def _rand(s):
    s = s ^ (s >> np.uint64(12))
    s = s ^ (s << np.uint64(25))
    s = s ^ (s >> np.uint64(27))
    v = (s * np.uint64(2685821657736338717)) >> np.uint64(11)
    return s, v * (1.0 / 9007199254740992.0)


# ----------------------------------------------------------------------------
# Geometry: convex prisms stored as planes n.p <= d
# ----------------------------------------------------------------------------


@njit(fastmath=True, inline="always")
def _convex_entry(P, b, ox, oy, oz, dx, dy, dz):
    tn = -1e30
    tf = 1e30
    kn = -1
    for k in range(6):
        nx = P[b, k, 0]
        ny = P[b, k, 1]
        nz = P[b, k, 2]
        den = nx * dx + ny * dy + nz * dz
        sd = nx * ox + ny * oy + nz * oz - P[b, k, 3]
        if den > -1e-14 and den < 1e-14:
            if sd > 0.0:
                return 1e30, -1
        else:
            t = -sd / den
            if den < 0.0:
                if t > tn:
                    tn = t
                    kn = k
            elif t < tf:
                tf = t
        if tn > tf:
            return 1e30, -1
    return tn, kn


@njit(fastmath=True, inline="always")
def _box_hit(B, p, ox, oy, oz, dx, dy, dz, tmax):
    t0 = 0.0
    t1 = tmax
    for a in range(3):
        if a == 0:
            o = ox
            d = dx
        elif a == 1:
            o = oy
            d = dy
        else:
            o = oz
            d = dz
        lo = B[p, 2 * a]
        hi = B[p, 2 * a + 1]
        if d > -1e-14 and d < 1e-14:
            if o < lo or o > hi:
                return False
        else:
            ta = (lo - o) / d
            tb = (hi - o) / d
            if ta > tb:
                ta, tb = tb, ta
            if ta > t0:
                t0 = ta
            if tb < t1:
                t1 = tb
            if t0 > t1:
                return False
    return True


@njit(fastmath=True)
def _trace(ox, oy, oz, dx, dy, dz, SP, Sprio, Spanel, GP, galpha, chosen, PB,
           ntime, s, use_ground):
    """Nearest hit. Returns t, id, plane, rng state. id: -1 sky, -2 ground,
    0..S-1 static block, S+g ghost block g."""
    best = 1e30
    bid = -1
    bpl = -1
    bprio = -1
    if use_ground and dy < -1e-12:
        t = -oy / dy
        if t > 1e-7:
            best = t
            bid = -2
    S = SP.shape[0]
    npan = PB.shape[0]
    for p in range(npan):
        if not _box_hit(PB, p, ox, oy, oz, dx, dy, dz, best + 1e-6):
            continue
        for b in range(S):
            if Spanel[b] != p:
                continue
            tn, kn = _convex_entry(SP, b, ox, oy, oz, dx, dy, dz)
            if kn < 0 or tn <= 1e-7:
                continue
            if tn < best - 1e-7 or (tn < best + 1e-7 and Sprio[b] > bprio):
                best = tn
                bid = b
                bpl = kn
                bprio = Sprio[b]
        c = chosen[p]
        if c >= 0:
            for cell in range(2):
                g = (p * ntime + c) * 2 + cell
                tn, kn = _convex_entry(GP, g, ox, oy, oz, dx, dy, dz)
                if kn < 0 or tn <= 1e-7:
                    continue
                if tn < best - 1e-7 or (tn < best + 1e-7 and 1 > bprio):
                    s, u = _rand(s)
                    if u < galpha:
                        best = tn
                        bid = S + g
                        bpl = kn
                        bprio = 1
    return best, bid, bpl, s


@njit(fastmath=True, inline="always")
def _onb(nx, ny, nz):
    if abs(nx) > 0.9:
        ax, ay, az = 0.0, 1.0, 0.0
    else:
        ax, ay, az = 1.0, 0.0, 0.0
    tx = ay * nz - az * ny
    ty = az * nx - ax * nz
    tz = ax * ny - ay * nx
    l = math.sqrt(tx * tx + ty * ty + tz * tz)
    tx /= l
    ty /= l
    tz /= l
    return tx, ty, tz, ny * tz - nz * ty, nz * tx - nx * tz, nx * ty - ny * tx


@njit(fastmath=True)
def _shade_normal(P, b, kn, hx, hy, hz, bevel):
    """Face normal bent towards neighbouring faces near edges: a rounded edge
    in shading only (the silhouette stays exact)."""
    nx = P[b, kn, 0]
    ny = P[b, kn, 1]
    nz = P[b, kn, 2]
    mx, my, mz = nx, ny, nz
    for k in range(6):
        if k == kn:
            continue
        ax = P[b, k, 0]
        ay = P[b, k, 1]
        az = P[b, k, 2]
        if ax * nx + ay * ny + az * nz < -0.5:
            continue
        d = P[b, k, 3] - (ax * hx + ay * hy + az * hz)
        if d < bevel:
            w = 1.0 - d / bevel
            w = w * w
            mx += w * ax
            my += w * ay
            mz += w * az
    l = math.sqrt(mx * mx + my * my + mz * mz)
    return mx / l, my / l, mz / l


@njit(parallel=True, fastmath=True)
def render_kernel(W, H, spp, cam, SP, Sprio, Spanel, Skind, Scol, Sseam,
                  GP, Gcol, galpha, gcdf, PB, light, sky, shade, seed):
    """Path trace the scene. Returns object radiance sums, object hit counts and
    the shadow catcher numerator and denominator for the ground."""
    obj = np.zeros((H, W, 3))
    objn = np.zeros((H, W))
    gnum = np.zeros((H, W))
    gden = np.zeros((H, W))
    S = SP.shape[0]
    npan = PB.shape[0]
    ntime = gcdf.shape[1]
    lcx, lcy, lcz = light[0], light[1], light[2]
    lux, luy, luz = light[3], light[4], light[5]
    lvx, lvy, lvz = light[6], light[7], light[8]
    Lk = light[9]
    lnx = luy * lvz - luz * lvy
    lny = luz * lvx - lux * lvz
    lnz = lux * lvy - luy * lvx
    larea = math.sqrt(lnx * lnx + lny * lny + lnz * lnz)
    lnx /= larea
    lny /= larea
    lnz /= larea
    bevel = shade[0]
    seam_w = shade[1]
    seam_dark = shade[2]
    ftr, ftg, ftb = shade[3], shade[4], shade[5]
    spec_k = shade[6]
    spec_n = shade[7]
    groundalb = shade[8]
    sq = int(math.sqrt(spp))
    if sq * sq < spp:
        sq += 1
    for j in prange(H):
        chosen = np.empty(npan, np.int64)
        for i in range(W):
            s = _mix(np.uint64(seed) * np.uint64(0x100000001B3) + np.uint64(j * W + i + 1))
            for k in range(spp):
                s, u1 = _rand(s)
                s, u2 = _rand(s)
                jx = ((k % sq) + u1) / sq
                jy = (((k // sq) % sq) + u2) / sq
                px = ((i + jx) / W * 2.0 - 1.0) * cam[12]
                py = (1.0 - (j + jy) / H * 2.0) * cam[13]
                ox = cam[0] + px * cam[6] + py * cam[9]
                oy = cam[1] + px * cam[7] + py * cam[10]
                oz = cam[2] + px * cam[8] + py * cam[11]
                dx, dy, dz = cam[3], cam[4], cam[5]
                # one past state per panel for the motion trail
                for p in range(npan):
                    s, u = _rand(s)
                    c = -1
                    for q in range(ntime):
                        if u < gcdf[p, q]:
                            c = q
                            break
                    chosen[p] = c
                tr, tg, tb = 1.0, 1.0, 1.0
                Lr, Lg, Lb = 0.0, 0.0, 0.0
                hit_obj = False
                for depth in range(5):
                    t, bid, pl, s = _trace(ox, oy, oz, dx, dy, dz, SP, Sprio, Spanel, GP,
                                           galpha, chosen, PB, ntime, s, True)
                    if bid == -1:
                        if depth > 0:
                            sk = sky[0] if dy > 0.0 else sky[1]
                            Lr += tr * sk
                            Lg += tg * sk
                            Lb += tb * sk
                        break
                    hx = ox + t * dx
                    hy = oy + t * dy
                    hz = oz + t * dz
                    glossy = False
                    if bid == -2:
                        nx, ny, nz = 0.0, 1.0, 0.0
                        ar = groundalb
                        ag = groundalb
                        ab = groundalb
                    elif bid < S:
                        nx, ny, nz = _shade_normal(SP, bid, pl, hx, hy, hz, bevel)
                        ar = Scol[bid, 0]
                        ag = Scol[bid, 1]
                        ab = Scol[bid, 2]
                        if Skind[bid] == 1:
                            fnz = SP[bid, pl, 2]
                            if fnz > 0.5:
                                ar *= ftr
                                ag *= ftg
                                ab *= ftb
                            if abs(SP[bid, pl, 0]) < 0.5:
                                fx = (hx - Sseam[bid]) / 0.1
                                off = (fx - math.floor(fx + 0.5)) * 0.1
                                if abs(off) < seam_w:
                                    w = 1.0 - abs(off) / seam_w
                                    sg = 1.0 if off > 0 else -1.0
                                    nx += sg * 0.9 * w
                                    l = math.sqrt(nx * nx + ny * ny + nz * nz)
                                    nx /= l
                                    ny /= l
                                    nz /= l
                                    dk = 1.0 - (1.0 - seam_dark) * w
                                    ar *= dk
                                    ag *= dk
                                    ab *= dk
                        else:
                            glossy = True
                    else:
                        g = bid - S
                        nx, ny, nz = _shade_normal(GP, g, pl, hx, hy, hz, bevel)
                        ar = Gcol[g, 0]
                        ag = Gcol[g, 1]
                        ab = Gcol[g, 2]
                    # light sample
                    s, u1 = _rand(s)
                    s, u2 = _rand(s)
                    qx = lcx + (u1 - 0.5) * lux + (u2 - 0.5) * lvx
                    qy = lcy + (u1 - 0.5) * luy + (u2 - 0.5) * lvy
                    qz = lcz + (u1 - 0.5) * luz + (u2 - 0.5) * lvz
                    wx = qx - hx
                    wy = qy - hy
                    wz = qz - hz
                    d2 = wx * wx + wy * wy + wz * wz
                    dl = math.sqrt(d2)
                    wx /= dl
                    wy /= dl
                    wz /= dl
                    cs = nx * wx + ny * wy + nz * wz
                    cl = -(lnx * wx + lny * wy + lnz * wz)
                    geo = 0.0
                    if cs > 0.0 and cl > 0.0:
                        geo = Lk * cs * cl * larea / d2
                    # cosine sample for the sky and the next bounce
                    s, u1 = _rand(s)
                    s, u2 = _rand(s)
                    r = math.sqrt(u1)
                    ph = 2.0 * math.pi * u2
                    tx, ty, tz, bx, by, bz = _onb(nx, ny, nz)
                    lx = r * math.cos(ph)
                    ly = r * math.sin(ph)
                    lz = math.sqrt(max(0.0, 1.0 - u1))
                    ex = lx * tx + ly * bx + lz * nx
                    ey = lx * ty + ly * by + lz * ny
                    ez = lx * tz + ly * bz + lz * nz
                    if bid == -2 and depth == 0:
                        vk = 0.0
                        if geo > 0.0:
                            t2, b2, p2, s = _trace(hx, hy, hz, wx, wy, wz, SP, Sprio, Spanel, GP,
                                                   galpha, chosen, PB, ntime, s, False)
                            if b2 == -1 or t2 > dl:
                                vk = 1.0
                        t3, b3, p3, s = _trace(hx, hy, hz, ex, ey, ez, SP, Sprio, Spanel, GP,
                                               galpha, chosen, PB, ntime, s, False)
                        vs = 1.0 if b3 == -1 else 0.0
                        skyE = sky[0] * math.pi
                        gnum[j, i] += vk * geo + vs * skyE
                        gden[j, i] += geo + skyE
                        break
                    if depth == 0:
                        hit_obj = True
                    ox = hx + nx * 1e-7
                    oy = hy + ny * 1e-7
                    oz = hz + nz * 1e-7
                    if geo > 0.0:
                        t2, b2, p2, s = _trace(ox, oy, oz, wx, wy, wz, SP, Sprio, Spanel, GP,
                                               galpha, chosen, PB, ntime, s, True)
                        if b2 == -1 or t2 > dl:
                            f = geo / math.pi
                            sr = 0.0
                            if glossy:
                                # Blinn-Phong sheen from the key light
                                vx = -dx
                                vy = -dy
                                vz = -dz
                                hhx = wx + vx
                                hhy = wy + vy
                                hhz = wz + vz
                                hl = math.sqrt(hhx * hhx + hhy * hhy + hhz * hhz)
                                ch = (nx * hhx + ny * hhy + nz * hhz) / hl
                                if ch > 0.0:
                                    sr = spec_k * (spec_n + 8.0) / (8.0 * math.pi) * ch ** spec_n * geo
                            Lr += tr * (ar * f + sr)
                            Lg += tg * (ag * f + sr)
                            Lb += tb * (ab * f + sr)
                    tr *= ar
                    tg *= ag
                    tb *= ab
                    dx, dy, dz = ex, ey, ez
                    if depth >= 2:
                        q = max(tr, max(tg, tb))
                        s, u = _rand(s)
                        if u > q:
                            break
                        tr /= q
                        tg /= q
                        tb /= q
                if hit_obj:
                    obj[j, i, 0] += Lr
                    obj[j, i, 1] += Lg
                    obj[j, i, 2] += Lb
                    objn[j, i] += 1.0
    return obj, objn, gnum, gden


# ----------------------------------------------------------------------------
# Scene construction
# ----------------------------------------------------------------------------


def _box_planes(x0, x1, y0, y1, z0, z1):
    return [[1, 0, 0, x1], [-1, 0, 0, -x0], [0, 1, 0, y1],
            [0, -1, 0, -y0], [0, 0, 1, z1], [0, 0, -1, -z0]]


def _prism_planes(xy, z0, z1):
    """Planes of the quadrilateral xy (perimeter order) extruded from z0 to z1."""
    c = np.asarray(xy, float)
    cen = c.mean(0)
    out = []
    for i in range(4):
        a, b = c[i], c[(i + 1) % 4]
        e = b - a
        n = np.array([e[1], -e[0]]) / np.hypot(e[0], e[1])
        if np.dot(n, cen - a) > 0:
            n = -n
        out.append([n[0], n[1], 0.0, float(n @ a)])
    out.append([0, 0, 1, z1])
    out.append([0, 0, -1, -z0])
    return out


def _lin(rgb255):
    c = np.asarray(rgb255, float) / 255.0
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def _srgb(x):
    x = np.clip(x, 0.0, 1.0)
    return np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(x, 1 / 2.4) - 0.055)


def _camera(target, az_deg, el_deg, half_w, half_h, dist=40.0):
    az, el = math.radians(az_deg), math.radians(el_deg)
    c = np.array([math.sin(az) * math.cos(el), math.sin(el), math.cos(az) * math.cos(el)])
    f = -c
    r = np.cross(f, [0.0, 1.0, 0.0])
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    o = np.asarray(target, float) + c * dist
    return np.array([*o, *f, *r, *u, half_w, half_h], float), r, u


class Look:
    """Every appearance choice in one place."""
    width_px = 1660           # output width; shown at half this width on a page
    height_px = 660
    az = 22.0                 # camera azimuth, degrees from the front towards the right
    el = 14.0                 # camera elevation, degrees
    floor_cells = 1           # floor cells shown on each side of the robot's cell
    floor_depth = 0.3         # extrusion of the floor, metres (drawing choice)
    robot_depth = 0.1         # extrusion of each robot cell, metres (drawing choice)
    panel_gap = 0.2           # space between the three panels, metres
    label_band = 0.17         # fraction of the image height kept for labels
    margin = 0.045
    label_size = 15.0         # px at 1x
    label_small = 0.84        # second label line, relative to the first
    label_gap = 20.0          # px at 1x below the lowest floor edge
    muscle = (86, 86, 86)     # horizontal muscle: dark grey
    rigid = (255, 61, 48)     # rigid: chosen so that its front face renders within a few units of #e73121, vermilion
    floor = (232, 232, 232)   # fixed floor cells
    cut_tint = (0.84, 0.84, 0.84)   # the floor's cut face, as a fraction of its colour
    bevel = 0.0035
    seam_width = 0.0016
    seam_dark = 0.55
    spec_k = 0.10
    spec_n = 40.0
    ground_albedo = 0.85
    sky_up = 0.32
    sky_down = 0.30
    light_dir = (-0.55, 0.72, 0.4)      # towards the key light
    light_dist = 5.0
    light_size = 1.2
    key_irradiance = 2.2      # key light irradiance at normal incidence; the sky gives pi * sky_up
    trail_alpha = 0.85
    trail_taper = 0.12        # fraction of the trail over which its oldest end fades in
    shadow_strength = 0.85
    shadow_floor = 0.015      # occlusion below this is treated as none
    shadow_edge = 0.08        # shadows fade out over this fraction of the image height at the border
    exposure = 1.05
    samples = 1024            # paths per pixel in the final images


def build_scene(data, look):
    """Three panels, each a short piece of floor with the robot at one instant
    and its trail. Panels are placed side by side along the screen's horizontal."""
    X, Y, cells = data["X"], data["Y"], data["cells"]
    mats = {1: _lin(look.rigid), 3: _lin(look.muscle)}
    when = find_instants(data)
    steps = [when["land"], when["inside"], when["out"]]
    n = look.floor_cells
    fx0, fx1 = 3.4 - CELL * n, 3.5 + CELL * n
    width = fx1 - fx0
    az = math.radians(look.az)
    along = np.array([math.cos(az), 0.0, -math.sin(az)])   # screen-horizontal direction
    SP, Sprio, Spanel, Skind, Scol, Sseam = [], [], [], [], [], []
    GP, Gcol = [], []
    PB = []
    verts, anchors = [], []
    for p, st in enumerate(steps):
        off = (p - 1) * (width + look.panel_gap) * along
        sx = off[0] - 3.45
        z1 = off[2]
        z0 = z1 - look.floor_depth
        SP.append(_box_planes(fx0 + sx, fx1 + sx, 0.0, FLOOR_TOP, z0, z1))
        Sprio.append(0)
        Spanel.append(p)
        Skind.append(1)
        Scol.append(_lin(look.floor))
        Sseam.append(3.4 + sx)
        lo = np.array([fx0 + sx, 0.0, z0])
        hi = np.array([fx1 + sx, FLOOR_TOP, z1])
        for c in cells:
            idx = c[1:]
            xy = np.stack([X[st, idx] + sx, Y[st, idx]], 1)
            SP.append(_prism_planes(xy, z1 - look.robot_depth, z1))
            Sprio.append(2)
            Spanel.append(p)
            Skind.append(0)
            Scol.append(mats[int(c[0])])
            Sseam.append(0.0)
            lo = np.minimum(lo, [xy[:, 0].min(), xy[:, 1].min(), z1 - look.robot_depth])
            hi = np.maximum(hi, [xy[:, 0].max(), xy[:, 1].max(), z1])
            for q in xy:
                verts.append((q[0], q[1], z1))
                verts.append((q[0], q[1], z1 - look.robot_depth))
        for g in range(1, TRAIL + 1):
            sg = st - g
            for c in cells:
                idx = c[1:]
                xy = np.stack([X[sg, idx] + sx, Y[sg, idx]], 1)
                GP.append(_prism_planes(xy, z1 - look.robot_depth, z1))
                Gcol.append(mats[int(c[0])])
                lo = np.minimum(lo, [xy[:, 0].min(), xy[:, 1].min(), z1 - look.robot_depth])
                hi = np.maximum(hi, [xy[:, 0].max(), xy[:, 1].max(), z1])
                for q in xy:
                    verts.append((q[0], q[1], z1))
        PB.append([lo[0] - 1e-4, hi[0] + 1e-4, lo[1] - 1e-4, hi[1] + 1e-4, lo[2] - 1e-4, hi[2] + 1e-4])
        for x in (fx0 + sx, fx1 + sx):
            for y in (0.0, FLOOR_TOP):
                for z in (z0, z1):
                    verts.append((x, y, z))
        anchors.append(np.array([3.45 + sx, 0.0, z1]))
    # trail: past states with equal weight, as a long exposure records them,
    # tapering over the oldest part so the trail has no hard start
    g = np.arange(1, TRAIL + 1)
    w = np.minimum(1.0, (TRAIL - g + 1) / (look.trail_taper * TRAIL))
    w = w / w.sum()
    cdf = np.cumsum(w)
    cdf[-1] = 1.0 + 1e-9
    gcdf = np.tile(cdf, (3, 1))
    return dict(SP=np.array(SP, float), Sprio=np.array(Sprio, np.int64), Spanel=np.array(Spanel, np.int64),
                Skind=np.array(Skind, np.int64), Scol=np.array(Scol, float), Sseam=np.array(Sseam, float),
                GP=np.array(GP, float), Gcol=np.array(Gcol, float), gcdf=gcdf, PB=np.array(PB, float),
                verts=np.array(verts), anchors=anchors, steps=steps)


def frame_camera(scene, look, W, H):
    cam0, r, u = _camera([0, 0, 0], look.az, look.el, 1, 1)
    V = scene["verts"]
    sx, sy = V @ r, V @ u
    hw = (sx.max() - sx.min()) / 2 + look.margin
    hh = (sy.max() - sy.min()) / 2 + look.margin
    usable = 1.0 - look.label_band
    if hw / hh > W / (H * usable):
        hh = hw * H * usable / W
    else:
        hw = hh * W / (H * usable)
    cx = (sx.min() + sx.max()) / 2
    cy = (sy.min() + sy.max()) / 2
    full_hh = hh / usable
    cy_full = cy - (full_hh - hh)
    target = cx * r + cy_full * u
    cam, r, u = _camera(target, look.az, look.el, hw, full_hh)
    return cam, r, u, target


def project(cam, r, u, target, p, W, H):
    d = np.asarray(p) - target
    x = (d @ r) / cam[12]
    y = (d @ u) / cam[13]
    return (x + 1) / 2 * W, (1 - y) / 2 * H


def make_light(scene, look):
    tc = scene["verts"].mean(0)
    dvec = np.array(look.light_dir, float)
    dvec /= np.linalg.norm(dvec)
    lc = tc + dvec * look.light_dist
    to = -dvec
    lu = np.cross(to, [0.0, 1.0, 0.0])
    lu /= np.linalg.norm(lu)
    lv = np.cross(lu, to)
    lu *= look.light_size
    lv *= look.light_size
    if np.dot(np.cross(lu, lv), to) < 0:
        lu = -lu
    area = look.light_size ** 2
    radiance = look.key_irradiance * look.light_dist ** 2 / area
    return np.array([*lc, *lu, *lv, radiance], float)


def render(data, look, W, H, spp, seed=7):
    scene = build_scene(data, look)
    cam, r, u, target = frame_camera(scene, look, W, H)
    light = make_light(scene, look)
    sky = np.array([look.sky_up, look.sky_down])
    shade = np.array([look.bevel, look.seam_width, look.seam_dark, *look.cut_tint,
                      look.spec_k, look.spec_n, look.ground_albedo], float)
    obj, objn, gnum, gden = render_kernel(W, H, spp, cam, scene["SP"], scene["Sprio"], scene["Spanel"],
                                          scene["Skind"], scene["Scol"], scene["Sseam"], scene["GP"],
                                          scene["Gcol"], look.trail_alpha, scene["gcdf"], scene["PB"],
                                          light, sky, shade, seed)
    return dict(obj=obj, objn=objn, gnum=gnum, gden=gden, spp=spp, scene=scene, cam=cam, r=r, u=u,
                target=target)


def tonemap(c, exposure):
    """Filmic curve (a common fit to the ACES reference transform)."""
    c = c * exposure
    a = 2.51
    b = 0.03
    cc = 2.43
    d = 0.59
    e = 0.14
    return np.clip((c * (a * c + b)) / (c * (cc * c + d) + e), 0.0, 1.0)


def composite(res, look, theme):
    """Blend the objects over the page colour. Where nothing shades the ground
    the page colour is exact, so the image sits seamlessly on the page."""
    cov = res["objn"] / res["spp"]
    col = res["obj"] / np.maximum(res["objn"], 1.0)[..., None]
    col = tonemap(col, look.exposure * theme["exposure"])
    sh = np.where(res["gden"] > 0, res["gnum"] / np.maximum(res["gden"], 1e-12), 1.0)
    occ = (1.0 - sh) * look.shadow_strength
    # the faintest, farthest occlusion is dropped so the open page stays exact,
    # and shadows fade out towards the image border so nothing is cut off there
    t = look.shadow_floor
    occ = np.maximum(occ - t, 0.0) / (1.0 - t)
    H, W = occ.shape
    e = look.shadow_edge * min(H, W)
    yy = np.minimum(np.arange(H), H - 1 - np.arange(H))[:, None] / e
    xx = np.minimum(np.arange(W), W - 1 - np.arange(W))[None, :] / e
    win = np.clip(np.minimum(yy, xx), 0.0, 1.0)
    occ = occ * win * win * (3.0 - 2.0 * win)
    sh = 1.0 - occ
    bg = _lin(theme["bg"])
    ground = bg[None, None, :] * sh[..., None]
    img = cov[..., None] * col + (1.0 - cov[..., None]) * ground
    return (_srgb(img) * 255.0 + 0.5).astype(np.uint8)


THEMES = {
    "light": dict(bg=(255, 255, 255), text=(35, 35, 35), muted=(98, 98, 98), exposure=1.0),
    "dark": dict(bg=(13, 17, 23), text=(236, 236, 236), muted=(151, 151, 151), exposure=0.9),
}


def _font(size):
    """DejaVu Sans, which ships with matplotlib, so the text is the same everywhere."""
    import matplotlib
    path = os.path.join(matplotlib.get_data_path(), "fonts", "ttf", "DejaVuSans.ttf")
    return ImageFont.truetype(path, size)


def _ms(t):
    return "0 ms" if abs(t) < 0.05 else "%.1f ms" % t


def panel_labels(data):
    """Two lines per panel: what is happening, then the time since landing and
    the centre of mass's vertical speed, all read from the recorded states."""
    VY = data["VY"]
    dt = time_step(data)
    when = find_instants(data)
    t = {k: (when[k] - when["land"]) * dt * 1e3 for k in ("land", "inside", "out")}
    v = {k: VY[when[k]].mean() for k in ("land", "inside", "out")}
    return [
        ("lands", "%s, falling at %.1f m/s" % (_ms(t["land"]), -v["land"])),
        ("inside the floor", "%s, still falling at %.0f m/s" % (_ms(t["inside"]), -v["inside"])),
        ("fired out", "%s, rising at %.0f m/s" % (_ms(t["out"]), v["out"])),
    ]


def draw_labels(img, res, look, theme, labels, scale):
    im = Image.fromarray(img)
    dr = ImageDraw.Draw(im)
    W, H = im.size
    f1 = _font(int(round(look.label_size * scale)))
    f2 = _font(int(round(look.label_size * look.label_small * scale)))
    ys = []
    for anchor in res["scene"]["anchors"]:
        ys.append(project(res["cam"], res["r"], res["u"], res["target"], anchor, W, H)[1])
    y0 = max(ys) + look.label_gap * scale
    for anchor, (a, b) in zip(res["scene"]["anchors"], labels):
        x, _ = project(res["cam"], res["r"], res["u"], res["target"], anchor, W, H)
        dr.text((x, y0), a, font=f1, fill=theme["text"], anchor="mt")
        dr.text((x, y0 + look.label_size * 1.45 * scale), b, font=f2, fill=theme["muted"], anchor="mt")
    return im


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--data", default=os.path.join(here, "data"), help="folder holding launch.npz")
    ap.add_argument("--out", default=here, help="folder for the images")
    ap.add_argument("--draft", action="store_true", help="half size, few samples, for quick looks")
    a = ap.parse_args()
    look = Look()
    data = dict(np.load(os.path.join(a.data, "launch.npz")))
    scale = 1.0 if a.draft else 2.0
    W = int(look.width_px * scale / 2)
    H = int(look.height_px * scale / 2)
    spp = 36 if a.draft else look.samples
    res = render(data, look, W, H, spp)
    labels = panel_labels(data)
    os.makedirs(a.out, exist_ok=True)
    for name, theme in THEMES.items():
        img = composite(res, look, theme)
        im = draw_labels(img, res, look, theme, labels, scale)
        im.save(os.path.join(a.out, "inside-%s.png" % name), optimize=True)


if __name__ == "__main__":
    main()

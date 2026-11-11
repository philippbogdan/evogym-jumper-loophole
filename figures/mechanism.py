"""Draw the four-step explanation of the contact loophole as an inline SVG, light and dark.

A schematic, not a plot. In steps 3 and 4 the robot and the tiles are to scale, at the
simulator's own coordinates for integration step 6065. In steps 1 and 2 the lean is drawn
hundreds of times wider than the real 0.06 mm, so it can be seen.

    python figures/mechanism.py            # writes figures/mechanism-light.svg and -dark.svg
"""

import os

HERE = os.path.dirname(os.path.abspath(__file__))

# Vermilion (accent) marks the floor's push: there in step 1, missing in steps 2 and 3,
# four times over in step 4, with the speed it gives the robot. Everything else is ink
# and neutral greys.
THEMES = {
    'light': dict(ink='#232323', soft='#626262', floor='#e8e8e8', floor_line='#b3b3b3', own='#d6d6d6',
                  rigid='#b2b2b2', rigid_line='#4e4e4e', muscle='#595959', muscle_line='#2b2b2b',
                  accent='#e73121', paper='#ffffff'),
    'dark': dict(ink='#ececec', soft='#979797', floor='#1f1f1f', floor_line='#5f5f5f', own='#2b2b2b',
                 rigid='#5e5e5e', rigid_line='#d0d0d0', muscle='#bdbdbd', muscle_line='#ececec',
                 accent='#e73121', paper='#0d1117'),
}

FONT = "-apple-system, BlinkMacSystemFont, 'Segoe UI', 'Noto Sans', Helvetica, Arial, sans-serif"
PW, GAP = 290, 40          # panel width and gap
W, H = 4 * PW + 3 * GAP, 640
TOP, BOT = 70, 500         # drawing band
CELL = 90                  # 0.1 m in the two full views


def text(x, y, s, c, size=21, weight=400, anchor='start'):
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" font-weight="{weight}" fill="{c}" '
            f'text-anchor="{anchor}">{s}</text>')


def arrow_up(x, y_tail, y_tip, c, width=8, head=15, hollow=False, paper='#fff'):
    yb = y_tip + head
    return (f'<line x1="{x:.1f}" y1="{y_tail:.1f}" x2="{x:.1f}" y2="{yb:.1f}" stroke="{c}" stroke-width="{width}"/>'
            f'<polygon points="{x - head * 0.85:.1f},{yb:.1f} {x + head * 0.85:.1f},{yb:.1f} {x:.1f},{y_tip:.1f}" '
            f'fill="{paper if hollow else c}" stroke="{c}" stroke-width="2.5" stroke-linejoin="round"/>')


def arrow_down(x, y_tail, y_tip, c, width=5, head=12):
    yb = y_tip - head
    return (f'<line x1="{x:.1f}" y1="{y_tail:.1f}" x2="{x:.1f}" y2="{yb:.1f}" stroke="{c}" stroke-width="{width}"/>'
            f'<polygon points="{x - head * 0.85:.1f},{yb:.1f} {x + head * 0.85:.1f},{yb:.1f} {x:.1f},{y_tip:.1f}" fill="{c}"/>')


def header(px, n, title, t):
    return (f'<circle cx="{px + 18}" cy="30" r="18" fill="{t["ink"]}"/>'
            + text(px + 18, 38, str(n), t['paper'], 23, 700, 'middle')
            + text(px + 48, 39, title, t['ink'], 25, 650))


def caption(px, lines, t):
    return ''.join(text(px, 545 + 29 * k, s, t['ink'], 21) for k, s in enumerate(lines))


def zoom(px, t, missed):
    """Close-up of one bottom corner where two floor tiles meet."""
    o = []
    left, right = px, px + PW
    bx, sy = px + 150, 290                 # tile boundary and floor surface
    cx = bx - 13 if missed else bx + 34    # the corner: just across the boundary in step 2
    cy = sy + 92
    tx, ty = cx + 44, TOP + 8              # the next corner up: the side leans in
    ex = cx + (tx - cx) * (cy - sy) / (cy - ty)   # where the side crosses the floor's top
    own_l, own_r = (left, bx) if missed else (bx, right)
    o.append(f'<rect x="{left}" y="{sy}" width="{PW}" height="{BOT - sy}" fill="{t["floor"]}"/>')
    o.append(f'<rect x="{own_l}" y="{sy}" width="{own_r - own_l}" height="{BOT - sy}" fill="{t["own"]}"/>')
    o.append(f'<polygon points="{cx},{cy} {tx},{ty} {right},{ty} {right},{cy}" fill="{t["rigid"]}" fill-opacity="0.6"/>')
    o.append(f'<line x1="{left}" y1="{sy}" x2="{right}" y2="{sy}" stroke="{t["floor_line"]}" stroke-width="3"/>')
    o.append(f'<line x1="{own_l}" y1="{sy}" x2="{own_r}" y2="{sy}" stroke="{t["ink"]}" stroke-width="7"/>')
    o.append(f'<line x1="{bx}" y1="{sy}" x2="{bx}" y2="{BOT}" stroke="{t["floor_line"]}" stroke-width="3" stroke-dasharray="9 7"/>')
    o.append(f'<line x1="{cx}" y1="{cy}" x2="{tx}" y2="{ty}" stroke="{t["rigid_line"]}" stroke-width="5" stroke-linecap="round"/>')
    o.append(f'<line x1="{cx}" y1="{cy}" x2="{right}" y2="{cy}" stroke="{t["rigid_line"]}" stroke-width="5"/>')
    o.append(f'<circle cx="{ex:.1f}" cy="{sy}" r="13" fill="none" stroke="{t["ink"]}" stroke-width="3.5"/>')
    o.append(f'<circle cx="{cx}" cy="{cy}" r="10" fill="{t["ink"]}"/>')
    o.append(text(right - 12, ty + 40, 'robot', t['ink'], 21, anchor='end'))
    o.append(text(left + 8, sy - 44, 'the edge', t['soft'], 20))
    o.append(text(left + 8, sy - 18, 'crosses here', t['soft'], 20))
    o.append(f'<line x1="{left + 124}" y1="{sy - 24}" x2="{ex - 16:.1f}" y2="{sy - 6}" stroke="{t["soft"]}" stroke-width="2"/>')
    if missed:
        o.append(text(left + 8, BOT - 14, "the corner's tile", t['ink'], 20, 650))
        o.append(text(cx - 22, cy + 8, 'no push', t['ink'], 24, 700, 'end'))
    else:
        o.append(text(right - 8, BOT - 14, "the corner's tile", t['ink'], 20, 650, 'end'))
        o.append(arrow_up(cx, cy + 70, cy + 16, t['accent'], paper=t['paper']))
        o.append(text(cx + 20, cy + 58, 'push', t['accent'], 24, 700))
    return ''.join(o)


def full(px, t, release):
    """The whole robot and three floor tiles, to scale."""
    o = []
    fx = px + (PW - 3 * CELL) / 2
    sy = 262
    by = sy + CELL                        # the floor's underside, y = 0
    for k in range(3):
        o.append(f'<rect x="{fx + k * CELL:.1f}" y="{sy}" width="{CELL}" height="{CELL}" fill="{t["floor"]}" '
                 f'stroke="{t["floor_line"]}" stroke-width="3"/>')
    x0, x1 = fx + CELL, fx + 2 * CELL
    m = CELL / 0.1
    r_top, r_bot = by - 0.0987 * m, by - 0.0018 * m       # the rigid cell's corners at step 6065
    m_top = by - 0.1887 * m                               # the muscle cell above it
    o.append(f'<rect x="{x0:.1f}" y="{r_top:.1f}" width="{CELL}" height="{r_bot - r_top:.1f}" fill="{t["rigid"]}" '
             f'fill-opacity="0.92" stroke="{t["rigid_line"]}" stroke-width="4"/>')
    o.append(f'<rect x="{x0:.1f}" y="{m_top:.1f}" width="{CELL}" height="{r_top - m_top:.1f}" fill="{t["muscle"]}" '
             f'stroke="{t["muscle_line"]}" stroke-width="4"/>')
    for x in (x0, x1):                                     # the swallowed tile's top corners
        o.append(f'<rect x="{x - 8:.1f}" y="{sy - 8}" width="16" height="16" fill="{t["paper"]}" stroke="{t["ink"]}" stroke-width="3"/>')
    for x in (x0, x1):
        o.append(f'<circle cx="{x:.1f}" cy="{r_bot:.1f}" r="8" fill="{t["ink"]}"/>')
    mid = px + PW / 2
    if not release:
        o.append(text(mid, m_top - 58, '21 m/s', t['ink'], 26, 700, 'middle'))
        o.append(arrow_down(mid, m_top - 46, m_top - 8, t['ink']))
        o.append(text(mid, by + 40, 'no force', t['ink'], 25, 700, 'middle'))
        o.append(text(mid, by + 72, 'robot inside the floor', t['soft'], 20, anchor='middle'))
    else:
        for x in (x0, x1):
            o.append(arrow_up(x, by + 60, by + 12, t['accent'], width=9, head=17))
            o.append(arrow_up(x, by + 106, by + 66, t['accent'], width=9, head=17, hollow=True, paper=t['paper']))
        o.append(text(mid, by + 140, '4 × 214 kN', t['accent'], 25, 700, 'middle'))
        o.append(arrow_up(mid, m_top - 12, m_top - 72, t['accent'], width=6, head=14))
        o.append(text(mid, m_top - 84, '110 m/s', t['ink'], 26, 700, 'middle'))
    return ''.join(o)


def build(theme):
    t = THEMES[theme]
    xs = [k * (PW + GAP) for k in range(4)]
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="{FONT}">']
    o += [header(xs[0], 1, 'The rule', t), zoom(xs[0], t, missed=False),
          caption(xs[0], ['Inside a floor tile, a corner', 'is pushed out only if its own', "edge crosses that tile's top."], t)]
    o += [header(xs[1], 2, 'The miss', t), zoom(xs[1], t, missed=True),
          caption(xs[1], ['A corner sits 0.06 mm inside', 'one tile. Its edge crosses the', 'top of the next. Neither pushes.'], t)]
    o += [header(xs[2], 3, 'Through the floor', t), full(xs[2], t, release=False),
          caption(xs[2], ['The cell falls straight in', 'and stops at the underside', 'of the floor, 9.7 cm deep.'], t)]
    o += [header(xs[3], 4, 'The release', t), full(xs[3], t, release=True),
          caption(xs[3], ['The sides straighten, the rule', 'fires, and the force grows', 'with depth: 110 m/s in 1.2 ms.'], t)]
    o.append('</svg>')
    return '\n'.join(o) + '\n'


if __name__ == '__main__':
    for name in THEMES:
        path = os.path.join(HERE, f'mechanism-{name}.svg')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(build(name))
        print('written', path)

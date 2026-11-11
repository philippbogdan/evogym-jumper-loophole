"""Draw the results figures as plain SVG, light and dark, from the numbers in results/.

    python figures/results.py              # writes figures/score-*.svg, energy-*.svg and climber-*.svg

Each figure makes one comparison. Bars are to scale; nothing is on a log axis.
Vermilion marks the one bar each figure is about, and its number; the others are grey.
"""

import os, csv, json

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FONT = "-apple-system, BlinkMacSystemFont, 'Segoe UI', 'Noto Sans', Helvetica, Arial, sans-serif"

THEMES = {
    'light': dict(ink='#232323', soft='#626262', bar='#9e9e9e', accent='#e73121', line='#d8d8d8'),
    'dark': dict(ink='#ececec', soft='#979797', bar='#5e5e5e', accent='#e73121', line='#434343'),
}


def text(x, y, s, c, size=24, weight=400, anchor='start'):
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" font-weight="{weight}" fill="{c}" '
            f'text-anchor="{anchor}">{s}</text>')


def bars(rows, t, unit='', width=1280, label_w=430, bar_w=640, row_h=92, top=18):
    """Horizontal bars to one linear scale. rows: (label, sublabel, value, formatted value, colour key)."""
    top_value = max(r[2] for r in rows)
    h = top + row_h * len(rows) + 6
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {h}" width="{width}" height="{h}" font-family="{FONT}">']
    x0 = label_w + 24
    o.append(f'<line x1="{x0}" y1="{top}" x2="{x0}" y2="{h - 6}" stroke="{t["line"]}" stroke-width="2"/>')
    for k, (label, sub, value, shown, colour) in enumerate(rows):
        y = top + k * row_h
        w = max(2.0, bar_w * value / top_value)
        o.append(text(label_w, y + 38, label, t['ink'], 26, 600, 'end'))
        o.append(text(label_w, y + 68, sub, t['soft'], 21, 400, 'end'))
        o.append(f'<rect x="{x0}" y="{y + 16}" width="{w:.1f}" height="52" fill="{t[colour]}"/>')
        o.append(text(x0 + w + 16, y + 52, shown + unit, t['accent' if colour == 'accent' else 'ink'], 30, 700))
    o.append('</svg>')
    return '\n'.join(o) + '\n'


def main():
    E = json.load(open(os.path.join(ROOT, 'results', 'expected.json')))['rules']['base']
    released = json.load(open(os.path.join(ROOT, 'results', 'released.json')))
    column = [r for r in csv.DictReader(open(os.path.join(ROOT, 'results', 'released_launchers.csv'))) if r['uid'].startswith('cca8b49a')][0]
    rise = lambda k: max([float(column[k + '_rise_m'])] + [float(v) for v in column[k + '_noise_rise_m'].split()])
    runs = 1 + len(column['released_noise'].split())
    figures = {
        'score': [
            ('Best released robot', f"of {released['jumper_robots']:,}, by the benchmark's authors", released['jumper_best_reward'],
             f"{released['jumper_best_reward']:.2f}", 'bar'),
            ('Best jump found', 'loophole closed', E['best_closed'], f"{E['best_closed']:.2f}", 'bar'),
            ('Two-cell robot', 'simulator as released', E['best'], f"{E['best']:.2f}", 'accent'),
        ],
        'climber': [
            ('Best released Climber-v0 robot', f'as released, highest of {runs} runs: the top', rise('released'), f"{rise('released'):.1f} m", 'accent'),
            ('The same robot', f'loophole closed, highest of {runs} runs', rise('closed'), f"{rise('closed'):.1f} m", 'bar'),
        ],
        'energy': [
            ('The floor', 'work done on the robot', E['terms']['ground normal'], f"{E['terms']['ground normal']:,.0f} J", 'accent'),
            ('Its muscle', 'net work', E['terms']['muscle spring work'], f"{E['terms']['muscle spring work']:,.0f} J", 'bar'),
        ],
    }
    for name, rows in figures.items():
        for theme, t in THEMES.items():
            path = os.path.join(HERE, f'{name}-{theme}.svg')
            with open(path, 'w', encoding='utf-8') as f:
                f.write(bars(rows, t))
            print('written', path)


if __name__ == '__main__':
    main()

"""Recompute every number in results/expected.json from the stored designs and compare.

    python scripts/check.py                 # exact comparison (the reference platform)
    python scripts/check.py --tol 1e-6      # agreement to six significant figures (other platforms)

Prints MATCH or DIFF for each value, writes the local values to results/numbers.json and exits
non-zero if anything differs. The comparisons between Evolution Gym's own environment and the
measuring engine are always exact.
"""

import os, sys, json, glob, subprocess, argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ROOT, RULES, closed, load_design, admissible, engine, engine_module, scene_text
from energy import account

BEST = os.path.join(ROOT, 'designs', 'best-stock', 'design.json')
BEST_CLOSED = os.path.join(ROOT, 'designs', 'best-closed', 'design.json')
rows = []


def gym(tree, path):
    """Replay a design in the chosen tree's own Jumper-v0, in a fresh process."""
    out = subprocess.run([sys.executable, os.path.join(ROOT, 'scripts', 'replay.py'), tree, path],
                         capture_output=True, text=True)
    if out.returncode != 0:
        sys.exit(out.stderr)
    s = out.stdout
    return json.loads(s[s.index('{'):s.rindex('}') + 1])


def report(name, got, want, tol=0.0, fmt='{:.6f}'):
    """Print and record one comparison; return the local value."""
    ok = got == want if tol == 0 else abs(got - want) <= tol * max(1.0, abs(want))
    rows.append((name, ok))
    note = 'exact' if got == want else f'differs by {got - want:+.3g}'
    print(f"{'MATCH' if ok else 'DIFF ':5s}  {name:62s} {fmt.format(got):>16s}   reference {fmt.format(want)} ({note})", flush=True)
    return got


def design_path(rules):
    """The stored best design of a rule setting in the simulator as released."""
    return BEST if rules == 'base' else os.path.join(ROOT, 'designs', 'rule-variants', rules, 'design.json')


def best_closed(rules, candidates, plain=False):
    """The best admissible stored design with the loophole closed, and where it is."""
    R = closed(RULES[rules], plain=plain)
    T = R['T']
    best = (-np.inf, None)
    for p in candidates:
        d, body, conn, A = load_design(p)
        found_under = d['rules'].split(',')[0]
        if R['box'] == 7 and body.shape == (5, 5):
            if conn is not None:
                continue
            b7 = np.zeros((7, 7), int)
            b7[2:7, 1:6] = body
            body = b7
        if rules == 'control' and found_under != 'control':
            A = np.repeat(A, 2, axis=0)
        if A.shape[0] < T or not admissible(body, R, conn, A[:T]):
            continue
        r = float(engine(body, conn, R).rollout(A, T, False)[0])
        if r > best[0]:
            best = (r, os.path.relpath(os.path.dirname(p), ROOT))
    return best


def stray_push(path, rule):
    """The largest contact force on a robot point that is above the floor's surface, over the first command."""
    _, body, conn, A = load_design(path)
    e = engine_module('stock_debug').Engine(scene_text(body, conn), dict(rule['engine'], substeps=1))
    A1 = np.repeat(A, rule['engine'].get('substeps', 30), axis=0)
    worst = (0.0, 0.0, -1)
    for s in range(30):
        e.rollout(A1, s, False)
        py = np.asarray(e.state()[1])
        for p, _, _, _, fx, fy, _ in np.asarray(e.contacts()[0]):
            p = int(p)
            if p < len(py) and py[p] > 0.15 and abs(fy) > worst[0]:
                worst = (float(abs(fy)), float(py[p]), s)
    return worst


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tol', type=float, default=0.0,
                    help='accept abs(local - reference) <= TOL * max(1, abs(reference)); default 0 is exact')
    tol = ap.parse_args().tol
    X = json.load(open(os.path.join(ROOT, 'results', 'expected.json')))
    E = X['rules']
    out = {}

    print('1. The two-cell robot, in Evolution Gym as released and with the loophole closed')
    g = gym('stock', BEST)
    out['best'] = report('two-cell robot, Evolution Gym as released (Jumper-v0)', g['ret'], E['base']['best'], tol)
    out['best_rise_m'] = g['com_height_end'] - g['com_height_start']
    _, body, conn, A = load_design(BEST)
    e = engine(body, conn, RULES['base'])
    report('  same, measuring engine against Evolution Gym', float(e.rollout(A, 500, False)[0]), g['ret'])
    point_steps, depth, _ = e.unpushed()
    out['best_unpushed_depth_m'] = report('  deepest corner inside the floor with no force on it (m)', float(depth), X['unpushed']['best_depth_m'], tol)
    g = gym('closed', BEST)
    out['best_under_closed'] = report('two-cell robot, loophole closed', g['ret'], E['base']['best_under_closed'], tol)
    report('  same, measuring engine against Evolution Gym', float(engine(body, conn, closed(RULES['base'])).rollout(A, 500, False)[0]), g['ret'])
    g = gym('closed', BEST_CLOSED)
    out['best_closed'] = report('best design found with the loophole closed', g['ret'], E['base']['best_closed'], tol)
    out['best_closed_rise_m'] = report('  its centre of mass rises (m)', g['com_height_end'] - g['com_height_start'], X['best_closed_rise_m'], tol)
    _, body, conn, A = load_design(BEST_CLOSED)
    report('  same, measuring engine against Evolution Gym', float(engine(body, conn, closed(RULES['base'])).rollout(A, 500, False)[0]), g['ret'])
    e = engine(body, conn, RULES['base'])
    out['best_closed_in_stock'] = report('  that design in Evolution Gym as released', gym('stock', BEST_CLOSED)['ret'], X['best_closed_in_stock'], tol)
    e.rollout(A, 500, False)
    out['best_closed_unpushed_depth_m'] = report('  its deepest corner with no force on it (m)', float(e.unpushed()[1]), X['unpushed']['best_closed_depth_m'], tol)
    out['share'] = report('share of the best score supplied by the loophole', 1 - out['best_closed'] / out['best'], E['base']['share'], tol, '{:.4f}')

    print('\n2. Where the energy comes from, for the best design of each rule setting')
    out['energy'] = {}
    for rs in RULES:
        acc = account(design_path(rs), rs)
        out['energy'][rs] = acc
        if rs == 'base':
            for k in ['ground normal', 'passive spring work', 'muscle spring work', 'ground damping', 'drag']:
                report(f'  {k} (J)', acc['terms'][k], E['base']['terms'][k], tol, '{:.1f}')
            report('  energy the robot gains (J)', acc['energy_change'], E['base']['robot_gain'], tol, '{:.1f}')
            report('  fewest steps holding 95% of the floor\'s work', acc['release_steps'], E['base']['release_steps'], 0, '{:d}')
            report('  work in those steps (J)', acc['release_work'], E['base']['release_work'], tol, '{:.1f}')
        ok = acc['closure'] < 1e-12
        rows.append((f'energy closure {rs}', ok))
        print(f"{'MATCH' if ok else 'DIFF ':5s}  {'account closes, ' + rs:62s} {acc['closure']:16.2e}   (must be under 1e-12)")

    print('\n3. Sixteen rule settings, as released and with the loophole closed (measuring engine)')
    cands = sorted(glob.glob(os.path.join(ROOT, 'designs', 'closed-search', '*', 'design.json')))
    out['rules'] = {}
    for rs in RULES:
        _, body, conn, A = load_design(design_path(rs))
        R = RULES[rs]
        b = report(f'{rs}: best found', float(engine(body, conn, R).rollout(A, R['T'], False)[0]), E[rs]['best'], tol, '{:.2f}')
        u = report(f'{rs}: that design, loophole closed', float(engine(body, conn, closed(R)).rollout(A, R['T'], False)[0]),
                   E[rs]['best_under_closed'], tol, '{:.2f}')
        c, src = best_closed(rs, cands)
        report(f'{rs}: best found with the loophole closed', c, E[rs]['best_closed'], tol, '{:.2f}')
        out['rules'][rs] = dict(best=b, best_under_closed=u, best_closed=c, best_closed_design=src, share=1 - c / b)

    print('\n4. The plain closing, without the containment check')
    c, src = best_closed('base', cands, plain=True)
    out['plain'] = dict(best_closed=report('best found with the plain closing', c, X['plain']['best_closed'], tol), best_closed_design=src)
    path = os.path.join(ROOT, src, 'design.json')
    report('  same, Evolution Gym with the plain patch against the engine', gym('plain', path)['ret'], c)
    _, body, conn, A = load_design(path)
    out['plain']['same_design_released'] = report('  that design in Evolution Gym as released', gym('stock', path)['ret'], X['plain']['same_design_released'], tol)
    out['plain']['same_design_closed'] = report('  that design with the default closing', float(engine(body, conn, closed(RULES['base'])).rollout(A, 500, False)[0]),
                                                X['plain']['same_design_closed'], tol)
    f, y, s = stray_push(path, closed(RULES['base'], plain=True))
    out['plain'].update(stray_force_N=report('  force it puts on a corner that is above the floor (N)', f, X['plain']['stray_force_N'], tol, '{:.1f}'),
                        stray_height_m=report('  height of that corner (m)', y, X['plain']['stray_height_m'], tol, '{:.3f}'), stray_step=s)
    f, _, _ = stray_push(path, closed(RULES['base']))
    out['plain']['stray_force_default_N'] = report('  the same under the default closing (N)', f, 0.0, 0, '{:.1f}')

    json.dump(out, open(os.path.join(ROOT, 'results', 'numbers.json'), 'w'), indent=1)
    bad = [n for n, ok in rows if not ok]
    print(f"\n{len(rows) - len(bad)} of {len(rows)} match; local values written to results/numbers.json")
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()

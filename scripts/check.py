"""Recompute headline results, compare references and write results/numbers.json."""

import os, sys, json, glob, subprocess, argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ROOT, RULES, closed, load_design, admissible, engine
from energy import account

BEST = os.path.join(ROOT, 'designs', 'best-stock', 'design.json')
BEST_CLOSED = os.path.join(ROOT, 'designs', 'best-closed', 'design.json')
EXPECTED = json.load(open(os.path.join(ROOT, 'results', 'expected.json')))
rows = []


def gym(tree, path):
    """Run an environment replay in a fresh process and read its JSON result."""
    out = subprocess.run([sys.executable, os.path.join(ROOT, 'scripts', 'replay.py'), tree, path],
                         capture_output=True, text=True)
    if out.returncode != 0:
        sys.exit(out.stderr)
    s = out.stdout
    return json.loads(s[s.index('{'):s.rindex('}') + 1])


def same(a, b, tol):
    """Compare exactly, or with tolerance scaled by the reference magnitude."""
    return a == b if tol == 0 else abs(a - b) <= tol * max(1.0, abs(b))


def report(name, got, want, tol=0.0, fmt='{:.6f}'):
    """Print and retain one comparison, then return the measured value."""
    ok = same(got, want, tol)
    rows.append((name, ok))
    exact = 'exact' if got == want else f'differs by {got - want:+.3g}'
    print(f"{'MATCH' if ok else 'DIFF ':5s}  {name:58s} {fmt.format(got):>16s}   reference {fmt.format(want)} ({exact})", flush=True)
    return got


def design_path(rules):
    """Locate the stored open-contact winner for a rule setting."""
    return BEST if rules == 'base' else os.path.join(ROOT, 'designs', 'rule-variants', rules, 'design.json')


def best_closed(rules, candidates):
    """Select the best admissible closed replay, adapting body and action layout."""
    R = closed(RULES[rules])
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


def main():
    """Check stored designs and write local measurements before reporting failure."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--tol', type=float, default=0.0,
                    help='reference tolerance: T * max(1, abs(reference)); default 0 is exact; '
                         'direct engine comparisons and energy-closure checks are unchanged')
    a = ap.parse_args()
    tol = a.tol
    E = EXPECTED['rules']
    out = {}

    print('1. The best design in stock Evolution Gym and with the loophole closed')
    g = gym('stock', BEST)
    out['best_stock'] = report('best design, stock Evolution Gym (Jumper-v0)', g['ret'], E['base']['best'], tol)
    out['best_stock_rise_m'] = g['com_height_end'] - g['com_height_start']
    _, body, conn, A = load_design(BEST)
    report('  same, re-implemented engine', float(engine(body, conn, RULES['base']).rollout(A, 500, False)[0]), g['ret'])
    g = gym('closed', BEST)
    out['best_stock_under_closed'] = report('best design, loophole closed', g['ret'], E['base']['best_under_closed'], tol)
    g = gym('closed', BEST_CLOSED)
    out['best_closed'] = report('best design found with the loophole closed', g['ret'], E['base']['best_closed'], tol)
    _, body, conn, A = load_design(BEST_CLOSED)
    report('  same, re-implemented engine', float(engine(body, conn, closed(RULES['base'])).rollout(A, 500, False)[0]), g['ret'])
    out['best_closed_in_stock'] = gym('stock', BEST_CLOSED)['ret']
    print(f"       (the same design in stock Evolution Gym: {out['best_closed_in_stock']:.6f})")
    out['share'] = report('share of the best return supplied by the loophole', 1 - out['best_closed'] / out['best_stock'],
                          E['base']['share'], tol, '{:.4f}')

    print('\n2. Energy account of each best design')
    out['energy'] = {}
    for rs in RULES:
        acc = account(design_path(rs), rs)
        out['energy'][rs] = acc
        if rs == 'base':
            for k in ['ground normal', 'passive spring work', 'muscle spring work', 'ground damping', 'drag']:
                report(f'  {k} (J)', acc['terms'][k], E['base']['terms'][k], tol, '{:.1f}')
            report('  robot energy gained (J)', acc['energy_change'], E['base']['robot_gain'], tol, '{:.1f}')
            report('  integration steps holding 95% of the ground normal work', acc['release_steps'], E['base']['release_steps'], 0, '{:d}')
            report('  work done in those steps (J)', acc['release_work'], E['base']['release_work'], tol, '{:.1f}')
        ok = acc['closure'] < 1e-12
        rows.append((f'energy closure {rs}', ok))
        print(f"{'MATCH' if ok else 'DIFF ':5s}  {'energy account closes, ' + rs:58s} {acc['closure']:16.2e}   reference {E[rs]['energy_closure']:.2e}")

    print('\n3. Every rule variant, with and without the loophole (re-implemented engine)')
    cands = sorted(glob.glob(os.path.join(ROOT, 'designs', 'closed-search', '*', 'design.json')))
    out['variants'] = {}
    for rs in RULES:
        _, body, conn, A = load_design(design_path(rs))
        R = RULES[rs]
        b = report(f'{rs}: best found', float(engine(body, conn, R).rollout(A, R['T'], False)[0]), E[rs]['best'], tol, '{:.2f}')
        u = float(engine(body, conn, closed(R)).rollout(A, R['T'], False)[0])
        c, src = best_closed(rs, cands)
        report(f'{rs}: best found with the loophole closed', c, E[rs]['best_closed'], tol, '{:.2f}')
        out['variants'][rs] = dict(best=b, best_design_with_loophole_closed=u, best_closed=c, best_closed_design=src,
                                   share=1 - c / b)

    json.dump(out, open(os.path.join(ROOT, 'results', 'numbers.json'), 'w'), indent=1)
    bad = [n for n, ok in rows if not ok]
    print(f"\n{len(rows) - len(bad)} of {len(rows)} match; results written to results/numbers.json")
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()

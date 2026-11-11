"""Tell a jump from a floor launch: replay one design and report where its score comes from.

    python scripts/diagnose.py designs/best-stock/design.json

Prints the score in the unmodified simulator, the score of the same commands with the contact
loophole closed, how deep any corner sat inside the floor with no force on it, and the work done
by the floor against the work done by the muscles.
"""

import os, sys, json, argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ROOT, RULES, closed, load_design, engine
from energy import NAMES


def diagnose(path, rules='base'):
    """Return scores, the deepest unpushed point and the energy terms that separate a jump from a launch."""
    _, body, conn, A = load_design(path)
    R = RULES[rules]
    T = R['T']
    stock = engine(body, conn, R)
    score = float(stock.rollout(A, T, False)[0])
    point_steps, depth, _ = stock.unpushed()
    score_closed = float(engine(body, conn, closed(R)).rollout(A, T, False)[0])
    E, terms, _, _ = stock.account(A, T)
    t = dict(zip(NAMES, np.asarray(terms).sum(0)[:13]))
    return dict(design=os.path.relpath(path, ROOT), rules=rules,
                score=score, score_loophole_closed=score_closed,
                deepest_unpushed_point_m=float(depth), unpushed_point_steps=int(point_steps),
                floor_normal_work_J=float(t['ground normal']), muscle_work_J=float(t['muscle spring work']),
                energy_gained_J=float(E[-1] - E[0]))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('design', help='path to a design.json (body, optional connections, actions)')
    ap.add_argument('--rules', default='base', choices=list(RULES), help='rule setting (default: base)')
    ap.add_argument('--json', action='store_true', help='print JSON instead of the table')
    a = ap.parse_args()
    d = diagnose(os.path.abspath(a.design), a.rules)
    if a.json:
        print(json.dumps(d, indent=1))
    else:
        print(f"score, unmodified simulator        {d['score']:12.2f}")
        print(f"score, loophole closed             {d['score_loophole_closed']:12.2f}")
        print(f"deepest corner with no floor force {d['deepest_unpushed_point_m'] * 100:12.2f} cm")
        print(f"work by the floor's normal force   {d['floor_normal_work_J']:12,.0f} J")
        print(f"net work by the muscles            {d['muscle_work_J']:12,.0f} J")
        print(f"energy the robot gains             {d['energy_gained_J']:12,.0f} J")

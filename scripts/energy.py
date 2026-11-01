"""Account for discrete energy transfers and the main ground-work release."""

import os, sys, json, argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ROOT, RULES, load_design, engine

NAMES = ['muscle spring work', 'passive spring work', 'drag', 'gravity work', 'ground normal', 'ground damping',
         'ground friction', 'self-contact normal', 'self-contact damping', 'self-contact friction', 'ground penalty (y<0)',
         'potential change in RK4 step', 'potential change in projection']
SPLIT = ['muscle reset charge', 'passive reset charge', 'muscle discard', 'passive discard']


def account(path, rules='base'):
    """Return signed work, energy closure and steps carrying most ground work."""
    _, body, conn, A = load_design(path)
    R = RULES[rules]
    T = R['T']
    S = R['engine'].get('substeps', 30)
    E, terms, steps, unstable = engine(body, conn, R).account(A, T)
    terms = np.asarray(terms)
    tot = terms.sum(0)
    main = tot[:13]
    dE = float(E[-1] - E[0])
    largest = float(np.abs(main).max())
    per_step = engine(body, conn, dict(R, engine=dict(R['engine'], substeps=1))).account(np.repeat(A, S, axis=0), T * S)[1]
    gn = np.asarray(per_step)[:, NAMES.index('ground normal')]
    order = np.argsort(-gn)
    k, s = 0, 0.0
    while s < 0.95 * gn.sum() and k < len(gn):
        s += gn[order[k]]
        k += 1
    rel = np.sort(order[:k])
    return dict(rules=rules, design=os.path.relpath(path, ROOT), actions=int(steps), unstable=bool(unstable),
                energy_start=float(E[0]), energy_end=float(E[-1]), energy_change=dE,
                closure=abs(dE - float(main.sum())) / largest,
                terms={n: float(v) for n, v in zip(NAMES, main)},
                spring_split={n: float(v) for n, v in zip(SPLIT, tot[13:17])},
                ground_normal=float(gn.sum()),
                release_steps=int(k), release_work=float(s),
                release_first_step=int(rel.min()), release_last_step=int(rel.max()),
                release_action=int(rel.min()) // S)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('design', help='path to the design.json to account for')
    ap.add_argument('--rules', default='base', choices=list(RULES),
                    help='rule setting for the replay (default: base)')
    a = ap.parse_args()
    print(json.dumps(account(os.path.abspath(a.design), a.rules), indent=1))

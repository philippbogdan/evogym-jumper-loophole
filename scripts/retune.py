"""Search around a stored design's commands for a better score under a chosen contact rule.

A stored design's commands are piecewise constant: a few phases, and muscles that always receive
the same command form a group. This moves each group's value in each phase and the phase
boundaries with CMA-ES, starting from the stored commands, and keeps the best full episode.

    pip install cma
    python scripts/retune.py designs/closed-search/no_hmuscle-01/design.json --out designs/closed-search/base-15 --popsize 30

By default the loophole is closed. It is a local search: what it returns is a design that was
found, not the best possible.
"""

import os, sys, json, argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import RULES, closed, load_design, engine

S = {}      # search state, set up once per process


def setup(design, rules, contact):
    d, body, conn, A0 = load_design(design)
    R0 = RULES[rules]
    R = R0 if contact == 'stock' else closed(R0, plain=(contact == 'plain'))
    T = R['T']
    A0 = A0[:T]
    starts = [0] + [i for i in range(1, T) if not np.array_equal(A0[i], A0[i - 1])]
    columns = {}
    for j in range(A0.shape[1]):
        columns.setdefault(tuple(A0[starts, j]), []).append(j)
    groups = list(columns.values())
    S.update(body=body, conn=conn, R=R, T=T, n=A0.shape[1], groups=groups, K=len(starts),
             V0=np.array([[A0[s, g[0]] for g in groups] for s in starts]), B0=np.array(starts[1:], dtype=float), engine=None)


def commands(x):
    K, G, T = S['K'], len(S['groups']), S['T']
    V = np.clip(np.asarray(x[:K * G]).reshape(K, G), S['R']['lo'], S['R']['hi'])
    B = np.clip(np.round(S['B0'] + np.asarray(x[K * G:]) * 20.0), 1, T - 1).astype(int)     # boundaries move in units of 20 commands
    edges = [0] + list(np.maximum.accumulate(B)) + [T]
    A = np.zeros((T, S['n']))
    for k in range(K):
        for gi, g in enumerate(S['groups']):
            A[edges[k]:edges[k + 1], g] = V[k, gi]
    return A


def loss(x):
    if S['engine'] is None:
        S['engine'] = engine(S['body'], S['conn'], S['R'])
    return -float(S['engine'].rollout(commands(x), S['T'], False)[0])


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('design')
    ap.add_argument('--out', required=True, help='folder for the design that is found')
    ap.add_argument('--rules', default='base', choices=list(RULES))
    ap.add_argument('--contact', default='closed', choices=['closed', 'plain', 'stock'])
    ap.add_argument('--restarts', type=int, default=3)
    ap.add_argument('--sigmas', type=float, nargs='+', default=[0.02, 0.05, 0.1], help='step size of each restart, in turn')
    ap.add_argument('--iterations', type=int, default=100)
    ap.add_argument('--popsize', type=int, default=32)
    ap.add_argument('--workers', type=int, default=4)
    a = ap.parse_args()
    import cma, multiprocessing as mp
    setup(os.path.abspath(a.design), a.rules, a.contact)
    x0 = np.concatenate([S['V0'].ravel(), np.zeros(S['K'] - 1)])
    start = -loss(x0)
    best = (start, x0)
    print(f"{a.design}: {S['K']} phases, {len(S['groups'])} muscle groups; stored commands score {start:.4f} ({a.rules}, {a.contact})", flush=True)
    with mp.get_context('spawn').Pool(a.workers, initializer=setup, initargs=(os.path.abspath(a.design), a.rules, a.contact)) as pool:
        for restart in range(a.restarts):
            es = cma.CMAEvolutionStrategy(best[1], a.sigmas[restart % len(a.sigmas)],
                                          dict(seed=restart + 1, popsize=a.popsize, maxiter=a.iterations, verbose=-9))
            while not es.stop():
                X = es.ask()
                F = pool.map(loss, X)
                es.tell(X, F)
                i = int(np.argmin(F))
                if -F[i] > best[0]:
                    best = (-F[i], np.array(X[i]))
            print(f'restart {restart + 1}: best {best[0]:.4f}', flush=True)
    A = commands(best[1])
    assert -loss(best[1]) == best[0]
    os.makedirs(a.out, exist_ok=True)
    np.save(os.path.join(a.out, 'actions.npy'), A)
    body, conn = S['body'], S['conn']
    json.dump(dict(body=body.tolist(), connections=None if conn is None else conn.tolist(), actions='actions.npy',
                   rules=f"{a.rules}, loophole {'open' if a.contact == 'stock' else 'closed'}"),
              open(os.path.join(a.out, 'design.json'), 'w'), indent=1)
    print(f'written {a.out}: {best[0]:.4f} (stored commands {start:.4f})')

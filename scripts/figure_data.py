"""Export replay geometry, contact records and result summaries as CSV."""

import os, sys, csv, json, argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ROOT, RULES, closed, load_design, engine, engine_module, scene_text
from energy import NAMES

OUT = os.path.join(ROOT, 'figures', 'data')
BEST = os.path.join(ROOT, 'designs', 'best-stock', 'design.json')
BEST_CLOSED = os.path.join(ROOT, 'designs', 'best-closed', 'design.json')
S = 30
CELL = 0.1


def write(name, header, rows):
    """Write a named CSV with a header under the figure-data directory."""
    with open(os.path.join(OUT, name), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def main():
    """Export fresh traces and summaries from the latest numbers.json."""
    os.makedirs(OUT, exist_ok=True)
    _, body, conn, A = load_design(BEST)
    R = RULES['base']
    X, Y, VX, VY, _ = [np.asarray(a) for a in engine(body, conn, R).trace(A, 500)]
    n = X.shape[1]

    x0, y0 = X[0], Y[0]
    cells = []
    for r in range(body.shape[0]):
        for c in range(body.shape[1]):
            if body[r, c] == 0:
                continue
            left, right = 3.2 + c * CELL, 3.2 + (c + 1) * CELL
            top, bottom = 0.1 + (body.shape[0] - r) * CELL, 0.1 + (body.shape[0] - r - 1) * CELL
            def find(px, py):
                """Locate the initial robot point at the requested cell corner."""
                k = int(np.argmin((x0 - px) ** 2 + (y0 - py) ** 2))
                assert abs(x0[k] - px) < 1e-9 and abs(y0[k] - py) < 1e-9
                return k
            cells.append([r * body.shape[1] + c, {1: 'rigid', 2: 'soft', 3: 'horizontal muscle', 4: 'vertical muscle'}[int(body[r, c])],
                          find(left, top), find(right, top), find(left, bottom), find(right, bottom)])
    write('body_cells.csv', ['cell', 'material', 'top_left_point', 'top_right_point', 'bottom_left_point', 'bottom_right_point'], cells)

    write('episode_points_per_action.csv', ['action', 'point', 'x_m', 'y_m'],
          [[t, p, X[t * S, p], Y[t * S, p]] for t in range(501) for p in range(n)])

    acc1 = engine(body, conn, dict(R, engine=dict(R['engine'], substeps=1))).account(np.repeat(A, S, axis=0), 500 * S)
    terms = np.asarray(acc1[1])
    gn = terms[:, NAMES.index('ground normal')]
    s0, s1 = 6030, 6110
    write('release_points_per_step.csv', ['step', 'action', 'point', 'x_m', 'y_m', 'vx_m_per_s', 'vy_m_per_s'],
          [[s, s // S, p, X[s, p], Y[s, p], VX[s, p], VY[s, p]] for s in range(s0, s1 + 1) for p in range(n)])
    write('ground_normal_work_per_step.csv', ['step', 'action', 'ground_normal_work_J', 'cumulative_J'],
          [[s, s // S, gn[s], gn[: s + 1].sum()] for s in range(len(gn))])

    dbg = engine_module('stock_debug')
    e1 = dbg.Engine(scene_text(body, conn), dict(R['engine'], substeps=1))
    A1 = np.repeat(A, S, axis=0)
    ea, eb = [np.asarray(a) for a in e1.edges()][:2]
    rows = []
    for s in range(6060, 6082):
        e1.rollout(A1, s, False)
        px, py = [np.asarray(a) for a in e1.state()][:2]
        rec = np.asarray(e1.contacts()[0])
        for p, cell, e, dist, fx, fy, _ in rec:
            p, e = int(p), int(e)
            if p < n:
                kind = 'robot point inside floor cell, pushed along that cell edge'
            else:
                kind = 'floor point inside robot cell, robot edge pushed back'
            rows.append([s, s // S, kind, p, int(cell), e, int(ea[e]) if e < len(ea) else '', int(eb[e]) if e < len(eb) else '',
                         dist, fx, fy] + [py[k] for k in range(n)])
    write('release_contacts.csv', ['step', 'action', 'contact', 'point', 'cell_grid_index', 'edge', 'edge_point_a', 'edge_point_b',
                                   'depth_m', 'normal_force_x_N', 'normal_force_y_N'] + [f'robot_point_{k}_y_m' for k in range(n)], rows)

    ys = {}
    for label, path, rules in [('best design, stock', BEST, R), ('best design, loophole closed', BEST, closed(R)),
                               ('best loophole-closed design, loophole closed', BEST_CLOSED, closed(R))]:
        _, b, c, a = load_design(path)
        ys[label] = np.asarray(engine(b, c, rules).rollout(a, 500, True)[3])
    write('com_height_per_action.csv', ['action'] + list(ys), [[t] + [ys[k][t] for k in ys] for t in range(501)])

    nums = json.load(open(os.path.join(ROOT, 'results', 'numbers.json')))
    acc = nums['energy']['base']
    write('energy_terms_best_design.csv', ['term', 'work_J'], [[k, v] for k, v in acc['terms'].items() if abs(v) > 1.0])
    change = {'base': 'the task as released', 'box7': 'box 7x7', 'no_rigid': 'no rigid cells', 'no_soft': 'no soft cells',
              'no_hmuscle': 'no horizontal muscles', 'no_vmuscle': 'no vertical muscles', 'joins': 'any subset of joins',
              'symmetric': 'mirror-symmetric body and actions', 'range': 'muscle range [0.475, 1.725]',
              'control': '1000 actions of 15 steps', 'horizon': '1000 actions', 'sideways': 'sideways penalty 0',
              'instability': 'instability never checked', 'strain': 'strain limits 50% / rigid 6%',
              'dt': 'step 5e-5, 60 per action', 'contact': 'contact forces at every RK stage'}
    write('rule_variants.csv', ['rules', 'change', 'best_found', 'best_found_loophole_closed', 'share_from_loophole',
                                'best_design_with_loophole_closed'],
          [[rs, change[rs], v['best'], v['best_closed'], v['share'], v['best_design_with_loophole_closed']]
           for rs, v in nums['variants'].items()])
    print('written', OUT)


if __name__ == '__main__':
    argparse.ArgumentParser(description=__doc__).parse_known_args()
    main()

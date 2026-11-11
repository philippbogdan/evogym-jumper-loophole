"""Shared rule definitions, design loading and scene construction for Jumper-v0."""

import os, sys, json, glob, importlib.util
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(ROOT, 'build')
GROUND = open(os.path.join(ROOT, 'engine', 'ground_scene.txt')).read()

BASE = dict(T=500, lo=0.6, hi=1.6, side=5.0, box=5, forbid=(), joins=False, symmetric=False)


def _rule(engine=None, **kw):
    """Combine the base task rules with overrides and engine parameters."""
    d = dict(BASE)
    d.update(kw)
    d['engine'] = dict(engine or {})
    d['engine'].update(lo=d['lo'], hi=d['hi'], side=d['side'])
    return d


RULES = {
    'base': _rule(),
    'box7': _rule(box=7),
    'no_rigid': _rule(forbid=(1,)),
    'no_soft': _rule(forbid=(2,)),
    'no_hmuscle': _rule(forbid=(3,)),
    'no_vmuscle': _rule(forbid=(4,)),
    'joins': _rule(joins=True),
    'symmetric': _rule(symmetric=True),
    'range': _rule(lo=0.475, hi=1.725),
    'control': _rule(dict(substeps=15), T=1000),
    'horizon': _rule(T=1000),
    'sideways': _rule(side=0.0),
    'instability': _rule(dict(check_instability=False)),
    'strain': _rule(dict(thresh=0.5, thresh_rigid=0.06)),
    'dt': _rule(dict(dt=0.00005, substeps=60)),
    'contact': _rule(dict(contact_per_stage=True)),
}


def closed(rule, plain=False):
    """Copy a rule setting with the loophole closed, as patches/close-loophole.patch does.

    plain=True is patches/close-loophole-plain.patch instead: the floor cell's top edge, without
    the check that the point really lies within the cell."""
    d = dict(rule)
    d['engine'] = dict(rule['engine'], nohide=True, plain=bool(plain))
    return d


def load_design(path):
    """Load metadata, body, connections and actions from a design JSON file."""
    d = json.load(open(path))
    body = np.array(d['body'])
    conn = None if d.get('connections') is None else np.array(d['connections'])
    acts = d['actions']
    A = np.load(os.path.join(os.path.dirname(path), acts)) if isinstance(acts, str) else np.array(acts, dtype=np.float64)
    return d, body, conn, A


def full_connectivity(body):
    """List joins between occupied side neighbours in flattened cell order."""
    body = np.asarray(body)
    R, C = body.shape
    out = []
    for i in range(R * C):
        x, y = i % C, i // C
        if body[y][x] == 0:
            continue
        if x + 1 < C and body[y][x + 1] != 0:
            out.append((i, i + 1))
        if y + 1 < R and body[y + 1][x] != 0:
            out.append((i, i + C))
    return out


def connected(body, pairs):
    """Check that the supplied joins connect every occupied cell."""
    cells = [i for i, v in enumerate(np.asarray(body).flatten()) if v]
    adj = {c: [] for c in cells}
    for a, b in pairs:
        adj[a].append(b)
        adj[b].append(a)
    seen = {cells[0]}
    st = [cells[0]]
    while st:
        u = st.pop()
        for w in adj[u]:
            if w not in seen:
                seen.add(w)
                st.append(w)
    return len(seen) == len(cells)


def mirror_ok(body, A):
    """Check equality of actions for left-right mirror muscle pairs."""
    W = body.shape[1]
    idx = [i for i, v in enumerate(body.flatten()) if v in (3, 4)]
    pos = {g: k for k, g in enumerate(idx)}
    mir = [pos[g - g % W + (W - 1 - g % W)] for g in idx]
    return np.array_equal(A[:, :len(idx)], A[:, mir])


def admissible(body, rule, conn=None, A=None):
    """Check body, join and symmetry restrictions for the selected rules."""
    body = np.asarray(body)
    if body.shape != (rule['box'], rule['box']):
        return False
    if not np.isin(body, [0, 1, 2, 3, 4]).all() or not np.isin(body, [3, 4]).any():
        return False
    if any(np.any(body == m) for m in rule['forbid']):
        return False
    if rule['symmetric']:
        if not np.array_equal(body, np.fliplr(body)):
            return False
        if A is not None and not mirror_ok(body, A):
            return False
    pairs = full_connectivity(body)
    if not pairs and (body != 0).sum() > 1:
        return False
    if (body != 0).sum() == 1:
        return conn is None
    if conn is not None:
        if not rule['joins']:
            return False
        allowed = set(pairs)
        sub = [tuple(sorted(map(int, p))) for p in np.asarray(conn).T]
        if any(p not in allowed for p in sub):
            return False
        pairs = sub
    return connected(body, pairs)


def scene_text(body, conn=None, x=32, y=1, world=(60, 8)):
    """Describe the robot and fixed ground in the engine scene format."""
    body = np.asarray(body, dtype=int)
    R, C = body.shape
    pairs = full_connectivity(body) if conn is None else [tuple(map(int, p)) for p in np.asarray(conn).T]
    row0, row1 = [], []
    for a, b in pairs:
        row0 += [a, b]
        row1 += [b, a]
    act = [i for i, v in enumerate(body.flatten()) if v in (3, 4)]
    s = f"init {world[0]} {world[1]}\n"
    s += f"robot robot {x} {y} {R} {C} " + " ".join(str(int(v)) for v in body.flatten()) + "\n"
    s += f"conn {len(row0)} " + " ".join(repr(float(v)) for v in row0 + row1) + "\n"
    s += GROUND
    s += f"actuators {len(act)} " + " ".join(map(str, act)) + "\n"
    return s


_mods = {}


def engine_module(tree='stock'):
    """Load and cache the compiled extension for a named build tree."""
    if tree in _mods:
        return _mods[tree]
    name = f'jumpengine_{tree}'
    so = glob.glob(os.path.join(BUILD, tree, name + '.*.so'))
    if not so:
        sys.exit(f'{name} not built: run scripts/setup.sh first')
    spec = importlib.util.spec_from_file_location(name, so[0])
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    _mods[tree] = m
    return m


def engine(body, conn, rule, tree='stock'):
    """Construct an engine with the supplied body, joins and rule parameters."""
    return engine_module(tree).Engine(scene_text(body, conn), rule['engine'])


if __name__ == '__main__':
    import argparse
    argparse.ArgumentParser(description=__doc__,
                            epilog='Library module; use replay.py, energy.py, check.py or figure_data.py to run an analysis.').parse_known_args()

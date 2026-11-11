"""Every number quoted on the front page and in docs/, recomputed from the tables in this repository.

No simulator is needed: these tests read results/ and figures/data/ only.

    python -m pytest tests
"""

import csv, json, os, re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def table(*parts):
    return list(csv.DictReader(open(os.path.join(ROOT, *parts))))


def text(*parts):
    return open(os.path.join(ROOT, *parts), encoding='utf-8').read()


X = json.load(open(os.path.join(ROOT, 'results', 'expected.json')))
RULES = X['rules']
BASE = RULES['base']
CONTACTS = table('figures', 'data', 'release_contacts.csv')
POINTS = table('figures', 'data', 'release_points_per_step.csv')
RELEASED = json.load(open(os.path.join(ROOT, 'results', 'released.json')))
README = text('README.md')
EVIDENCE = text('docs', 'evidence.md')


def mean(v):
    return sum(v) / len(v)


def sd(v):
    m = mean(v)
    return (sum((x - m) ** 2 for x in v) / (len(v) - 1)) ** 0.5


def median(v):
    v = sorted(v)
    return v[len(v) // 2] if len(v) % 2 else (v[len(v) // 2 - 1] + v[len(v) // 2]) / 2


def points(step):
    """x, y, vx, vy of the six points at one step; points 2, 3 are the bottom cell's top corners, 4, 5 its bottom corners."""
    rows = sorted((r for r in POINTS if r['step'] == str(step)), key=lambda r: int(r['point']))
    return [(float(r['x_m']), float(r['y_m']), float(r['vx_m_per_s']), float(r['vy_m_per_s'])) for r in rows]


def speed(step):
    return sum(p[3] for p in points(step)) / 6


def pushes(step, kind='robot point'):
    return [r for r in CONTACTS if r['step'] == str(step) and r['contact'].startswith(kind)]


# ---------------------------------------------------------------- the front page

def test_headline_scores():
    assert f"{BASE['best']:.2f}" == '514.48'
    assert f"{BASE['best_closed']:.2f}" == '13.83'
    assert f"{BASE['best_under_closed']:.2f}" == '-1.99'
    share = 1 - BASE['best_closed'] / BASE['best']
    assert abs(share - BASE['share']) < 1e-12
    assert f'{100 * share:.1f}' == '97.3' and round(100 * share) == 97
    assert round(BASE['best'] / BASE['best_closed']) == 37
    assert X['best_closed_in_stock'] == BASE['best_closed']               # the best honest jump scores the same patched or not, on x86-64 Linux
    assert f"{X['best_closed_in_stock_apple_silicon_wheel']:.2f}" == '58.52'   # and is thrown as released in the shipped wheel on Apple silicon
    assert X['unpushed']['best_closed_depth_m'] == 0.0
    assert f"{X['best_closed_rise_m']:.2f}" == '1.40'
    assert f"{100 * X['unpushed']['best_depth_m']:.2f}" == '9.98'
    for quoted in ('scores 514.48 on Jumper-v0', 'An estimated 97%', 'the best design found scores 13.83', 'then scores -1.99',
                   'jumps 1.40 m and scores 13.83, and the same unpatched on x86-64 Linux', 'Apple silicon the floor throws it too, for 58.52',
                   '51.45 m', '59,641 J', '450 J', '214,000 N'):
        assert quoted in README, quoted


def test_best_closed_design():
    here = lambda *p: open(os.path.join(ROOT, 'designs', *p), 'rb').read()
    assert BASE['best_closed_design'] == 'designs/closed-search/base-15'
    assert here('best-closed', 'actions.npy') == here('closed-search', 'base-15', 'actions.npy')
    body = json.loads(here('best-closed', 'design.json'))['body']
    assert [row for row in body] == [[0, 0, 4, 0, 0]] * 5                 # a column of five vertical muscles
    stock = json.loads(here('best-stock', 'design.json'))['body']
    assert sorted(v for row in stock for v in row if v) == [1, 3]         # a horizontal muscle and a rigid cell
    assert len(os.listdir(os.path.join(ROOT, 'designs', 'closed-search'))) == 35
    variants = sorted(os.listdir(os.path.join(ROOT, 'designs', 'rule-variants')))
    assert len(variants) == 15
    for name in variants:                                                 # every winner is a two-cell column
        cells = [(y, x) for y, row in enumerate(json.loads(here('rule-variants', name, 'design.json'))['body']) for x, v in enumerate(row) if v]
        assert len(cells) == 2 and cells[0][1] == cells[1][1] and cells[1][0] == cells[0][0] + 1


def test_plain_patch():
    P = X['plain']
    assert f"{P['best_closed']:.2f}" == '14.20' and P['best_closed_design'] == 'designs/closed-search/instability-01'
    assert f"{P['same_design_released']:.2f}" == '12.22' and P['same_design_closed'] == P['same_design_released']
    assert f"{P['best_closed'] - P['same_design_released']:.2f}" == '1.98'
    assert f"{P['stray_force_N']:,.0f}" == '220,500' and P['stray_step'] == 0
    assert abs(P['stray_height_m'] - 0.2) < 1e-12                         # 10 cm above the floor's surface at 0.1 m
    assert abs(2.1e6 * (0.005 + 0.1) - P['stray_force_N']) < 1e-6         # the contact law at a depth of one whole cell
    for quoted in ('220,500 N', '10 cm above', 'scores 14.20', 'scores 12.22'):
        assert quoted in README, quoted
    for quoted in ('220,500 N', '14.20', '12.22', '1.98'):
        assert quoted in EVIDENCE, quoted


def test_rise():
    rows = table('figures', 'data', 'com_height_per_action.csv')
    rise = lambda k: float(rows[-1][k]) - float(rows[0][k])
    assert f"{rise('best design, stock'):.2f}" == '51.45'
    assert abs(rise('best design, stock') - X['best_rise_m']) < 1e-9
    assert abs(rise('best loophole-closed design, loophole closed') - X['best_closed_rise_m']) < 1e-9
    assert abs(10 * rise('best design, stock') - BASE['best']) < 0.05      # the score is ten times the rise, less the sideways penalty
    assert round(rise('best design, stock') / 0.2) == 257                 # in units of the robot's own height


def test_energy_account():
    t = BASE['terms']
    assert f"{t['ground normal']:,.0f}" == '59,641'
    assert f"{t['muscle spring work']:,.0f}" == '450'
    assert f"{t['passive spring work']:,.0f}" == '-17,740'
    assert f"{t['ground damping']:,.0f}" == '-6,057'
    assert f"{t['ground friction']:,.0f}" == '-13'
    assert f"{t['ground penalty (y<0)']:,.0f}" == '146'
    assert f"{t['drag']:,.0f}" == '-2,323'
    assert f"{BASE['robot_gain']:,.0f}" == '34,105'
    assert abs(sum(t.values()) - BASE['robot_gain']) < 5.0                # terms under 1 J are not listed
    assert BASE['energy_closure'] < 2.5e-16
    assert BASE['release_steps'] == 12 and BASE['release_first_step'] == 6066 and BASE['release_last_step'] == 6077
    assert f"{BASE['release_work']:,.0f}" == '56,883'
    assert round(100 * BASE['release_work'] / t['ground normal']) == 95
    assert all(v['energy_closure'] < 2e-15 for v in RULES.values())


# ---------------------------------------------------------------- docs/mechanism.md

def test_the_miss():
    p = points(6003)
    assert f'{-speed(6002):.1f}' == '22.6'
    assert f'{(3.4 - p[4][0]) * 1e6:.0f}' == '66' and f'{(p[5][0] - 3.5) * 1e6:.0f}' == '66'       # bottom corners outside
    assert f'{(p[2][0] - 3.4) * 1e6:.0f}' == '727' and f'{(3.5 - p[3][0]) * 1e6:.0f}' == '727'     # top corners inside
    assert p[4][1] < 0.1 < points(6002)[4][1]                                                      # it meets the floor at 6003


def test_through_the_floor():
    for step, force in ((6003, '10,698'), (6004, '15,194')):
        rows = pushes(step) + pushes(step, 'floor point')
        up = [r for r in rows if abs(float(r['normal_force_y_N'])) > 1e3]
        assert len(up) == 4 and all(f"{abs(float(r['normal_force_y_N'])):,.0f}" == force for r in up)
    assert f'{-speed(6005):.1f}' == '20.5'
    assert f'{(0.1 - points(6005)[4][1]) * 1e3:.0f}' == '4'
    assert not [r for r in CONTACTS if 6005 <= int(r['step']) <= 6051]                             # no contact force at all
    assert all(20.5 <= -speed(s) <= 21.1 for s in range(6005, 6052))
    assert points(6051)[4][1] > 0 > points(6052)[4][1]                                             # bottom corners pass the underside
    assert points(6051)[2][1] > 0.1 > points(6052)[2][1]                                           # top corners reach the surface
    assert {r['point'] for r in pushes(6052)} == {'2', '3'}
    assert f'{-min(points(s)[4][1] for s in range(6052, 6066)) * 1e3:.1f}' == '6.4'
    assert f'{speed(6066):.1f}' == '12.9'


def test_the_release():
    p = points(6066)
    assert f'{(3.4 - p[2][0]) * 1e6:.1f}' == '7.3' and p[2][1] > 0.1                               # top corners back above, outside
    big = [r for r in CONTACTS if r['step'] == '6066' and abs(float(r['normal_force_y_N'])) > 2e5]
    assert len(big) == 4
    for r in big:
        assert f"{abs(float(r['normal_force_y_N'])):,.0f}" == '213,965'
        assert f"{100 * float(r['depth_m']):.2f}" == '9.69'
        assert abs(2.1e6 * (0.005 + float(r['depth_m'])) - abs(float(r['normal_force_y_N']))) < 1e-6
    assert sum(r['contact'].startswith('robot point') for r in big) == 2
    assert sum(r['contact'].startswith('floor point') for r in big) == 2
    assert 2 * 213965 == 427930
    assert all(abs(float(r['normal_force_y_N'])) < 2e4 for r in CONTACTS if r['step'] == '6065')   # nothing of the kind a step earlier
    lowest = lambda s: min(q[1] for q in points(s))
    assert lowest(6077) < 0.1 < lowest(6078)                                                       # clear of the floor twelve steps later
    assert f'{speed(6078):.1f}' == '110.3'
    assert f'{max(q[3] for s in range(6066, 6090) for q in points(s)[4:]):.1f}' == '124.7'


def test_pencil_estimate():
    one = 2.1e6 * (0.005 * 0.1 + 0.1 ** 2 / 2)
    assert round(one) == 11550 and round(4 * one) == 46200
    assert f"{BASE['robot_gain'] / (6 * 110):.1f}" == '51.7'
    assert round(100 * (1 - BASE['robot_gain'] / BASE['terms']['ground normal'])) == 43
    assert f"{RULES['contact']['release_work']:,.0f}" == '48,143'


# ---------------------------------------------------------------- docs/rules.md

def test_rule_settings():
    assert len(RULES) == 16
    shares = [v['share'] for v in RULES.values()]
    doc = text('docs', 'rules.md')
    assert f'The share runs from {100 * min(shares):.1f}% to {100 * max(shares):.1f}%.' in doc
    assert f'with {100 * min(shares):.1f}% to {100 * max(shares):.1f}% of its score from the loophole' in README
    assert f"{RULES['no_vmuscle']['best_closed']:.2f}" == '0.70' and f"{RULES['box7']['best_closed']:.2f}" == '17.36'
    assert all(v['best'] > 400 for v in RULES.values())
    assert all(v['best_under_closed'] < 0 for v in RULES.values())
    assert f"{RULES['contact']['best']:.2f}" == '419.08'
    assert round(100 * (1 - RULES['contact']['best'] / BASE['best'])) == 19
    for name, v in RULES.items():                                         # every row of the table
        row = [line for line in doc.splitlines() if line.startswith(f'| `{name}` |')][0]
        cells = [c.strip() for c in row.strip('|').split('|')]
        assert cells[2:] == [f"{v['best']:.2f}", f"{v['best_under_closed']:.2f}", f"{v['best_closed']:.2f}", f"{100 * v['share']:.1f}%"]


# ---------------------------------------------------------------- the other tasks

def test_other_tasks():
    rows = {r['task']: r for r in table('results', 'other_tasks.csv')}
    assert len(rows) == 32 == len(RELEASED['tasks'])
    rise = lambda r: float(r['height_max_m']) - float(r['height_start_m'])
    thrown = [t for t, r in rows.items() if float(r['deepest_unpushed_m']) > 0.04 and rise(r) > 1]     # sinks unpushed, and rises
    assert len(thrown) == 22                                              # the floor throws it in 22 of the 32
    assert all(float(r['deepest_unpushed_m']) == 0 and rise(r) == 0 for t, r in rows.items() if t not in thrown)
    assert all(float(r['closed_height_max_m']) == float(r['height_start_m']) for r in rows.values())   # closed: never above its start
    better = sorted(t for t, r in rows.items() if float(r['reward']) > float(r['best_released_reward']))
    assert better == ['Climber-v0', 'Climber-v1', 'Jumper-v0']
    j, c0, c1 = rows['Jumper-v0'], rows['Climber-v0'], rows['Climber-v1']
    assert float(j['reward']) == BASE['best'] and float(j['closed_reward']) == BASE['best_under_closed']
    assert (f"{float(c0['reward']):.2f}", c0['commands'], c0['commands_allowed'], c0['ended_early']) == ('9.45', '230', '400', '1')
    assert (f"{float(c1['reward']):.2f}", c1['commands'], c1['commands_allowed'], c1['ended_early']) == ('7.32', '223', '600', '1')
    assert float(c0['height_end_m']) > 8.6 and float(c1['height_end_m']) > 6.5        # past each task's finish line
    assert f"{float(c0['closed_reward']):.2f}" == '-0.20' == f"{float(c1['closed_reward']):.2f}"
    assert (c0['released_robots'], f"{float(c0['best_released_reward']):.2f}", f"{float(c0['second_best_released_reward']):.2f}") == ('3417', '9.27', '5.49')
    assert (c1['released_robots'], f"{float(c1['best_released_reward']):.2f}") == ('2162', '4.13')
    # a robot that reaches the top is paid its climb plus 1, and starts with its centre of mass at most 0.55 m up
    assert float(c1['best_released_reward']) < 6.5 - 0.55 + 1             # no released Climber-v1 robot reaches the top
    assert float(c0['second_best_released_reward']) < 8.6 - 0.55 + 1 < float(c0['best_released_reward'])   # one Climber-v0 robot does
    for quoted in ('the floor throws it in 22', 'In three it outscores every robot', '9.45, at the top by command 230 of 400',
                   '7.32, at the top by command 223 of 600', '9.27, of 3,417', 'the next scores 5.49',
                   '4.13, of 2,162; none reaches the top', '10.17, of 2,578'):
        assert quoted in README, quoted


# ---------------------------------------------------------------- who stands on it

def test_released_dataset():
    assert RELEASED['robots'] == 90563 and RELEASED['robots_with_policies'] == 2553
    assert RELEASED['jumper_robots'] == 2578 == RELEASED['tasks']['Jumper-v0']['robots']
    assert RELEASED['jumper_robots_with_policies'] == 78
    assert f"{RELEASED['jumper_best_reward']:.2f}" == '10.17' and RELEASED['jumper_best_generated_by'] == 'CPPN-NEAT'
    assert f"{RELEASED['jumper_by_method']['Genetic Algorithm']['best_reward']:.2f}" == '5.43'
    assert sum(t['robots'] for t in RELEASED['tasks'].values()) == 90563


def test_released_jumper_robots():
    t = table('results', 'released_jumper.csv')
    assert len(t) == 78
    num = lambda r, k: float(r[k])
    runs = lambda r, k: [num(r, k)] + [float(v) for v in r[k + '_perturbed'].split()]
    assert all(len(runs(r, 'released')) == 9 == len(runs(r, 'closed')) for r in t)
    assert sum(abs(num(r, 'released') - num(r, 'dataset_reward')) < 0.001 for r in t) == 72        # the replay is the authors' rollout
    assert f"{max(abs(num(r, 'released') - num(r, 'dataset_reward')) for r in t):.2f}" == '0.10'
    released, closed = [mean(runs(r, 'released')) for r in t], [mean(runs(r, 'closed')) for r in t]
    assert (f'{mean(released):.2f}', f'{mean(closed):.2f}') == ('3.40', '3.38')
    diff = [a - b for a, b in zip(released, closed)]
    assert (f'{mean(diff):.3f}', f'{sd(diff) / len(diff) ** 0.5:.3f}') == ('0.016', '0.049')
    assert f"{median([sd(runs(r, 'released')) for r in t]):.2f}" == '0.29'                       # what one part in a million does
    assert f"{max(max(runs(r, 'released')) for r in t):.2f}" == '7.61' and f"{max(max(runs(r, 'closed')) for r in t):.2f}" == '7.93'
    assert f"{1000 * max(num(r, 'deepest_unpushed_m') for r in t):.2f}" == '12.59'
    assert sum(int(r['unpushed_point_steps']) > 0 for r in t) == 37
    assert all(r['engine_replay'] == r['released'] for r in t)                                    # the engine against the environment, exactly
    assert all(num(r, 'floor_normal_work_J') > num(r, 'energy_gained_J') for r in t)
    for quoted in ('None of the authors\' 2,578 scores passes 10.17', 'For 78 they released the controller',
                   'those 78 average 3.40 and 3.38, a difference of 0.02 ± 0.05'):
        assert quoted in README, quoted


def test_baseline_rerun():
    robots = table('results', 'baseline_robots.csv')
    assert len(robots) == 1345
    assert sorted(sum(r['seed'] == s for r in robots) for s in '012') == [267, 328, 750]
    assert f"{max(float(r['fitness']) for r in robots):.2f}" == '3.66'
    ep = table('results', 'baseline_episodes.csv')
    assert len(ep) == 400 and all(r['episode'] == 'deterministic' for r in ep)
    fittest = sorted((float(r['fitness']) for r in robots), reverse=True)[:400]
    assert sorted((float(r['fitness']) for r in ep), reverse=True) == fittest                       # the 400 fittest bodies
    assert f"{1000 * max(float(r['deepest_unpushed_m']) for r in ep):.2f}" == '18.74'
    assert sum(int(r['unpushed_point_steps']) > 0 for r in ep) == 138
    assert all(r['engine_replay'] == r['released'] for r in ep)
    assert 2578 + 1345 == 3923
    for quoted in ('on 1,345 bodies, tops out at 3.66', 'in 3,923 robots, never on the jump'):
        assert quoted in README, quoted


def test_published_scores():
    t = table('results', 'published_scores.csv')
    papers = [r for r in t if 'dataset' not in r['source']]
    assert f"{max(float(r['score']) for r in papers if r['body'] == '5x5'):.2f}" == '9.17'
    assert f"{max(float(r['score']) for r in papers):.2f}" == '14.50' and max(papers, key=lambda r: float(r['score']))['body'] == '7x7'
    assert max(float(r['score']) for r in t if r['body'] == '5x5') < BASE['best_closed']            # nothing published on the standard body passes an honest jump
    assert max(float(r['score']) for r in t) < RULES['box7']['best_closed']
    assert 'The highest score in a paper is 9.17, or 14.50 on a 7 × 7 body, where a jump with the loophole closed reaches 17.36' in README
    for r in t:                                                           # every row is on the evidence page
        assert f"{float(r['score']):.2f}".rstrip('0').rstrip('.') in EVIDENCE or f"{float(r['score']):.2f}" in EVIDENCE, r['source']


def test_released_scan():
    t = table('results', 'released_scan.csv')
    assert len(t) == 2553 and not any(r['error'] for r in t) and len({r['task'] for r in t}) == 32
    depth = lambda r: 1000 * float(r['deepest_unpushed_m'])
    climbs = ('Climber-v0', 'Climber-v1', 'Climber-v2')
    assert f"{max(depth(r) for r in t if r['task'] not in climbs):.2f}" == '17.11'               # the other 29 tasks
    deep = [r for r in t if depth(r) > 20]
    assert len(deep) == 14 and all(r['task'] in climbs for r in deep)
    assert sorted(r['task'] for r in t if depth(r) > 50) == ['Climber-v0'] * 2 + ['Climber-v1'] * 4
    assert sorted(r['uid'][:8] for r in t if depth(r) > 50 and r['task'] == 'Climber-v0') == ['51f55c20', 'cca8b49a']
    assert sum(r['task'] == 'Climber-v1' and r['ended_early'] == '0' and r['commands'] == '600' for r in t) == 26   # none of the 26 reaches the top
    v1 = table('results', 'released_deep_climber_v1.csv')                                            # the four deep Climber-v1 robots
    assert sorted(r['uid'] for r in v1) == sorted(r['uid'] for r in t if depth(r) > 50 and r['task'] == 'Climber-v1')
    assert all(1.0 <= float(r['dataset_reward']) <= 1.25 for r in v1)
    runs = lambda r, k: [float(r[k])] + [float(v) for v in r[k + '_noise'].split()]
    assert all(len(runs(r, 'closed')) == 11 and abs(mean(runs(r, 'closed')) - mean(runs(r, 'released'))) < 0.09 for r in v1)
    assert max(max([float(r['closed_rise_m'])] + [float(v) for v in r['closed_noise_rise_m'].split()]) for r in v1) < 1.33 and '1' not in ''.join(r['closed_noise_ended_early'] for r in v1)
    assert 'those four average within 0.09 of what they average as released and none rises more than 1.33 m' in EVIDENCE


def launcher(uid):
    r = [r for r in table('results', 'released_launchers.csv') if r['uid'].startswith(uid)][0]
    f = lambda k: [float(v) for v in r[k].split()]
    return r, f


def test_best_released_climber():
    c0 = RELEASED['tasks']['Climber-v0']
    assert (c0['robots'], f"{c0['best_reward']:.2f}", f"{c0['second_best_reward']:.2f}", c0['robots_with_policies']) == (3417, '9.27', '5.49', 166)
    r, f = launcher('cca8b49a')
    assert r['uid'] == c0['best_uid']                                     # the robot with the best released Climber-v0 reward
    assert (r['task'], r['generated_by'], r['cells']) == ('Climber-v0', 'CPPN-NEAT', '4')
    assert r['body'] == '00400/00400/00400/00400/00000'                   # a column of four vertical muscles
    assert f"{float(r['dataset_reward']):.2f}" == '1.77' == f"{float(r['released']):.2f}"          # the replay is the authors' second evaluation
    ended = [int(v) for v in r['released_noise_ended_early'].split()]
    assert len(ended) == 200 and sum(ended) == 2
    assert sorted(f'{x:.2f}' for x, e in zip(f('released_noise'), ended) if e) == ['9.21', '9.24']
    assert all(d > 0.09 for d, e in zip(f('released_noise_deepest_unpushed_m'), ended) if e)        # each time a corner 9 to 10 cm inside the floor
    assert f"{float(r['released_rise_m']):.1f}" == '5.8' and f"{1000 * float(r['released_deepest_unpushed_m']):.0f}" == '85'
    closed_rise = max(f('closed_noise_rise_m') + [float(r['closed_rise_m'])])
    assert f'{closed_rise:.1f}' == '1.1' and r['closed_ended_early'] == '0' and '1' not in r['closed_noise_ended_early']
    assert f"{max(f('closed_noise') + [float(r['closed'])]):.2f}" == '1.05'
    for quoted in ('the best scores 9.27 and the next 5.49', 'throws it 5.8 m up the channel', 'in 2 of 200 runs',
                   'for 9.24 and 9.21', 'never rises more than 1.1 m, in 201 runs', 'the best of the 3,417 Climber-v0 robots'):
        assert quoted in README, quoted
    r2, f2 = launcher('51f55c20')                                                                   # a second column of four muscles
    assert (r2['task'], r2['cells'], r2['body']) == ('Climber-v0', '4', '00000/00400/00400/00400/00400')
    assert sum(int(v) for v in r2['released_noise_ended_early'].split()) == 2 and '1' not in r2['closed_noise_ended_early']
    assert max(f2("closed_noise_rise_m") + [float(r2["closed_rise_m"])]) < 1.3 < 6 < float(r2["released_rise_m"])


# ---------------------------------------------------------------- what closing it does to everyone else

def fix_table():
    t = table('results', 'released_fix.csv')
    assert len(t) == 2553
    return [r for r in t if not r['error']]


def test_fix_cost():
    t = fix_table()
    assert len(t) == 2545
    pushed = [r for r in t if int(r['fixed_pushes']) > 0]
    assert len(pushed) == 802
    assert all(r['fixed'] == r['released'] for r in t if int(r['fixed_pushes']) == 0)                # never pushed: the same to the last bit
    assert len(t) - len(pushed) == 1743
    assert sum(r['fixed'] != r['released'] for r in pushed) == 801
    assert f"{1000 * median([float(r['fixed_deepest_m']) for r in pushed]):.1f}" == '0.5'
    assert f"{1000 * max(float(r['fixed_deepest_m']) for r in pushed):.2f}" == '12.62'
    d = [float(r['fixed']) - float(r['released']) for r in t]
    assert sum(abs(x) > 1 for x in d) == 97
    assert (f'{mean(d):.3f}', f'{sd(d) / len(d) ** 0.5:.3f}') == ('-0.002', '0.011')
    jumper = [r for r in t if r['task'] == 'Jumper-v0']
    assert len(jumper) == 78
    assert (f"{mean([float(r['released']) for r in jumper]):.2f}", f"{mean([float(r['fixed']) for r in jumper]):.2f}") == ('2.81', '2.58')
    dj = [float(r['released']) - float(r['fixed']) for r in jumper]
    assert (f'{mean(dj):.2f}', f'{sd(dj) / len(dj) ** 0.5:.2f}') == ('0.23', '0.11')
    for quoted in ('the 2,545 released controllers that run there meet it in 802 episodes, at a median depth of 0.5 mm',
                   '801 scores change, 97 by more than a point, by -0.002 ± 0.011 on average', 'The 1,743 it never touches',
                   'run once each in the shipped wheel\'s arithmetic on Apple silicon, 2.81 and 2.58, a difference of 0.23 ± 0.11'):
        assert quoted in README, quoted
    assert 'they average 2.81 and 2.58, and the difference averages 0.23 with a standard error of 0.11' in EVIDENCE


def test_top_edge_patch():
    t = fix_table()
    deep = [r for r in t if float(r['top_deepest_m']) > 0.02]
    by = {}
    for r in deep:
        by[r['task']] = by.get(r['task'], 0) + 1
    assert len(deep) == 37 and by == {'BeamToppler-v0': 22, 'Climber-v2': 9, 'BeamSlider-v0': 4, 'Lifter-v0': 2}
    worst = max(float(r['top_deepest_m']) for r in t)
    assert f'{2.1e6 * (0.005 + worst):,.0f}'[:3] == '220'                                            # about 220,000 N
    beam = max((r for r in t if r['task'] == 'BeamToppler-v0'), key=lambda r: float(r['released']))
    assert (f"{float(beam['released']):.2f}", f"{float(beam['top']):.2f}", f"{float(beam['fixed']):.2f}") == ('9.41', '11.46', '9.39')
    assert max(float(r['top']) for r in t if r['task'] == 'BeamToppler-v0') == float(beam['top'])
    for quoted in ('with up to 220,000 N', 'more than 2 cm deep to 37 released robots', 'from 9.41 to 11.46'):
        assert quoted in README, quoted
    for quoted in ('22 in BeamToppler-v0, 9 in Climber-v2, 4 in BeamSlider-v0 and 2 in Lifter-v0', 'scores 11.46 under this patch and 9.39 under the fix',
                   'deeper than 12.62 mm'):
        assert quoted in EVIDENCE, quoted

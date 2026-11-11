"""Fetch the robots released by the authors of Evolution Gym, with their policies.

Downloads the two datasets (CC BY-NC 4.0; they are not redistributed with this repository), checks their
SHA-256, fetches the code that defines the policy class, and unpacks one policy file per robot under
build/released/. Writes the counts and the best released score of every task to results/released.json.

    python scope/fetch.py
"""

import os, sys, json, hashlib, subprocess, urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'build', 'released')
HF = 'https://huggingface.co/datasets/EvoGym/{}/resolve/{}/data/train-00000-of-00001.parquet'
FILES = {       # dataset: (revision, SHA-256 of the file)
    'robots': ('7b038e2f47ca0a48fbb102020867c7a89b0387bf', '647c14d5b14fbfa5ee036f97afe87daf368ff11e142d2dbc9f5c09a5d4627c06'),
    'robots-with-policies': ('d70b80fae5a3a7b0decfaff6744ae09da556dd2a', '7842f70226fa18413992b1389b4e6a6f27cd4af5414597e3b8bf75a50a61994f'),
}
POLICY_CODE = ('https://github.com/EvolutionGym/evogym-datasets.git', 'c85123c83d8337b3803257d73a4ab0b2ef9dc2b3')


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def main():
    import pyarrow.parquet as pq
    os.makedirs(OUT, exist_ok=True)
    for name, (revision, want) in FILES.items():
        path = os.path.join(OUT, name + '.parquet')
        if not os.path.exists(path):
            print('downloading', name)
            urllib.request.urlretrieve(HF.format(name, revision), path)
        got = sha256(path)
        if got != want:
            sys.exit(f'{name}: SHA-256 {got} differs from the pinned {want}')
        print(name, got)
    repo = os.path.join(OUT, 'evogym-datasets')
    if not os.path.isdir(os.path.join(repo, '.git')):
        subprocess.run(['git', 'clone', '--quiet', POLICY_CODE[0], repo], check=True)
    subprocess.run(['git', '-C', repo, 'checkout', '--quiet', POLICY_CODE[1]], check=True)

    # every released score for Jumper-v0
    t = pq.read_table(os.path.join(OUT, 'robots.parquet'), columns=['uid', 'reward', 'env_name', 'generated_by']).to_pylist()
    jumper = [r for r in t if r['env_name'] == 'Jumper-v0']
    best = max(jumper, key=lambda r: r['reward'])
    by = {}
    for r in jumper:
        m = by.setdefault(r['generated_by'], dict(robots=0, best_reward=float('-inf')))
        m['robots'] += 1
        m['best_reward'] = max(m['best_reward'], r['reward'])
    tasks = {}
    for r in t:
        tasks.setdefault(r['env_name'], []).append((r['reward'], r['uid']))
    tasks = {name: dict(robots=len(v), best_reward=max(v)[0], best_uid=max(v)[1], second_best_reward=sorted(v)[-2][0])
             for name, v in sorted(tasks.items())}
    summary = dict(robots=len(t), tasks=tasks, jumper_robots=len(jumper),
                   jumper_best_reward=best['reward'], jumper_best_uid=best['uid'], jumper_best_generated_by=best['generated_by'],
                   jumper_by_method=by)
    json.dump(summary, open(os.path.join(ROOT, 'results', 'released.json'), 'w'), indent=1)
    print(len(t), 'released robots;', len(jumper), 'for Jumper-v0, best', best['reward'])

    # one policy file per robot, and an index
    pol = os.path.join(OUT, 'policies')
    os.makedirs(pol, exist_ok=True)
    index = []
    for batch in pq.ParquetFile(os.path.join(OUT, 'robots-with-policies.parquet')).iter_batches(batch_size=64):
        d = batch.to_pydict()
        for i, uid in enumerate(d['uid']):
            with open(os.path.join(pol, uid + '.pt'), 'wb') as f:
                f.write(d['policy_blob'][i])
            index.append(dict(uid=uid, env_name=d['env_name'][i], reward=float(d['reward'][i]), generated_by=d['generated_by'][i],
                              body=[list(map(int, row)) for row in d['body'][i]],
                              connections=[list(map(int, row)) for row in d['connections'][i]]))
    json.dump(index, open(os.path.join(OUT, 'index.json'), 'w'))
    summary.update(robots_with_policies=len(index), jumper_robots_with_policies=sum(1 for r in index if r['env_name'] == 'Jumper-v0'))
    for r in index:
        tasks[r['env_name']]['robots_with_policies'] = tasks[r['env_name']].get('robots_with_policies', 0) + 1
    json.dump(summary, open(os.path.join(ROOT, 'results', 'released.json'), 'w'), indent=1)
    print(len(index), 'robots with policies;', summary['jumper_robots_with_policies'], 'for Jumper-v0')


if __name__ == '__main__':
    main()

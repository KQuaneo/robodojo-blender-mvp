"""Exact-input cube-only diagnostic; immutable legacy executor, no retries."""
import concurrent.futures
import copy
import hashlib
import json
import subprocess
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OLD = ROOT / 'experiments/random20'
OUT = HERE / 'a_results'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def outcome(result, events):
    raw = result['termination_reason']
    if raw == 'invalid_initialization':
        return raw
    failed = [g['frame'] for g in result['grasp_gates'] if not g['passed']]
    if events and (not failed or events[0]['frame'] < min(failed)):
        return 'collision'
    return 'task_failure' if raw == 'task_or_acceptance_failure' else raw

def main():
    OUT.mkdir(exist_ok=False)
    (OUT / 'logs').mkdir()
    source_path = OLD / 'results/manifest_v2.json'
    source = json.loads(source_path.read_text())
    manifest = copy.deepcopy(source)
    for trial in manifest['trials']:
        trial['offsets_m']['broom_driver'] = [0., 0., 0.]
        trial['offsets_m']['broom_shovel'] = [0., 0., 0.]
    manifest.update(experiment='A_cube_only', source_manifest_sha256=sha(source_path),
                    protocol_sha256=sha(HERE / 'PROTOCOL.md'),
                    runner_sha256=sha(Path(__file__)),
                    held_out_seeds_reserved_only=[100, 119])
    for name, expected in source['harness'].items():
        assert sha(OLD / name) == expected, f'Legacy harness changed: {name}'
    for name, expected in source['artifacts'].items():
        assert sha(ROOT / name) == expected, f'Baseline changed: {name}'
    manifest_path = OUT / 'manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2))
    records = []

    def run(trial):
        ident = trial['id']
        dest = OUT / ident
        command = ['blender', '--background', str(ROOT / 'output_smooth/sweep_blocks_smooth.blend'),
                   '--python-exit-code', '1', '--python', str(OLD / 'run_trial.py'), '--',
                   '--manifest', str(manifest_path), '--trial', ident, '--output', str(dest)]
        started = time.time()
        logpath = OUT / 'logs' / (ident + '.log')
        timed_out = False
        with logpath.open('x') as log:
            proc = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            print('START', ident, proc.pid, flush=True)
            size, stalls = 0, 0
            while proc.poll() is None:
                try:
                    proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    current = logpath.stat().st_size
                    stalls = stalls + 1 if current == size else 0
                    size = current
                    print('MONITOR', ident, round(time.time()-started), 'log_bytes', size, flush=True)
                    if stalls in (3, 5):
                        print('ADVISORY OUTPUT_STALL', ident, 'continuing', flush=True)
                    if time.time()-started >= 600:
                        print('HARD_TIMEOUT: terminating', ident, flush=True)
                        timed_out = True
                        proc.terminate()
                        try:
                            proc.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            proc.kill()
                            proc.wait()
        record = dict(id=ident, command=command, pid=proc.pid, exit_code=proc.returncode,
                      duration_s=time.time()-started,
                      status='timeout' if timed_out else ('completed' if proc.returncode == 0 else 'crashed'))
        (OUT / (ident + '_process.json')).write_text(json.dumps(record, indent=2))
        if record['status'] != 'completed':
            raise RuntimeError(f'{ident}: {record["status"]}; no retry')
        result = json.loads((dest / 'result.json').read_text())
        events = json.loads((dest / 'collision_events.json').read_text())
        normalized = outcome(result, events)
        (dest / 'outcome.json').write_text(json.dumps(dict(outcome=normalized,
            raw_termination_reason=result['termination_reason'], collision_events=len(events)), indent=2))
        print('DONE', ident, normalized, flush=True)
        return record, result, normalized

    control = run(manifest['trials'][0])
    assert control[1]['qualified'], 'Control failed; no diagnostic seeds dispatched'
    records.append(control)
    # Submit only two at a time; failure prevents further dispatch.
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        trials = iter(manifest['trials'][1:])
        pending = {pool.submit(run, next(trials)) for _ in range(2)}
        while pending:
            done, pending = concurrent.futures.wait(pending, return_when=concurrent.futures.FIRST_COMPLETED)
            completed = [f.result() for f in done]
            records.extend(completed)
            for _ in done:
                trial = next(trials, None)
                if trial is not None:
                    pending.add(pool.submit(run, trial))
    rows = sorted(records[1:], key=lambda r: r[1]['seed'])
    summary = dict(control_qualified=control[1]['qualified'], attempted=len(rows),
                   qualified=sum(r[1]['qualified'] for r in rows),
                   outcomes={k:sum(r[2] == k for r in rows) for k in sorted({r[2] for r in rows})},
                   baseline_unchanged=all(sha(ROOT/n) == s for n,s in source['artifacts'].items()))
    paired = []
    for _, result, normalized in rows:
        previous = json.loads((OLD / 'results' / result['id'] / 'result.json').read_text())
        paired.append(dict(seed=result['seed'], A=normalized, B='not_run',
                           previous_frozen_script=previous['termination_reason'],
                           source_offsets_m=previous['offsets_m'], A_offsets_m=result['offsets_m']))
    (OUT / 'summary.json').write_text(json.dumps(summary, indent=2))
    (OUT / 'paired.json').write_text(json.dumps(paired, indent=2))
    (OUT / 'SHA256.json').write_text(json.dumps({str(p.relative_to(OUT)):sha(p)
        for p in sorted(OUT.rglob('*')) if p.is_file()}, indent=2))
    print('SUMMARY', json.dumps(summary), flush=True)

if __name__ == '__main__':
    main()

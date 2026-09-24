"""Sequential B planner/executor batch with frozen method and no crash retries."""
import collections
import copy
import json
import subprocess
import time
from pathlib import Path
from run_a import HERE, ROOT, OLD, sha, outcome

OUT = HERE / 'b_results'

def process(command, ident, phase):
    started = time.time()
    logpath = OUT / 'logs' / f'{ident}_{phase}.log'
    timed_out = False
    with logpath.open('x') as log:
        proc = subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        print('START',ident,phase,proc.pid,flush=True)
        last_size,stalls = 0,0
        while proc.poll() is None:
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                size = logpath.stat().st_size
                stalls = stalls+1 if size == last_size else 0
                last_size = size
                print('MONITOR',ident,phase,round(time.time()-started),'log_bytes',size,flush=True)
                if stalls in (3,5):
                    print('ADVISORY OUTPUT_STALL',ident,phase,'continuing',flush=True)
                if time.time()-started >= 600:
                    print('HARD_TIMEOUT: terminating',ident,phase,flush=True)
                    timed_out = True
                    proc.terminate()
                    try:
                        proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
    record = dict(id=ident,phase=phase,command=command,pid=proc.pid,exit_code=proc.returncode,
                  duration_s=time.time()-started,
                  status='timeout' if timed_out else ('completed' if proc.returncode == 0 else 'crashed'))
    (OUT / f'{ident}_{phase}_process.json').write_text(json.dumps(record,indent=2))
    if record['status'] != 'completed':
        raise RuntimeError(f'{ident} {phase}: {record["status"]}; stopped, no automatic retry')
    return record

def main():
    OUT.mkdir(exist_ok=False)
    (OUT / 'logs').mkdir()
    source_path = OLD / 'results/manifest_v2.json'
    source = json.loads(source_path.read_text())
    manifest = copy.deepcopy(source)
    files = ['PROTOCOL.md','B_METHOD.md','plan_b.py','run_b_trial.py','run_b.py','run_a.py']
    manifest.update(experiment='B_local_retarget_v1',source_manifest_sha256=sha(source_path),
                    method_sha256=sha(HERE / 'B_METHOD.md'),protocol_sha256=sha(HERE / 'PROTOCOL.md'),
                    B_harness={n:sha(HERE / n) for n in files},held_out_seeds_reserved_only=[100,119])
    for n,h in source['artifacts'].items():
        assert sha(ROOT / n) == h
    manifest_path = OUT / 'manifest.json'
    manifest_path.write_text(json.dumps(manifest,indent=2))
    records = []
    for trial in manifest['trials']:
        ident = trial['id']
        assert trial['seed'] is None or trial['seed'] in range(20)
        for n,h in manifest['B_harness'].items():
            assert sha(HERE / n) == h, f'B method changed: {n}'
        plan_dir = OUT / ident / 'plan'
        command = ['blender','--background',str(ROOT / 'output_smooth/sweep_blocks_smooth.blend'),
                   '--python-exit-code','1','--python',str(HERE / 'plan_b.py'),'--',
                   '--manifest',str(manifest_path),'--trial',ident,'--output',str(plan_dir)]
        process(command,ident,'planning')
        plan = json.loads((plan_dir / 'plan.json').read_text())
        normalized = plan['outcome']
        if normalized == 'planned':
            dest = OUT / ident / 'execution'
            command = ['blender','--background',str(plan_dir / 'planned.blend'),
                       '--python-exit-code','1','--python',str(HERE / 'run_b_trial.py'),'--',
                       '--manifest',str(manifest_path),'--trial',ident,'--output',str(dest),
                       '--plan-dir',str(plan_dir)]
            process(command,ident,'execution')
            result = json.loads((dest / 'result.json').read_text())
            events = json.loads((dest / 'collision_events.json').read_text())
            normalized = outcome(result,events)
        record = dict(id=ident,seed=trial['seed'],outcome=normalized,executed=plan['outcome']=='planned')
        records.append(record)
        (OUT / ident / 'outcome.json').write_text(json.dumps(record,indent=2))
        print('DONE',json.dumps(record),flush=True)
        if ident == 'control' and normalized != 'success':
            raise RuntimeError('Nominal B control failed; diagnostic seeds NOT dispatched')
    summary = dict(control_qualified=records[0]['outcome']=='success',attempted=20,
                   outcomes=dict(collections.Counter(r['outcome'] for r in records[1:])),
                   executed=sum(r['executed'] for r in records[1:]),
                   baseline_unchanged=all(sha(ROOT/n)==h for n,h in source['artifacts'].items()))
    (OUT / 'summary.json').write_text(json.dumps(summary,indent=2))
    paired = json.loads((HERE / 'a_results/paired.json').read_text())
    for row in paired:
        record = next(r for r in records if r['seed']==row['seed'])
        row['B'] = record['outcome']
        row['B_executed'] = record['executed']
    (OUT / 'paired.json').write_text(json.dumps(paired,indent=2))
    (OUT / 'SHA256.json').write_text(json.dumps({str(p.relative_to(OUT)):sha(p)
        for p in sorted(OUT.rglob('*')) if p.is_file()},indent=2))
    print('SUMMARY',json.dumps(summary),flush=True)

if __name__ == '__main__':
    main()

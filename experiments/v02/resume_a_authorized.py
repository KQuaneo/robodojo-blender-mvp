"""User-authorized retries of A seeds 10/11, then untouched seeds 12–19."""
import collections
import concurrent.futures
import datetime
import json
from pathlib import Path
from prepare import HERE,ROOT,sha
from monitored import run
from run_batch import normalized

def main():
    out=HERE/'results_a';path=out/'manifest.json'
    manifest=json.loads(path.read_text())
    for n,h in manifest['artifacts'].items():assert sha(ROOT/n)==h
    existing=[json.loads((out/t['id']/'outcome.json').read_text()) for t in manifest['trials'][:11]]
    assert existing[0]['outcome']=='success'
    for seed in (10,11):
        prior=json.loads((out/'logs'/f'seed_{seed:02d}_execution.process.json').read_text())
        assert prior['status']=='timeout'
        assert not (out/f'seed_{seed:02d}'/'outcome.json').exists()
        assert not (out/f'seed_{seed:02d}'/'attempt_02').exists()
    for seed in range(12,20):assert not (out/f'seed_{seed:02d}').exists()
    authorization=dict(approved_by_user=True,approval='User 好 to retry 10/11 in new attempt directories and continue remaining trials',
        utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),retry_seeds=[10,11],
        first_attempt_seeds=list(range(12,20)),timeout_seconds=600,manifest_sha256=sha(path),
        dispatcher_sha256=sha(Path(__file__)),prior_timeouts_preserved=True)
    with (out/'resume_authorization.json').open('x') as f:json.dump(authorization,f,indent=2)
    def trial(t):
        ident=t['id'];retry=t['seed'] in (10,11)
        folder=out/ident
        relative='attempt_02/execution' if retry else 'execution'
        dest=folder/relative
        if retry:(folder/'attempt_02').mkdir(exist_ok=False)
        log=out/'logs'/(ident+('_attempt_02' if retry else '')+'_execution.log')
        command=['blender','--background',str(ROOT/manifest['reference_dir']/'reference.blend'),
            '--python-exit-code','1','--python',str(HERE/'run_trial.py'),'--',
            '--manifest',str(path),'--trial',ident,'--output',str(dest)]
        run(command,log,ROOT,timeout=600)
        result=json.loads((dest/'result.json').read_text());events=json.loads((dest/'collision_events.json').read_text())
        record=dict(id=ident,seed=t['seed'],outcome=normalized(result,events),executed=True,
            qualified=result['qualified'],execution_dir=relative,attempt=2 if retry else 1)
        with (folder/'outcome.json').open('x') as f:json.dump(record,f,indent=2)
        print('DONE a',ident,record['outcome'],'attempt',record['attempt'],flush=True)
        return record
    records=list(existing)
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        todo=iter(manifest['trials'][11:]);pending={pool.submit(trial,next(todo)) for _ in range(2)}
        while pending:
            done,pending=concurrent.futures.wait(pending,return_when=concurrent.futures.FIRST_COMPLETED)
            records.extend(f.result() for f in done)
            for _ in done:
                t=next(todo,None)
                if t:pending.add(pool.submit(trial,t))
    assert len(records)==21
    assert all(sha(ROOT/n)==h for n,h in manifest['artifacts'].items())
    records.sort(key=lambda r:-1 if r['seed'] is None else r['seed'])
    summary=dict(group='a',control_qualified=True,diagnostic_attempted=20,
        outcomes=dict(collections.Counter(r['outcome'] for r in records[1:])),records=records,
        frozen_artifacts_unchanged=True,infrastructure_timeouts=2,user_authorized_retries=[10,11])
    with (out/'summary.json').open('x') as f:json.dump(summary,f,indent=2)
    (out/'SHA256.json').write_text(json.dumps({str(p.relative_to(out)):sha(p) for p in sorted(out.rglob('*')) if p.is_file() and p.name!='SHA256.json'},indent=2))
    print('SUMMARY',json.dumps(summary),flush=True)

if __name__=='__main__':main()

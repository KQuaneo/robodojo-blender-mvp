"""v0.2 batches: fresh controls, no retry, reject invalid plans explicitly."""
import collections
import concurrent.futures
import json
import sys
from pathlib import Path
from prepare import prepare,HERE,ROOT,sha
from monitored import run

def normalized(result,events):
    raw=result['termination_reason']
    if raw=='invalid_initialization':return raw
    failed=[g['frame'] for g in result['grasp_gates'] if not g['passed']]
    if events and (not failed or events[0]['frame']<min(failed)):return 'collision'
    return 'task_failure' if raw=='task_or_acceptance_failure' else raw

def main(group):
    if group!='control':
        assert json.loads((HERE/'results_control/control/execution/result.json').read_text())['qualified']
    path,manifest=prepare(group);out=path.parent;records=[]
    baseline=ROOT/manifest['reference_dir']
    def trial(t):
        ident=t['id'];plan_dir=None
        if group=='b':
            plan_dir=out/ident/'plan'
            command=['blender','--background',str(baseline/'reference.blend'),'--python-exit-code','1',
                     '--python',str(HERE/'plan_b.py'),'--','--manifest',str(path),'--trial',ident,'--output',str(plan_dir)]
            run(command,out/'logs'/f'{ident}_planning.log',ROOT)
            plan=json.loads((plan_dir/'plan.json').read_text())
            if plan['outcome']!='planned':
                result=dict(id=ident,seed=t['seed'],outcome=plan['outcome'],executed=False)
                (out/ident/'outcome.json').write_text(json.dumps(result,indent=2))
                print('DONE',group,ident,result['outcome'],flush=True)
                return result
        dest=out/ident/'execution'
        scene=plan_dir/'planned.blend' if plan_dir else baseline/'reference.blend'
        command=['blender','--background',str(scene),'--python-exit-code','1','--python',str(HERE/'run_trial.py'),
                 '--','--manifest',str(path),'--trial',ident,'--output',str(dest)]
        if plan_dir:command+=['--plan-dir',str(plan_dir)]
        run(command,out/'logs'/f'{ident}_execution.log',ROOT)
        result=json.loads((dest/'result.json').read_text());events=json.loads((dest/'collision_events.json').read_text())
        record=dict(id=ident,seed=t['seed'],outcome=normalized(result,events),executed=True,qualified=result['qualified'])
        (out/ident/'outcome.json').write_text(json.dumps(record,indent=2))
        print('DONE',group,ident,record['outcome'],flush=True)
        return record
    control=trial(manifest['trials'][0]);records.append(control)
    assert control['outcome']=='success', 'Nominal control rejected; diagnostic seeds not dispatched'
    if group=='a':
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            todo=iter(manifest['trials'][1:]);pending={pool.submit(trial,next(todo)) for _ in range(2)}
            while pending:
                done,pending=concurrent.futures.wait(pending,return_when=concurrent.futures.FIRST_COMPLETED)
                records.extend(f.result() for f in done)
                for _ in done:
                    t=next(todo,None)
                    if t:pending.add(pool.submit(trial,t))
    else:
        for t in manifest['trials'][1:]:records.append(trial(t))
    assert all(sha(ROOT/n)==h for n,h in manifest['artifacts'].items())
    summary=dict(group=group,control_qualified=True,diagnostic_attempted=len(records)-1,
                 outcomes=dict(collections.Counter(r['outcome'] for r in records[1:])),
                 frozen_artifacts_unchanged=True,records=records)
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    (out/'SHA256.json').write_text(json.dumps({str(p.relative_to(out)):sha(p) for p in sorted(out.rglob('*')) if p.is_file()},indent=2))
    print('SUMMARY',json.dumps(summary),flush=True)

if __name__=='__main__':main(sys.argv[1])

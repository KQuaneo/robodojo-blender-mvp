"""Same-reference all-object Frozen Script comparator; no policy adaptation."""
import collections
import copy
import json
from pathlib import Path
from prepare import HERE,ROOT,sha
from monitored import run
from run_batch import normalized

def main():
    assert json.loads((HERE/'results_a/summary.json').read_text())['control_qualified']
    out=HERE/'results_frozen';out.mkdir(exist_ok=False)
    source_path=ROOT/'experiments/random20/results/manifest_v2.json'
    source=json.loads(source_path.read_text())
    manifest=copy.deepcopy(json.loads((HERE/'results_a/manifest.json').read_text()))
    manifest.update(experiment='v02_all_object_frozen',trials=source['trials'])
    manifest['artifacts'][str(Path(__file__).resolve().relative_to(ROOT))]=sha(Path(__file__))
    path=out/'manifest.json';path.write_text(json.dumps(manifest,indent=2))
    records=[]
    for t in manifest['trials']:
        ident=t['id'];dest=out/ident/'execution'
        command=['blender','--background',str(ROOT/manifest['reference_dir']/'reference.blend'),
                 '--python-exit-code','1','--python',str(HERE/'run_trial.py'),'--',
                 '--manifest',str(path),'--trial',ident,'--output',str(dest)]
        run(command,out/'logs'/f'{ident}_execution.log',ROOT)
        result=json.loads((dest/'result.json').read_text());events=json.loads((dest/'collision_events.json').read_text())
        record=dict(id=ident,seed=t['seed'],outcome=normalized(result,events),qualified=result['qualified'],executed=True)
        (out/ident/'outcome.json').write_text(json.dumps(record,indent=2))
        records.append(record);print('DONE frozen',ident,record['outcome'],flush=True)
        if ident=='control':assert result['qualified'], 'Frozen comparator control rejected'
    assert all(sha(ROOT/n)==h for n,h in manifest['artifacts'].items())
    summary=dict(group='frozen',control_qualified=True,diagnostic_attempted=20,
                 outcomes=dict(collections.Counter(r['outcome'] for r in records[1:])),records=records,
                 frozen_artifacts_unchanged=True)
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    (out/'SHA256.json').write_text(json.dumps({str(p.relative_to(out)):sha(p) for p in sorted(out.rglob('*')) if p.is_file()},indent=2))
    print('SUMMARY',json.dumps(summary),flush=True)

if __name__=='__main__':main()

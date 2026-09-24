"""Freeze exact source offsets and reference/harness hashes before dispatch."""
import copy
import hashlib
import json
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def prepare(group):
    assert group in ('control','a','b')
    source_path=ROOT/'experiments/random20/results/manifest_v2.json'
    source=json.loads(source_path.read_text());manifest=copy.deepcopy(source)
    baseline=HERE/'baseline_candidate_02'
    assert json.loads((baseline/'repair_report.json').read_text())['continuity']['passed']
    if group=='control':manifest['trials']=manifest['trials'][:1]
    if group=='a':
        for t in manifest['trials']:
            for name in ('broom_driver','broom_shovel'):t['offsets_m'][name]=[0.,0.,0.]
    for name,expected in source['artifacts'].items():assert sha(ROOT/name)==expected
    files=[p for p in baseline.iterdir() if p.is_file()]
    scripts=['run_trial.py','motion.py','monitored.py','prepare.py','PROTOCOL.md','run_batch.py']
    if group=='b':scripts+=['plan_b.py','B_METHOD.md']
    files += [HERE/name for name in scripts]
    manifest['artifacts'].update({str(p.relative_to(ROOT)):sha(p) for p in files})
    manifest.update(experiment='v02_'+group,reference_dir=str(baseline.relative_to(ROOT)),
                    source_manifest_sha256=sha(source_path),held_out_reserved_only=[100,119],
                    audit='all rotation keys + quarter frames + interval midpoints; analytic joint rate bounds')
    out=HERE/('results_'+group);out.mkdir(exist_ok=False)
    path=out/'manifest.json';path.write_text(json.dumps(manifest,indent=2))
    return path,manifest

if __name__=='__main__':print(prepare(sys.argv[1])[0])

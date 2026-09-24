"""Independent same-reference paired audit. Never runs or tunes experiments."""
import collections
import json
import sys
from pathlib import Path
from prepare import HERE,ROOT,sha

source_path=ROOT/'experiments/random20/results/manifest_v2.json'
source=json.loads(source_path.read_text())
taxonomy={'invalid_initialization','pickup_planning_failure','missed_pickup',
          'handoff_planning_failure','missed_handoff','pan_planning_failure',
          'missed_pan_hold','transition_to_sweep_failure','collision','task_failure','success'}
partial='--partial' in sys.argv
groups={}
for group in ('a','frozen','b'):
    out=HERE/('results_'+group)
    if not (out/'manifest.json').exists():
        if partial:
            groups[group]=dict(completed=0,status='not_started')
            continue
        raise AssertionError(f'{group} not started')
    manifest=json.loads((out/'manifest.json').read_text())
    assert manifest['source_manifest_sha256']==sha(source_path)
    assert manifest['reference_dir']=='experiments/v02/baseline_candidate_02'
    assert [t['seed'] for t in manifest['trials']]==[None]+list(range(20))
    for name,expected in manifest['artifacts'].items():assert sha(ROOT/name)==expected
    for original,actual in zip(source['trials'],manifest['trials']):
        assert original['id']==actual['id']
        for name in original['offsets_m']:
            expected=[0.,0.,0.] if group=='a' and name in ('broom_driver','broom_shovel') else original['offsets_m'][name]
            assert actual['offsets_m'][name]==expected
    rows=[]
    for trial in manifest['trials']:
        folder=out/trial['id'];path=folder/'outcome.json'
        if not path.exists():continue
        row=json.loads(path.read_text());assert row['outcome'] in taxonomy
        if row['executed']:
            execution=folder/row.get('execution_dir','execution')
            assert execution.resolve().is_relative_to(folder.resolve())
            if row.get('attempt')==2:
                assert group=='a' and trial['seed'] in (10,11)
                auth=json.loads((out/'resume_authorization.json').read_text())
                assert auth['approved_by_user'] and auth['manifest_sha256']==sha(out/'manifest.json')
                assert json.loads((out/'logs'/(trial['id']+'_execution.process.json')).read_text())['status']=='timeout'
                assert json.loads((out/'logs'/(trial['id']+'_attempt_02_execution.process.json')).read_text())['status']=='completed'
            result=json.loads((execution/'result.json').read_text())
            trace=json.loads((execution/'trace.json').read_text())
            events=json.loads((execution/'collision_events.json').read_text())
            assert result['manifest_sha256']==sha(out/'manifest.json')
            assert result['offsets_m']==trial['offsets_m']
            assert len(trace)==result['executed_through_frame']
            assert len(events)==result['collision_events']
            assert len(result['audit_grid_frames'])==result['collision_samples']
            assert result['continuity']['passed']
            assert result['dynamic_cubes_without_animation'] and result['urdf_position_limits_passed']
            if group=='b':
                plan=json.loads((folder/'plan/plan.json').read_text())
                assert plan['outcome']=='planned' and plan['analytic_continuity']['passed']
                assert plan['frozen_sweep_pose_error']<1e-10
                assert plan['right_and_driver_reference_actions_unchanged']
                assert result['plan_sha256']==sha(folder/'plan/plan.json')
                for name,h in plan['files'].items():assert sha(folder/'plan'/name)==h
        else:
            plan=json.loads((folder/'plan/plan.json').read_text())
            assert plan['outcome']==row['outcome'] and plan['no_fallback']
            assert not (folder/'execution').exists()
        if trial['seed'] is None:assert row['outcome']=='success'
        rows.append(row)
    seeds=[r for r in rows if r['seed'] is not None]
    groups[group]=dict(completed=len(seeds),outcomes=dict(collections.Counter(r['outcome'] for r in seeds)),
                       records=rows,status='complete' if len(rows)==21 else 'partial')
print(json.dumps({g:{k:v for k,v in data.items() if k!='records'} for g,data in groups.items()},indent=2))
if partial:raise SystemExit(0)
assert all(g['status']=='complete' for g in groups.values())
paired=[]
for original in source['trials'][1:]:
    seed=original['seed'];row=dict(seed=seed,source_offsets_m=original['offsets_m'])
    for group,data in groups.items():row[group]=next(r['outcome'] for r in data['records'] if r['seed']==seed)
    paired.append(row)
(HERE/'paired_results.json').write_text(json.dumps(paired,indent=2))
(HERE/'result_audit.json').write_text(json.dumps(dict(groups=groups,exact_input_reuse=True,
    same_v02_reference=True,all_frozen_hashes_unchanged=True,held_out_not_dispatched=True),indent=2))
table='\n'.join(f"| {r['seed']} | {r['frozen']} | {r['a']} | {r['b']} |" for r in paired)
report=f'''# v0.2 paired diagnostic results

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: run
- Origin Date: 2026-09-24
- Verification Status: ANALYZED
- Version Label: v02_paired_diagnostics

## Descriptive outcomes (20 seen seeds each)

- Frozen Script, all objects perturbed: `{json.dumps(groups['frozen']['outcomes'])}`
- A, cubes perturbed and tools nominal: `{json.dumps(groups['a']['outcomes'])}`
- B, all objects perturbed with local acquisition planning: `{json.dumps(groups['b']['outcomes'])}`

All three groups passed a fresh nominal control. Independent audit confirmed
exact source offsets, identical v0.2 reference hashes, unchanged frozen scripts,
consistent trace/event/sample counts, and no execution after planning rejection.
Accepted B plans retained the right-arm/driver reference actions and world-fixed
sweep poses. No held-out seed layout was generated or evaluated.

A seeds 10 and 11 originally timed out during audit. Their original logs and
process records remain preserved; the user explicitly authorized one new attempt
per seed with the same 600-second limit. The selected scientific results come
from `attempt_02/execution`, with provenance in each outcome record and
`results_a/resume_authorization.json`. Timeouts are infrastructure events, not
additional task failures or extra seeds in the denominator.

| Seed | v0.2 Frozen Script | A | B |
|---|---|---|---|
{table}

## Interpretation boundaries

B versus the SAME-v0.2 Frozen Script is the paired acquisition-adaptation
comparison. A uses nominal tools, so B versus A alone is not an isolated causal
contrast. Planning failures are first-class outcomes, not missing trials; the
planner is local/bounded and does not establish global infeasibility.

These seeds were already seen during development. No confidence interval,
significance test, held-out performance or generalization claim is made.
The corrected reference and stronger audit differ from v0.1; historical data
remain preserved, not silently overwritten or pooled with v0.2.

Kinematic attachment is not force closure. Joint-rate bounds concern kinematic
interpolation, not physical actuator dynamics. Mesh collision checks remain
sampled, now including every rotation key and interval midpoints. The original
Blender predicate adapter is not the official Isaac runtime.

Evidence: per-group manifests, process logs/records, all raw plan/execution
files, `paired_results.json`, and `result_audit.json`.
'''
(HERE/'REPORT.md').write_text(report)

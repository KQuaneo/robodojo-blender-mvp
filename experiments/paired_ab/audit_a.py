"""Read-only input/result verification and evidence-backed A report generation."""
import collections
import json
from pathlib import Path
from run_a import HERE, ROOT, OLD, OUT, sha, outcome

def main():
    manifest = json.loads((OUT / 'manifest.json').read_text())
    source_path = OLD / 'results/manifest_v2.json'
    source = json.loads(source_path.read_text())
    assert sha(source_path) == manifest['source_manifest_sha256']
    assert sha(HERE / 'PROTOCOL.md') == manifest['protocol_sha256']
    assert sha(HERE / 'run_a.py') == manifest['runner_sha256']
    assert len(manifest['trials']) == 21
    assert [t['seed'] for t in manifest['trials']] == [None] + list(range(20))
    for original, actual in zip(source['trials'], manifest['trials']):
        assert original['id'] == actual['id']
        for name in ('cube_0', 'cube_1', 'cube_2'):
            assert original['offsets_m'][name] == actual['offsets_m'][name]
        for name in ('broom_driver', 'broom_shovel'):
            assert actual['offsets_m'][name] == [0., 0., 0.]
    for name, expected in source['artifacts'].items():
        assert sha(ROOT / name) == expected
    for name, expected in source['harness'].items():
        assert sha(OLD / name) == expected
    # Allow partial inspection without re-running or overwriting any evidence.
    rows = []
    for trial in manifest['trials']:
        folder = OUT / trial['id']
        if not (folder / 'result.json').exists():
            continue
        result = json.loads((folder / 'result.json').read_text())
        trace = json.loads((folder / 'trace.json').read_text())
        events = json.loads((folder / 'collision_events.json').read_text())
        assert result['manifest_sha256'] == sha(OUT / 'manifest.json')
        assert result['offsets_m'] == trial['offsets_m']
        assert len(trace) == result['executed_through_frame']
        assert len(events) == result['collision_events']
        last = result['executed_through_frame']
        assert result['collision_samples'] == (4*(last-1)+1 if last else 0)
        assert result['dynamic_cubes_without_animation']
        assert result['urdf_position_limits_passed']
        rows.append((trial, result, outcome(result, events)))
    seeds = [r for r in rows if r[0]['seed'] is not None]
    counts = dict(collections.Counter(r[2] for r in seeds))
    complete = len(rows) == 21
    audit = dict(status='complete' if complete else 'partial', completed_seeds=len(seeds),
                 outcome_counts=counts, exact_source_cube_offsets=True, nominal_tools=True,
                 baseline_and_executor_unchanged=True, held_out_not_in_manifest=True,
                 raw_trace_and_collision_counts_consistent=True)
    print(json.dumps(audit, indent=2))
    if not complete:
        return
    assert rows[0][1]['qualified']
    process_records = [json.loads((OUT / (t['id']+'_process.json')).read_text())
                       for t in manifest['trials']]
    assert all(p['status'] == 'completed' and p['exit_code'] == 0 for p in process_records)
    old_control = json.loads((OLD / 'results/control/trace.json').read_text())
    new_control = json.loads((OUT / 'control/trace.json').read_text())
    control_exact = new_control[-1]['positions'] == old_control[-1]['positions']
    audit.update(all_processes_completed=True, control_final_positions_exact_match=control_exact)
    (OUT / 'result_audit.json').write_text(json.dumps(audit, indent=2))
    table = '\n'.join('| '+str(t['seed'])+' | '+category+' | '+str(r['collision_events'])+' | '+
                      str(r['first_success_frame'])+' |' for t,r,category in seeds)
    gates_passed = sum(all(g['passed'] for g in r['grasp_gates']) and len(r['grasp_gates']) == 3
                       for _,r,_ in seeds)
    false_conditions = dict(collections.Counter(name for _,r,_ in seeds
        for name,passed in (r['conditions_at_termination'] or {}).items() if not passed))
    report = f'''# A: cube-only paired diagnostic

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: run
- Origin Date: 2026-09-23
- Verification Status: ANALYZED
- Version Label: paired_A_v1

## Execution

Command: `python3 experiments/paired_ab/run_a.py` from repository root.
Status: completed; 1 qualified nominal control and {len(seeds)} diagnostic seeds.
Control: first predicate success at frame {rows[0][1]['first_success_frame']},
{rows[0][1]['collision_samples']} collision samples,
{rows[0][1]['collision_events']} collision events.
Exact control final-position match with the previous fresh control: {control_exact}.
Per-process durations ranged from {min(p['duration_s'] for p in process_records):.1f}
to {max(p['duration_s'] for p in process_records):.1f} seconds. All 21 processes
exited normally; no hard timeout or automatic retry occurred.

Outcomes: `{json.dumps(counts, ensure_ascii=False)}`.
All three acquisition gates passed in {gates_passed}/{len(seeds)} diagnostic runs.
Counts of failed final predicate components: `{json.dumps(false_conditions)}`.
Counts describe this seen seed set only; no significance or generalization claim.

| Seed | Primary outcome | Collision samples with hits | First predicate success frame |
|---|---|---:|---:|
{table}

`first_success_frame` is the first predicate-only success, not independently
qualified full-episode success. Collision counts are sampling events, not
distinct impacts. Raw results retain all co-occurring gate/collision evidence.

## Reproducibility and boundaries

Independent audit confirmed exact reuse of prior cube offsets, nominal tools,
unchanged baseline/executor/harness hashes, trace lengths and collision counts.
See `a_results/result_audit.json`, per-seed raw files and process records.
`a_results/paired.json` pairs A with prior Frozen Script; B is explicitly not run.
No retries were performed. No held-out layout was generated or evaluated.

B's experiment definition is frozen in `PROTOCOL.md`; at A completion its
planner had not been evaluated. Subsequent B work is recorded separately.
The B comparison must retain the world-fixed sweep
and explicitly report planning failures. A uses nominal tools, so B versus A
alone cannot isolate acquisition adaptation; compare B with the previous
all-random Frozen Script using the same full offsets.

Limitations: gated kinematic attachment rather than force closure; finite-time
sampled robot geometry audits, not continuous collision proof; Blender adapter,
not the official Isaac runtime. This is a diagnostic, not an unseen benchmark.
No results have been uploaded to GitHub in this experiment.
'''
    (HERE / 'A_REPORT.md').write_text(report)
    (OUT / 'SHA256.json').write_text(json.dumps({str(p.relative_to(OUT)):sha(p)
        for p in sorted(OUT.rglob('*')) if p.is_file() and p.name != 'SHA256.json'}, indent=2))

if __name__ == '__main__':
    main()

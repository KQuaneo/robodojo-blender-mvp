"""Audit the stopped B control without running or changing the experiment."""
import json
from pathlib import Path
from run_a import HERE,ROOT,sha

out = HERE/'b_results'
manifest = json.loads((out/'manifest.json').read_text())
for name,expected in manifest['artifacts'].items():
    assert sha(ROOT/name)==expected
for name,expected in manifest['B_harness'].items():
    assert sha(HERE/name)==expected
assert not list(out.glob('seed_*'))
assert not (out/'control/execution').exists()
assert not list(out.glob('*execution_process.json'))
process = json.loads((out/'control_planning_process.json').read_text())
plan = json.loads((out/'control/plan/plan.json').read_text())
assert process['status']=='completed' and process['exit_code']==0
assert plan['outcome']=='handoff_planning_failure'
pose = json.loads((out/'reference_pose_diagnostic.json').read_text())
assert not pose['physics_enabled']
assert max(r['flange_fk_max_matrix_error'] for r in pose['poses'])<1e-6
audit = dict(status='stopped_after_control_rejection',diagnostic_seeds_attempted=0,
             control_planner_process_completed_normally=True,control_executed=False,
             baseline_and_frozen_B_method_unchanged=True,no_retry=True,
             no_held_out_trials=True,pose_diagnostic_fk_crosscheck_passed=True,
             control_diagnostic_outcome='transition_to_sweep_failure',
             raw_classifier_defect_preserved=True)
(out/'stop_audit.json').write_text(json.dumps(audit,indent=2))
(out/'SHA256.json').write_text(json.dumps({str(p.relative_to(out)):sha(p)
    for p in sorted(out.rglob('*')) if p.is_file() and p.name!='SHA256.json'},indent=2))
print(json.dumps(audit,indent=2))

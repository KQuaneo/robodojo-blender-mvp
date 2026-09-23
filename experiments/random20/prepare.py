"""Freeze input draws and hashes before any trial. Refuse overwriting manifests."""
import hashlib,json,random,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
commit=subprocess.check_output(['git','rev-parse','v0.1-scripted-baseline^{}'],cwd=ROOT,text=True).strip()
assert commit=='58d0d5cf9010fc9291e30d9a9522a401362c7a5f'
paths=['output_smooth/sweep_blocks_smooth.blend','output_smooth/joint_trace.json','output_smooth/grasp_calibration.json','output_smooth/transfer_plan.json','output_smooth/collision_report.json','output_smooth/trace.json','collision_geometry.py','official_checks.py','build_official_scene.py','plan_collision_scene.py','smooth_transfers.py']
artifacts={name:sha(ROOT/name) for name in paths}
for name in paths:
    frozen=subprocess.check_output(['git','show',f'{commit}:{name}'],cwd=ROOT)
    assert hashlib.sha256(frozen).hexdigest()==artifacts[name],f'Frozen artifact modified: {name}'
names=['cube_0','cube_1','cube_2','broom_driver','broom_shovel']
trials=[{'id':'control','seed':None,'offsets_m':{n:[0.,0.,0.] for n in names}}]
for seed in range(20):
    rng=random.Random(seed)
    trials.append({'id':f'seed_{seed:02d}','seed':seed,'offsets_m':{n:[rng.uniform(-.02,.02),rng.uniform(-.02,.02),0.] for n in names}})
protocol={'approved_by_user':True,'approval':'User ok to frozen XY ±2cm / 20 seeds / gated pickup protocol','baseline_commit':commit,'seed_generator':'Python random.Random(seed), MT19937','python_version':sys.version,'draw_order':names,'translation_range_m':[-.02,.02],'orientation_randomized':False,'alignment_m':.002,'alignment_deg':1,'per_trial_timeout_s':600,'step_budget':1000,'audit_dt_s':.01,'trials':trials,'artifacts':artifacts,'harness':{p.name:sha(p) for p in (HERE/'run_trial.py',HERE/'run_batch.py',HERE/'prepare.py')},'protocol_sha256':sha(HERE/'PROTOCOL_DRAFT.md')}
out=HERE/'results';out.mkdir(exist_ok=True)
target=out/(sys.argv[1] if len(sys.argv)>1 else 'manifest.json')
with target.open('x') as f:json.dump(protocol,f,indent=2)
print('MANIFEST',target,sha(target))

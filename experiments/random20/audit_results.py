"""Independent bookkeeping checks; no simulation, fitting, or policy edits."""
import hashlib,json,math,random
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1];OUT=HERE/'results'
manifest_path=OUT/'manifest_v2.json';manifest=json.loads(manifest_path.read_text())
sha=hashlib.sha256(manifest_path.read_bytes()).hexdigest()
control=json.loads((OUT/'control/result.json').read_text());all_results=[]
assert [r['seed'] for r in manifest['trials']]==[None]+list(range(20))
assert len(manifest['trials'])==21
for trial in manifest['trials']:
    result=json.loads((OUT/trial['id']/'result.json').read_text());all_results.append(result)
    assert result['manifest_sha256']==sha
    if trial['seed'] is not None:
        rng=random.Random(trial['seed'])
        expected={n:[rng.uniform(-.02,.02),rng.uniform(-.02,.02),0.] for n in manifest['draw_order']}
        assert expected==trial['offsets_m']==result['offsets_m']
    for n,offset in trial['offsets_m'].items():
        mesh='broom' if n=='broom_driver' else n
        old=control['initial_poses'][mesh];new=result['initial_poses'][mesh]
        for i in range(3):
            assert abs((new[i][3]-old[i][3])-offset[i])<1e-6,(trial['id'],n,'translation')
            for j in range(3):assert abs(new[i][j]-old[i][j])<1e-6,(trial['id'],n,'rotation')
    last=result['executed_through_frame']
    assert len(json.loads((OUT/trial['id']/'trace.json').read_text()))==last
    assert result['collision_samples']==(4*(last-1)+1 if last else 0)
    events=json.loads((OUT/trial['id']/'collision_events.json').read_text())
    assert len(events)==result['collision_events']
    assert all(e['frame']<=last for e in events)
    failed=[g for g in result['grasp_gates'] if not g['passed']]
    if failed:assert last==failed[0]['frame']-1 and not result['qualified']
    assert result['dynamic_cubes_without_animation']
    assert result['full_episode_audited']==(last==1000)
assert control['qualified'] and control['first_success_frame']==986 and control['collision_events']==0
reference=json.loads((ROOT/'output_smooth/trace.json').read_text())[-1]['positions']
repeat=json.loads((OUT/'control/trace.json').read_text())[-1]['positions']
assert reference==repeat,'Fresh control final positions differ from frozen run'
for n,s in manifest['artifacts'].items():assert hashlib.sha256((ROOT/n).read_bytes()).hexdigest()==s
processes=json.loads((OUT/'processes.json').read_text())
assert len(processes)==21 and all(p['status']=='completed' and p['exit_code']==0 for p in processes)
audit={'passed':True,'trial_count':21,'random_seeds_exactly_0_to_19':True,'sampled_offsets_reproduced':True,'actual_initial_poses_match_offsets':True,'rotations_and_z_unchanged':True,'no_unapproved_grasp_binding':True,'sample_counts_match_executed_prefixes':True,'control_final_positions_exact_match':True,'baseline_hashes_unchanged':True,'processes_completed_without_crash':True,'scope':'Bookkeeping/pose/hash checks; does not prove force-based grasp validity'}
(OUT/'result_audit.json').write_text(json.dumps(audit,indent=2));print(json.dumps(audit,indent=2))

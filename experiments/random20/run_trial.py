"""One frozen-policy trial. No replanning, policy edits, or cube animation."""
import argparse, hashlib, itertools, json, math, sys, time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
import bpy
import numpy as np
from mathutils import Vector
import build_official_scene as b
from collision_geometry import CollisionWorld,intersects
from official_checks import StateAdapter,source_fingerprint

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def angle(a,c):
    # Remove float32 scale/orthogonality noise before angular measurements.
    u,_,vt=np.linalg.svd(a[:3,:3]);ra=u@vt
    u,_,vt=np.linalg.svd(c[:3,:3]);rc=u@vt
    return math.degrees(math.acos(float(np.clip((np.trace(ra@rc.T)-1)/2,-1,1))))
def residual(actual,expected):
    distance=float(np.linalg.norm(actual[:3,3]-expected[:3,3]));rotation=angle(actual,expected)
    return {'translation_m':distance,'rotation_deg':rotation,'passed':distance<=.002 and rotation<=1}
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--manifest',required=True);ap.add_argument('--trial',required=True);ap.add_argument('--output',required=True);ap.add_argument('--preflight',action='store_true')
    args=ap.parse_args(sys.argv[sys.argv.index('--')+1:]);out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    manifest=json.loads(Path(args.manifest).read_text());layout=next(x for x in manifest['trials'] if x['id']==args.trial)
    for name,sha in manifest['artifacts'].items():assert digest(ROOT/name)==sha, f'Artifact changed: {name}'
    started=time.time();scene=bpy.context.scene;scene.frame_set(1)
    def mat(name):return np.array(bpy.data.objects[name].evaluated_get(bpy.context.evaluated_depsgraph_get()).matrix_world)
    base=ROOT/'output_smooth';qtrace=json.loads((base/'joint_trace.json').read_text());cal=json.loads((base/'grasp_calibration.json').read_text())
    ee=[np.array(m) for m in cal['tool_to_flange']]
    roots=[mat(s+'_root') for s in ('Left','Right')]
    def flange(arm,f):return b.fk(np.array(qtrace[f-1]['q'][arm]),roots[arm])
    pan_original=mat('broom_shovel');pan_to_flange=np.linalg.inv(pan_original)@flange(0,270)
    # Initialization is the ONLY write to cube transforms. Driver delta applies
    # only while unheld; frozen COPY_TRANSFORMS takes over after a passed gate.
    for name,delta in layout['offsets_m'].items():
        obj=bpy.data.objects[name]
        if name=='broom_driver':obj.delta_location=Vector(delta)
        else:obj.location+=Vector(delta)
    bpy.context.view_layer.update();scene.frame_set(1)
    names=[f'{s}_{n}' for s in ('Left','Right') for n in ['base_link']+[f'link{i}' for i in range(1,9)]]
    movable=['broom','broom_shovel','cube_0','cube_1','cube_2'];names+=movable+['Table']
    world=CollisionWorld([bpy.data.objects[n] for n in names]);initial={n:mat(n) for n in names}
    initial_driver=mat('broom_driver')
    init_errors=[];init_hits=world.check(initial)
    if init_hits:init_errors.append({'robot_intersections':init_hits})
    for a,c in itertools.combinations(movable,2):
        if intersects(world.geometry[a],initial[a],world.geometry[c],initial[c]):init_errors.append({'object_intersection':[a,c]})
    table_lo,table_hi=world.geometry['Table'].bounds(initial['Table']);bounds={}
    for n in movable:
        points=world.geometry[n].vertices@initial[n][:3,:3].T+initial[n][:3,3]
        lo,hi=points.min(0),points.max(0);bounds[n]={'min':lo.tolist(),'max':hi.tolist()}
        if np.any(lo[:2]<table_lo[:2]) or np.any(hi[:2]>table_hi[:2]):init_errors.append({'outside_table':n})
        if lo[2]<table_hi[2]-.002:init_errors.append({'below_table':n,'penetration_m':float(table_hi[2]-lo[2])})
    tcp=np.eye(4);tcp[0,3]=.145
    checker=StateAdapter(initial['broom_shovel'],[mat(s+'_joint6')@tcp for s in ('Left','Right')])
    gates={65:('pickup',initial_driver,flange(0,65)@np.linalg.inv(ee[0])),
           190:('handoff',flange(0,190)@np.linalg.inv(ee[0]),flange(1,190)@np.linalg.inv(ee[1])),
           270:('pan_hold',initial['broom_shovel'],flange(0,270)@np.linalg.inv(pan_to_flange))}
    if args.preflight:
        preflight={'initialization_errors':init_errors,'initial_bounds':bounds,'gates':[{ 'name':name,'frame':f,**residual(actual,expected)} for f,(name,actual,expected) in gates.items()]}
        (out/'preflight.json').write_text(json.dumps(preflight,indent=2));print('PREFLIGHT',json.dumps(preflight),flush=True)
        assert not init_errors and all(g['passed'] for g in preflight['gates']), 'Control preflight failed'
        assert residual(np.eye(4),np.eye(4))['passed']
        shifted=np.eye(4);shifted[0,3]=.01
        assert not residual(shifted,np.eye(4))['passed']
        rotated=np.eye(4);r=math.radians(2);rotated[:2,:2]=[[math.cos(r),-math.sin(r)],[math.sin(r),math.cos(r)]]
        assert not residual(rotated,np.eye(4))['passed']
        print('PASS alignment gate positive/translation-negative/rotation-negative controls',flush=True)
        return
    gate_results=[];trace=[];reason='invalid_initialization' if init_errors else None;last=0;first_success=None
    if not init_errors:
        for f in range(1,1001):
            if f in gates:
                name,actual,expected=gates[f];gate={'name':name,'frame':f,**residual(actual,expected)};gate_results.append(gate)
                print('GATE',json.dumps(gate),flush=True)
                if not gate['passed']:reason='missed_'+name;break
            scene.frame_set(f);last=f
            poses={'pan':mat('broom_shovel'),'broom':mat('broom_driver'),**{f'cube{i}':mat(f'cube_{i}') for i in range(3)}}
            conditions=checker.check(poses,[mat(s+'_joint6')@tcp for s in ('Left','Right')])
            if all(conditions.values()) and first_success is None:first_success=f
            trace.append({'frame':f,'positions':[poses[f'cube{i}'][:3,3].tolist() for i in range(3)],'conditions':conditions})
            if f%100==0:print('PHYSICS',f,flush=True)
    # Independent geometry audit of the actual executed prefix, after caching.
    events=[];samples=0;max_attachment=0.;max_tilt=0.;max_speed=0.;previous=None
    ranges=json.loads((base/'transfer_plan.json').read_text())
    for f in range(1,last+1):
        for fraction in ([0,.25,.5,.75] if f<last else [0]):
            scene.frame_set(f,subframe=fraction);mats={n:mat(n) for n in names};hits=world.check(mats);samples+=1
            if hits:events.append({'frame':f+fraction,'pairs':hits})
            moment=f+fraction;owner=0 if 65<=moment<190 else (1 if 190<=moment<926 else None)
            if owner is not None:
                expected=mat(('Left','Right')[owner]+'_joint6')@np.linalg.inv(ee[owner])
                max_attachment=max(max_attachment,float(np.linalg.norm(mat('broom_driver')[:3,3]-expected[:3,3])))
            active=next((r for r in ranges if r['start']<=moment<=r['end']),None)
            if active:
                m=mat('broom_driver');axis=m[:3,1]/np.linalg.norm(m[:3,1])
                max_tilt=max(max_tilt,math.degrees(math.acos(float(np.clip(axis[2],-1,1)))))
                if previous and previous[0]==active['start']:max_speed=max(max_speed,angle(m,previous[2])*25/(moment-previous[1]))
                previous=(active['start'],moment,m)
            else:previous=None
        if f%100==0:print('AUDIT',f,'events',len(events),flush=True)
    support=[]
    if last:
        scene.frame_set(last);pan=bpy.data.objects['broom_shovel'].evaluated_get(bpy.context.evaluated_depsgraph_get())
        for i in range(3):
            cube=bpy.data.objects[f'cube_{i}'].evaluated_get(bpy.context.evaluated_depsgraph_get())
            hit,point,_,_=pan.ray_cast(pan.matrix_world.inverted()@cube.matrix_world.translation,Vector((0,0,-1)))
            bottom=min((cube.matrix_world@Vector(p)).z for p in cube.bound_box)
            support.append({'cube':i,'pan_below':bool(hit),'bottom_gap_m':bottom-(pan.matrix_world@point).z if hit else None})
    dynamic=all(bpy.data.objects[f'cube_{i}'].animation_data is None and not bpy.data.objects[f'cube_{i}'].rigid_body.kinematic for i in range(3))
    q=np.array([r['q'] for r in qtrace]);grips=np.array([r['grips'] for r in qtrace])
    limits=bool(np.all(np.abs(q[:,:,:5])<=10+1e-8) and np.all(np.abs(q[:,:,5])<=3.14+1e-8) and np.all(grips>=0) and np.all(grips<=.044))
    task=bool(last==1000 and all(trace[-1]['conditions'].values()))
    supported=bool(len(support)==3 and all(x['pan_below'] and abs(x['bottom_gap_m'])<.008 for x in support))
    qualified=bool(task and supported and dynamic and limits and not events and max_attachment<1e-5 and max_tilt<=2 and max_speed<=120 and all(g['passed'] for g in gate_results))
    if reason is None:reason='success' if qualified else ('collision' if events else 'task_or_acceptance_failure')
    result={'id':args.trial,'seed':layout['seed'],'status':'completed','termination_reason':reason,'qualified':qualified,
        'initialization_valid':not init_errors,'initialization_errors':init_errors,'initial_bounds':bounds,
        'initial_poses':{n:m.tolist() for n,m in initial.items() if n in movable},'offsets_m':layout['offsets_m'],
        'executed_through_frame':last,'task_success':task,'first_success_frame':first_success,
        'conditions_at_termination':trace[-1]['conditions'] if trace else None,'grasp_gates':gate_results,
        'collision_samples':samples,'collision_events':len(events),'full_episode_audited':last==1000,
        'max_attachment_translation_error_m':max_attachment if last>=65 else None,'pan_support':support,
        'dynamic_cubes_without_animation':dynamic,'urdf_position_limits_passed':limits,
        'transfer_orientation':{'max_tilt_deg':max_tilt,'max_angular_speed_deg_s':max_speed},
        'predicate_source_sha256':source_fingerprint(),'manifest_sha256':digest(args.manifest),
        'blender_version':bpy.app.version_string,'duration_s':time.time()-started,
        'limitations':['Frozen open-loop replay, no scene-conditioned replanning','Gated kinematic attachment, not force closure','Sampled robot collision check, not continuous-time proof']}
    for filename,data in [('result.json',result),('trace.json',trace),('collision_events.json',events)]:
        (out/filename).write_text(json.dumps(data,indent=2))
    print('RESULT',json.dumps(result),flush=True)
if __name__=='__main__':main()

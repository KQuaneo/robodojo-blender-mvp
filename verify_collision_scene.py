"""Independent mesh audit and physical task replay. Never substitutes a pass."""
import sys,json,math,collections
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import bpy,numpy as np
from mathutils import Matrix,Vector
from collision_geometry import CollisionWorld
from official_checks import StateAdapter,source_fingerprint
OUT=Path(bpy.data.filepath).parent
quick='--quick' in sys.argv
scene=bpy.context.scene
joint_trace=json.loads((OUT/'joint_trace.json').read_text())
assert len(joint_trace)==1000,'Incomplete trajectory must not be accepted'
calibration=json.loads((OUT/'grasp_calibration.json').read_text())
ee=[np.array(m) for m in calibration['tool_to_flange']]
names=[f'{side}_{link}' for side in ('Left','Right') for link in ['base_link']+[f'link{i}' for i in range(1,9)]]
names+=['broom','broom_shovel','Table']+[f'cube_{i}' for i in range(3)]
world=CollisionWorld([bpy.data.objects[n] for n in names])
def evaluated(name):return bpy.data.objects[name].evaluated_get(bpy.context.evaluated_depsgraph_get())
def mat(name):return np.array(evaluated(name).matrix_world)
scene.frame_set(1)
tcp=np.eye(4);tcp[0,3]=.145
home=[mat(f'{s}_joint6')@tcp for s in ('Left','Right')]
checker=StateAdapter(mat('broom_shovel'),home)
counts=collections.Counter();events=[];trace=[];first=None;max_gap=0.;samples=0
transfer_path=OUT/'transfer_plan.json'
transfer_ranges=json.loads(transfer_path.read_text()) if transfer_path.exists() else []
max_tilt=0.;max_angular_speed=0.;previous_motion=None
for f in range(1,1001):
    scene.frame_set(f)
    poses={'pan':mat('broom_shovel'),'broom':mat('broom_driver')}
    poses.update({f'cube{i}':mat(f'cube_{i}') for i in range(3)})
    robots=[mat(f'{side}_joint6')@tcp for side in ('Left','Right')]
    conditions=checker.check(poses,robots)
    if first is None and all(conditions.values()):first=f
    trace.append({'frame':f,'positions':[poses[f'cube{i}'][:3,3].tolist() for i in range(3)],'conditions':conditions})
    if f in (65,190,400,600,820,1000):
        scene.render.filepath=str(OUT/f'frame_{f:04d}.png')
        bpy.ops.render.render(write_still=True)
    if f%100==0:print('PHYSICS',f,flush=True)
# Audit a completed physics cache so fractional-frame queries cannot alter
# the physical run used for task verification.
print('FINAL_TASK_CONDITIONS',json.dumps(trace[-1]['conditions']),flush=True)
for f in range(1,1001):
    fractions=([0] if f%5==0 else []) if quick else ([0,.25,.5,.75] if f<1000 else [0])
    for fraction in fractions:
        scene.frame_set(f,subframe=fraction);bpy.context.view_layer.update()
        matrices={n:mat(n) for n in names};hits=world.check(matrices);samples+=1
        if hits:
            events.append({'frame':f+fraction,'pairs':hits});counts.update(' / '.join(p) for p in hits)
        moment=f+fraction
        active=next((r for r in transfer_ranges if r['start']<=moment<=r['end']),None)
        if active:
            rotation=mat('broom_driver')[:3,:3]
            max_tilt=max(max_tilt,math.degrees(math.acos(float(np.clip(rotation[2,1],-1,1)))))
            if previous_motion is not None and previous_motion[0]==active['start']:
                _,last_time,last_rotation=previous_motion
                angle=math.acos(float(np.clip((np.trace(rotation@last_rotation.T)-1)/2,-1,1)))
                max_angular_speed=max(max_angular_speed,math.degrees(angle)*scene.render.fps/(moment-last_time))
            previous_motion=(active['start'],moment,rotation)
        else:previous_motion=None
        owner=0 if 65<=moment<190 else (1 if 190<=moment<926 else None)
        if owner is not None:
            expected=mat(('Left','Right')[owner]+'_joint6')@np.linalg.inv(ee[owner])
            max_gap=max(max_gap,float(np.linalg.norm(mat('broom_driver')[:3,3]-expected[:3,3])))
    if f%50==0:print('VERIFY',f,'collision_samples',len(events),'max_grasp_gap',max_gap,flush=True)
scene.frame_set(1000)
pan=evaluated('broom_shovel');support=[]
for i in range(3):
    cube=evaluated(f'cube_{i}')
    origin=pan.matrix_world.inverted()@cube.matrix_world.translation
    hit,point,normal,face=pan.ray_cast(origin,Vector((0,0,-1)))
    bottom=min((cube.matrix_world@Vector(p)).z for p in cube.bound_box)
    support.append({'cube':i,'pan_below':bool(hit),'bottom_gap_m':bottom-(pan.matrix_world@point).z if hit else None})
task_ok=all(trace[-1]['conditions'].values())
support_ok=all(p['pan_below'] and abs(p['bottom_gap_m'])<.008 for p in support)
dynamic_ok=all(bpy.data.objects[f'cube_{i}'].animation_data is None and not bpy.data.objects[f'cube_{i}'].rigid_body.kinematic for i in range(3))
joint_values=np.array([row['q'] for row in joint_trace])
grip_values=np.array([row['grips'] for row in joint_trace])
limits_ok=bool(np.all(np.abs(joint_values[:,:,:5])<=10+1e-8) and np.all(np.abs(joint_values[:,:,5])<=3.14+1e-8) and np.all(grip_values>=0) and np.all(grip_values<=.044))
motion_ok=not transfer_ranges or (max_tilt<=2 and max_angular_speed<=120)
report={'qualified':not quick and task_ok and support_ok and dynamic_ok and limits_ok and not events and max_gap<1e-5 and motion_ok,
    'task_success':task_ok,'first_success_frame':first,'conditions':trace[-1]['conditions'],
    'collision_samples':samples,'collision_events':len(events),'collision_pairs':dict(counts),
    'sampling':'integer every 5 frames' if quick else 'all integer frames and quarter-frame intervals (10ms)',
    'max_grasp_gap_m':max_gap,'pan_support':support,'dynamic_cubes_without_animation':dynamic_ok,
    'urdf_position_limits_passed':limits_ok,
    'transfer_orientation':{'checked':bool(transfer_ranges),'max_bristles_tilt_deg':max_tilt,'max_angular_speed_deg_s':max_angular_speed,'limits_passed':motion_ok},
    'predicate_source_sha256':source_fingerprint(),
    'scope':'Full robot self/inter-arm meshes, robot/tool/pan/table/cubes. Only adjacent URDF mating links and fixed base/table mounting contact excluded.',
    'limitations':['Kinematic control, not force-based grasping','Sampled mesh audit, not a formal continuous-collision proof']}
(OUT/'collision_report.json').write_text(json.dumps(report,indent=2))
(OUT/'collision_events.json').write_text(json.dumps(events,indent=2))
(OUT/'trace.json').write_text(json.dumps(trace))
print('COLLISION_REPORT',json.dumps(report),flush=True)
if not quick and not report['qualified']:
    raise RuntimeError('Collision-aware run did not pass all acceptance checks')

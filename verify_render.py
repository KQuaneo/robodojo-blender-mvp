"""Replay every step, independently check evaluated geometry, render evidence."""
import sys,json,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import bpy,numpy as np
from mathutils import Matrix,Vector
from official_checks import StateAdapter,source_fingerprint
OUT=Path(bpy.data.filepath).parent
scene=bpy.context.scene
scene.camera.data.lens=60
for light in bpy.data.lights:
    if light.energy>300:light.energy*=.20
scene.frame_set(1);bpy.context.view_layer.update()
def mat(name):return np.array(bpy.data.objects[name].evaluated_get(bpy.context.evaluated_depsgraph_get()).matrix_world)
tcp=np.eye(4);tcp[0,3]=.145
robot_names=['Left_joint6','Right_joint6']
home=[mat(n)@tcp for n in robot_names]
joint_names=[f'{side}_joint{i}' for side in ('Left','Right') for i in range(1,7)]
home_joints={n:bpy.data.objects[n].rotation_quaternion.copy() for n in joint_names}
pan_initial=mat('broom_shovel')
checker=StateAdapter(pan_initial,home)
trace=[];first=None;max_attach=0;max_attach_angle=0
grasp=[]
for z,r in [(-.095,[[-1,0,0],[0,1,0],[0,0,-1]]),(-.055,[[0,1,0],[-.8,0,-.6],[-.6,0,.8]])]:
    g=np.eye(4);g[:3,:3]=r;g[:3,3]=(0,.02,z);grasp.append(g)
selected={1,65,190,300,420,570,820,1000}
video='--video' in sys.argv
frames=OUT/'video_frames';frames.mkdir(exist_ok=True)
for f in range(1,1001):
    scene.frame_set(f)
    robots=[mat(n)@tcp for n in robot_names]
    poses={'pan':mat('broom_shovel'),'broom':mat('broom_driver')}
    poses.update({f'cube{i}':mat(f'cube_{i}') for i in range(3)})
    conditions=checker.check(poses,robots)
    owner=0 if 65<=f<190 else (1 if 190<=f<=925 else None)
    if owner is not None:
        desired=poses['broom']@grasp[owner]
        max_attach=max(max_attach,float(np.linalg.norm(desired[:3,3]-robots[owner][:3,3])))
        qa=Matrix(desired[:3,:3].tolist()).to_quaternion();qb=Matrix(robots[owner][:3,:3].tolist()).to_quaternion()
        angle=2*math.acos(min(1,abs(qa.dot(qb))))
        max_attach_angle=max(max_attach_angle,angle)
    if all(conditions.values()) and first is None:first=f
    trace.append({'frame':f,'conditions':conditions,'positions':[poses[f'cube{i}'][:3,3].tolist() for i in range(3)]})
    if f in selected or (video and (f-1)%5==0):
        scene.render.resolution_x=960;scene.render.resolution_y=720
        scene.render.filepath=str(frames/f'frame_{(f-1)//5:04d}.png') if video and (f-1)%5==0 else str(OUT/f'frame_{f:04d}.png')
        bpy.ops.render.render(write_still=True)
    if f%100==0:print('REPLAY',f,flush=True)
result=json.loads((OUT/'result.json').read_text())
assert max_attach<.00001,'Tool detached from grasp'
assert all(bpy.data.objects[f'cube_{i}'].animation_data is None for i in range(3))
assert all(not bpy.data.objects[f'cube_{i}'].rigid_body.kinematic for i in range(3))
joint_home=all(abs(home_joints[n].dot(bpy.data.objects[n].rotation_quaternion))>1-1e-6 for n in joint_names)
assert joint_home,'Arm joints did not return to the initial pose'
assert bpy.data.objects['broom_shovel'].rigid_body.collision_shape=='MESH'
pan=bpy.data.objects['broom_shovel'];pan_support=[]
for i in range(3):
    cube=bpy.data.objects[f'cube_{i}'].evaluated_get(bpy.context.evaluated_depsgraph_get())
    origin=pan.matrix_world.inverted()@cube.matrix_world.translation
    hit,point,normal,face=pan.ray_cast(origin,Vector((0,0,-1)))
    bottom=min((cube.matrix_world@Vector(v)).z for v in cube.bound_box)
    gap=bottom-(pan.matrix_world@point).z if hit else None
    pan_support.append({'cube':i,'pan_below':bool(hit),'bottom_gap_m':gap})
assert all(p['pan_below'] and abs(p['bottom_gap_m'])<.008 for p in pan_support),'Cube is not supported by pan'
result.update({'success':all(trace[-1]['conditions'].values()),'first_success_frame':first,
    'conditions':trace[-1]['conditions'],'final_positions':trace[-1]['positions'],
    'max_grasp_position_error_m':max_attach,'max_grasp_rotation_error_rad':max_attach_angle,
    'pan_support_raycast':pan_support,
    'all_arm_joints_restored':joint_home,
    'checker':'Unmodified RoboDojo predicate bodies via Blender state adapter; sufficient support-circle branch',
    'predicate_source_sha256':source_fingerprint(),'verified_by_independent_replay':True})
(OUT/'result.json').write_text(json.dumps(result,indent=2))
(OUT/'trace.json').write_text(json.dumps(trace))
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'sweep_blocks_verified.blend'))
print('VERIFIED_RESULT',json.dumps(result),flush=True)
if not result['success']:raise RuntimeError('Independent replay did not pass')

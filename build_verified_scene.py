"""Blender adaptation: measured grasp transforms, mesh dustpan, honest checks.

Not Isaac Sim and not the original RoboDojo runtime. Reuses official assets.
Tools are kinematic; cubes remain unkeyframed dynamic rigid bodies.
"""
import sys, json, math, hashlib
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_official_scene as b
import bpy
import numpy as np
from mathutils import Matrix, Vector, Quaternion
from official_checks import StateAdapter, source_fingerprint

OUT = b.ROOT / ('output_trial' if '--trial' in sys.argv else 'output_verified')
OUT.mkdir(exist_ok=True)
RENDER = '--render' in sys.argv

def pose(p, r):
    m = np.eye(4); m[:3,:3] = np.asarray(r); m[:3,3] = p
    return m

def rotvec(r):
    angle=math.acos(np.clip((np.trace(r)-1)/2,-1,1))
    skew=np.array([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])
    if angle<1e-4:return skew*.5
    if angle<math.pi-.001:return skew*angle/(2*math.sin(angle))
    q=Matrix(r.tolist()).to_quaternion()
    if q.w<0:q.negate()
    return np.array(q.axis)*q.angle

def ik(target, robot, seed, restarts=False):
    seeds = [seed]
    if restarts:
        seeds += [np.zeros(6)] + list(np.random.default_rng(42).uniform(-2.5,2.5,(22,6)))
    best = (1e9, None, None)
    for initial in seeds:
        q = np.array(initial, dtype=float).copy()
        for _ in range(160):
            cur = b.fk(q, robot['base_np'])
            e = np.r_[target[:3,3]-cur[:3,3], .18*rotvec(target[:3,:3]@cur[:3,:3].T)]
            if np.linalg.norm(e) < .00012: break
            j = np.zeros((6,6))
            for i in range(6):
                shifted=q.copy(); shifted[i]+=.0001
                f=b.fk(shifted,robot['base_np'])
                j[:,i]=np.r_[(f[:3,3]-cur[:3,3])/.0001, .18*rotvec(f[:3,:3]@cur[:3,:3].T)/.0001]
            dq=j.T@np.linalg.solve(j@j.T+np.eye(6)*.000025,e)
            q+=dq*min(1,.18/max(np.linalg.norm(dq),1e-9))
        f=b.fk(q,robot['base_np'])
        pe=np.linalg.norm(target[:3,3]-f[:3,3]); re=np.linalg.norm(rotvec(target[:3,:3]@f[:3,:3].T))
        score=pe+.18*re
        if score<best[0]: best=(score,q,(float(pe),float(re)))
        if score<.0003: break
    if best[0]>.003 and not restarts: return ik(target,robot,seed,True)
    return best[1],best[2]

def interp(a,c,t):
    qa=Matrix(a[:3,:3].tolist()).to_quaternion(); qc=Matrix(c[:3,:3].tolist()).to_quaternion()
    t=t*t*(3-2*t)
    return pose(a[:3,3]*(1-t)+c[:3,3]*t,qa.slerp(qc,t).to_matrix())

def sample(keys,f):
    for (fa,a),(fc,c) in zip(keys,keys[1:]):
        if fa<=f<=fc: return interp(a,c,(f-fa)/(fc-fa))
    return keys[0][1] if f<keys[0][0] else keys[-1][1]

def keymat(obj,f,m):
    b.key_tool(obj,f,Vector(m[:3,3]),Matrix(m[:3,:3].tolist()).to_quaternion())

def main():
    bpy.ops.wm.read_factory_settings(use_empty=True); b.linear_keyframes()
    mats={k:b.make_material(k,c) for k,c in {
        'black':(.035,.045,.06,1),'white':(.65,.72,.78,1),
        'table':(.20,.25,.28,1),'table_edge':(.08,.11,.14,1),'ground':(.04,.055,.075,1),
        'proxy':(1,0,1,1),'cube_red':(.8,.06,.03,1),'cube_green':(.04,.55,.15,1),'cube_blue':(.03,.18,.8,1)}.items()}
    b.setup_scene(mats)
    scene=bpy.context.scene; scene.render.fps=25
    scene.camera.location=(1.25,1.35,1.95)
    scene.camera.rotation_euler=(Vector((0,-.05,.91))-scene.camera.location).to_track_quat('-Z','Y').to_euler()
    scene.camera.data.lens=60
    for light in bpy.data.lights:light.energy*=.20
    scene.render.resolution_x=960;scene.render.resolution_y=720
    robots=[b.create_robot('Left',(-.3,-.45,.765),mats),b.create_robot('Right',(.3,-.45,.765),mats)]
    objects=b.create_official_objects(mats)
    pan=objects['pan']; b.add_rigid_body(pan,'PASSIVE','MESH',friction=.45)
    pan.rigid_body.collision_margin=.0001
    visual=objects['broom_visual']; visual.location=(0,0,0)
    collider=objects['broom_collider']
    # Head bounds from official mesh: x +-19mm, y -41..13mm, z 18..125mm.
    collider.scale=(.019/.016,.027/.028,.053/.018)
    bpy.context.view_layer.objects.active=collider;collider.select_set(True)
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True);collider.select_set(False)
    collider.rigid_body.collision_margin=.0001
    for cube in objects['cubes']:
        cube.rigid_body.friction=.3;cube.rigid_body.linear_damping=.15;cube.rigid_body.angular_damping=.2
    b.configure_physics();scene.rigidbody_world.substeps_per_frame=24
    scene.rigidbody_world.solver_iterations=50
    bpy.context.view_layer.update()
    pan_t=np.array(pan.matrix_world)
    initial=pose(objects['broom_initial'],objects['broom_initial_quat'].to_matrix())
    # Fixed tool-to-TCP transforms. TCP +X approaches the handle along local -Y.
    tcp_r=np.array([[0,1,0],[-1,0,0],[0,0,1]],float)
    right_tcp_r=np.array([[0,1,0],[-.8,0,-.6],[-.6,0,.8]])
    left_tcp_r=np.diag([-1.,1.,-1.])
    grasp=[pose((0,.02,-.095),left_tcp_r),pose((0,.02,-.055),right_tcp_r)]
    ee=[g@b.np_transform((-.145,0,0)) for g in grasp]
    head=b.np_transform((0,-.014,.071))
    handoff=pose((-.02,-.20,1.14),np.array([[-1,0,0],[0,0,-1],[0,-1,0]]))
    # Tool points to local +Y of the dustpan; bristles face world -Z.
    def brush(contact,direction):
        d=np.array(direction,float); d[2]=0;d/=np.linalg.norm(d)
        across=np.array([-d[1],d[0],0]); up=np.array([0,0,1])
        r=np.column_stack((across,up,d))
        return pose(np.asarray(contact)-r@head[:3,3],r)
    def world(p): return (pan_t@np.r_[p,1])[:3]
    def raised(t,h=.04):
        t=t.copy();t[2,3]+=h;return t
    toolkeys=[(1,initial),(65,initial),(115,raised(initial)),(180,handoff),(210,handoff)]
    # Approach through the open mouth, never through side/back walls.
    paths=[]
    feedback=json.loads((b.ROOT/'sweep_feedback.json').read_text())
    for i,(cfg,lane) in enumerate(zip(objects['layout']['Rigid']['small_cube'],[-.05,0,.05])):
        start=np.array(cfg['default_pos']); start[2]=.792
        stage=world((lane,-.215,0));stage[2]=.792
        final=world((lane,-.045,0));final[2]=.792
        d=stage-start;d[2]=0;d/=np.linalg.norm(d)
        p0=brush(start-d*.09,d);p1=brush(stage-d*.075,d)
        measured_stage=np.array(feedback['staging_positions'][i]);measured_stage[2]=.792
        inward=final-measured_stage;inward[2]=0;inward/=np.linalg.norm(inward)
        p2=brush(measured_stage-inward*.09,inward);p3=brush(final-inward*.071,inward)
        t=240+i*205
        toolkeys.extend([(t,raised(p0)),(t+20,p0),(t+75,p1),(t+90,raised(p1)),(t+105,raised(p2)),(t+120,p2),(t+180,p3),(t+195,raised(p3))])
        paths.append((t,p0,p1,p2,p3))
    park=initial.copy();park[:3,3]=(.31,.10,initial[2,3])
    toolkeys.extend([(890,raised(park)),(925,park),(1000,park)])
    homes=[b.fk(np.zeros(6),r['base_np']) for r in robots]
    qseed=[np.zeros(6),np.zeros(6)]; errors=[]; return_q={}
    # The left hand holds the pan handle after passing the broom.
    pan_grasp=pose(world((0,.10,.025)),np.array([[0,1,0],[0,0,-1],[-1,0,0]],float))@b.np_transform((-.145,0,0))
    left_release=handoff@ee[0]
    left_away=raised(left_release,.1)
    right_approach=raised(handoff@ee[1],.04)
    for f in range(1,1001):
        desired=sample(toolkeys,f)
        if f<=65: lt=interp(homes[0],initial@ee[0],min(1,(f-1)/55))
        elif f<=190: lt=desired@ee[0]
        elif f<=215: lt=interp(left_release,left_away,(f-190)/25)
        elif f<=255: lt=interp(left_away,pan_grasp,(f-215)/40)
        elif f<=885: lt=pan_grasp
        else: lt=interp(pan_grasp,homes[0],min(1,(f-885)/90))
        if f<=140: rt=homes[1]
        elif f<=175: rt=interp(homes[1],right_approach,(f-140)/35)
        elif f<=190: rt=interp(right_approach,handoff@ee[1],(f-175)/15)
        elif f<=925: rt=desired@ee[1]
        else: rt=interp(park@ee[1],homes[1],min(1,(f-925)/65))
        actual=[]
        for i,target in enumerate((lt,rt)):
            return_start=885 if i==0 else 925
            duration=90 if i==0 else 65
            if f>return_start:
                t=min(1,(f-return_start)/duration);t=t*t*(3-2*t)
                q=return_q[i]*(1-t);err=(0.,0.)
            else:
                q,err=ik(target,robots[i],qseed[i])
                errors.append((f,i,*err))
            qseed[i]=q
            if f==return_start:return_q[i]=(q+math.pi)%(2*math.pi)-math.pi
            if err[0]>.006 or err[1]>.04:
                print('IK_FAIL',f,i,err,'TARGET',target.tolist(),flush=True)
                (OUT/'ik_failure.json').write_text(json.dumps({'frame':f,'arm':i,'error':err}))
                raise RuntimeError(f'Unreachable frame {f}, arm {i}: {err}')
            gripping=(65<=f<=190) if i==0 else (190<=f<=925)
            if i==0 and 255<=f<=885: gripping=True
            b.set_joint_pose(robots[i],q,-.009 if gripping else .025,f)
            actual.append(b.fk(q,robots[i]['base_np']))
        owner=0 if 65<=f<190 else (1 if 190<=f<=925 else None)
        tool=desired if owner is None else actual[owner]@np.linalg.inv(ee[owner])
        keymat(objects['broom_driver'],f,tool);keymat(collider,f,tool@head)
        if f%100==0: print('MOTION',f,flush=True)
    max_pos=max(x[2] for x in errors);max_rot=max(x[3] for x in errors)
    print('IK_DONE',max_pos,max_rot,flush=True)
    # Verify evaluated transforms, not planner targets. Log every physics step.
    support=world((0,-.053,-.015));trace=[];first=None
    tcp_offset=b.np_transform((.145,0,0))
    checker=StateAdapter(pan_t,[h@tcp_offset for h in homes])
    for f in range(1,1001):
        scene.frame_set(f)
        deps=bpy.context.evaluated_depsgraph_get()
        positions=[np.array(c.evaluated_get(deps).matrix_world.translation) for c in objects['cubes']]
        dist=[float(np.linalg.norm(p[:2]-support[:2])) for p in positions]
        home_checks=[];robot_poses=[]
        for r,h in zip(robots,homes):
            actual=np.array(r['frames']['link6'].evaluated_get(deps).matrix_world)
            robot_poses.append(actual@tcp_offset)
            home_checks.append(bool(np.all(np.abs(actual[:3,3]-h[:3,3])<=.15) and np.linalg.norm(rotvec(actual[:3,:3]@h[:3,:3].T))<=math.radians(20)))
        pan_actual=np.array(pan.evaluated_get(deps).matrix_world)
        tool_actual=np.array(objects['broom_driver'].evaluated_get(deps).matrix_world)
        poses={'pan':pan_actual,'broom':tool_actual}
        poses.update({f'cube{i}':np.array(c.evaluated_get(deps).matrix_world) for i,c in enumerate(objects['cubes'])})
        conditions=checker.check(poses,robot_poses)
        success=all(conditions.values())
        if success and first is None:first=f
        trace.append({'frame':f,'positions':[p.tolist() for p in positions],'distances':dist,'conditions':conditions})
        if RENDER and f in (1,190,570,1000):
            scene.render.filepath=str(OUT/f'frame_{f:04d}.png');bpy.ops.render.render(write_still=True)
    result={'success':all(trace[-1]['conditions'].values()),'first_success_frame':first,
        'conditions':trace[-1]['conditions'],'final_positions':trace[-1]['positions'],
        'max_ik_position_error_m':max_pos,'max_ik_rotation_error_rad':max_rot,
        'checker':'Unmodified original predicate bodies through Blender state adapter; support-circle OR branch only; not Isaac Sim runtime',
        'predicate_source_sha256':source_fingerprint(),
        'physics':'Official dustpan triangle mesh; brush head box fitted to official geometry; dynamic cubes',
        'boundaries':['Kinematic arm/tool control','No robot collision planning','No VLA training']}
    (OUT/'result.json').write_text(json.dumps(result,indent=2))
    (OUT/'trace.json').write_text(json.dumps(trace))
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'sweep_blocks_verified.blend'))
    print('RESULT',json.dumps(result),flush=True)
    if not result['success']:raise RuntimeError('Final task predicates did not pass; see result.json')

if __name__ == '__main__':
    main()

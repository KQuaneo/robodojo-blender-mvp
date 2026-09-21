"""Replace free sweeping transfers with bristles-down Cartesian paths.

Read the accepted collision scene; retain every contact stroke and all left
arm motion. Never fall back silently to unconstrained joint-space motion.
"""
import sys,json,math,shutil
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import bpy,numpy as np
import build_official_scene as b
import build_verified_scene as v
import plan_collision_scene as p

source=Path(bpy.data.filepath).parent
assert json.loads((source/'collision_report.json').read_text())['qualified'], 'Input must be an accepted collision-aware run'
out=source.parent/'output_smooth';out.mkdir(exist_ok=True)
p.OUT=out
scene=bpy.context.scene;scene.frame_set(1);b.linear_keyframes()
robots=[]
for side in ('Left','Right'):
    root=bpy.data.objects[side+'_root']
    joints={j.name:bpy.data.objects[side+'_'+j.name] for j in b.ARM_JOINTS+b.GRIPPER_JOINTS}
    robots.append({'prefix':side,'root':root,'joints':joints,
        'frames':{'base_link':root,**{j.child:joints[j.name] for j in b.ARM_JOINTS+b.GRIPPER_JOINTS}},
        'meshes':{n:bpy.data.objects[side+'_'+n] for n in ['base_link']+[f'link{i}' for i in range(1,9)]},
        'base_np':np.array(root.matrix_world)})
objects={'broom_visual':bpy.data.objects['broom'],'pan':bpy.data.objects['broom_shovel']}
planner=p.Planner(robots,objects)
cal=json.loads((source/'grasp_calibration.json').read_text())
planner.ee=[np.array(m) for m in cal['tool_to_flange']];planner.carry=1
trace=json.loads((source/'joint_trace.json').read_text())
ranges=[(320,350),(420,445),(525,555),(625,650),(730,760),(830,890)]
def tool(q):return b.fk(q,robots[1]['base_np'])@np.linalg.inv(planner.ee[1])
diagnostics=[]
for start,end in ranges:
    qa=np.array(trace[start-1]['q'][1]);qc=np.array(trace[end-1]['q'][1])
    a,c=tool(qa),tool(qc)
    best=None
    # Start with the direct Cartesian route, then small raised detours. The
    # orientation always follows the shortest arc between the brush headings.
    for lift in (0,.015,.03,.05):
        prev=qa.copy();route=[qa];failure=None
        for step in range(1,(end-start)*4+1):
            t=step/((end-start)*4);m=v.interp(a,c,t)
            m[2,3]+=lift*math.sin(math.pi*t)**2
            f=start+step/4;row=trace[min(end-1,int(f)-1)]
            planner.q=[np.array(row['q'][0]),prev];planner.grips=row['grips'];planner.tool=m;planner.frame=f
            q,err=p.single_ik(m@planner.ee[1],robots[1],prev)
            q=prev+(q-prev+math.pi)%(2*math.pi)-math.pi
            actual=tool(q)
            if err[0]>.0003 or err[1]>.003 or np.max(np.abs(q-prev))>.12:
                failure=('continuity',f,err);break
            if planner.hits(1,q):failure=('collision',f,planner.hits(1,q));break
            if planner.world.geometry['broom'].bounds(actual)[0][2]<.811:
                failure=('clearance',f);break
            if actual[2,1]<math.cos(math.radians(2)):
                failure=('tilt',f);break
            mats=planner.matrices(1,q)
            if any(p.intersects(planner.world.geometry['broom'],actual,planner.world.geometry[n],mats[n]) for n in ('Table','broom_shovel')):
                failure=('tool_collision',f);break
            route.append(q);prev=q
        if failure is None and np.max(np.abs(route[-1]-qc))<.02:
            best=route;break
        print('CARTESIAN_REJECT',start,lift,failure,'end_delta',float(np.max(np.abs(prev-qc))),flush=True)
    if best is None:raise RuntimeError(f'No bristles-down continuous route for {start}..{end}')
    # Remove old fractional path corners too; otherwise they override new keys.
    for obj in robots[1]['joints'].values():
        if obj.animation_data and obj.animation_data.action:
            for layer in obj.animation_data.action.layers:
                for strip in layer.strips:
                    for bag in strip.channelbags:
                        for fc in bag.fcurves:
                            for key in list(fc.keyframe_points):
                                if start<key.co.x<end:fc.keyframe_points.remove(key,fast=True)
                            fc.update()
    for k,q in enumerate(best):
        frame=start+k/4;grip=trace[min(end-1,int(frame)-1)]['grips'][1]
        b.set_joint_pose(robots[1],q,grip,frame)
        if k%4==0:trace[int(frame)-1]['q'][1]=q.tolist()
    diagnostics.append({'start':start,'end':end,'lift_m':lift,'samples':len(best)})
    print('CARTESIAN_ACCEPT',diagnostics[-1],flush=True)
for name in ('grasp_calibration.json',):shutil.copy2(source/name,out/name)
(out/'joint_trace.json').write_text(json.dumps(trace))
(out/'transfer_plan.json').write_text(json.dumps(diagnostics,indent=2))
scene.frame_set(1)
bpy.ops.wm.save_as_mainfile(filepath=str(out/'sweep_blocks_smooth.blend'))

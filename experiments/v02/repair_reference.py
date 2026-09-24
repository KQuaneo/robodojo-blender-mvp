"""Create v0.2 without altering the frozen v0.1 scene or files."""
import json
import sys
import hashlib
import shutil
import argparse
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path[:0]=[str(ROOT),str(HERE)]
import bpy
import numpy as np
import build_official_scene as b
import motion

parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args(sys.argv[sys.argv.index('--')+1:])
OUT=args.output.resolve()
assert OUT.parent==HERE, 'Use a fresh candidate directory inside experiments/v02'
OUT.mkdir(exist_ok=False)
source=ROOT/'output_smooth'
old=json.loads((source/'joint_trace.json').read_text())
scene=bpy.context.scene;scene.frame_set(1);b.linear_keyframes()
ranges=[(320,350),(420,445),(525,555),(625,650),(730,760),(830,890)]
removed=[]
for j in b.ARM_JOINTS:
    obj=bpy.data.objects['Right_'+j.name]
    for (path,channel),fc in motion.curves(obj).items():
        if path!='rotation_quaternion':continue
        for index in range(len(fc.keyframe_points)-1,-1,-1):
            key=fc.keyframe_points[index];t=float(key.co.x)
            if any(a<t<c for a,c in ranges) and abs(t*4-round(t*4))>1e-5:
                removed.append(dict(object=obj.name,channel=channel,frame=t,value=float(key.co.y)))
                fc.keyframe_points.remove(key,fast=True)
        fc.update()
fcs=motion.rotation_curves()
times=sorted({215.,240.}|{float(p.co.x) for joint in fcs[1] for fc in joint.values()
                         for p in fc.keyframe_points if 215<p.co.x<240})
route=[motion.read_q(t,1,fcs,old) for t in times]
lengths=[float(np.max(np.abs(c-a))) for a,c in zip(route,route[1:])]
total=sum(lengths);assert total>0
new_times=[215.]+[215+25*sum(lengths[:k])/total for k in range(1,len(route))]
right=dict(joints={j.name:bpy.data.objects['Right_'+j.name] for j in b.ARM_JOINTS+b.GRIPPER_JOINTS})
for j in b.ARM_JOINTS:
    for fc in motion.curves(right['joints'][j.name]).values():
        for index in range(len(fc.keyframe_points)-1,-1,-1):
            if 215<fc.keyframe_points[index].co.x<240:fc.keyframe_points.remove(fc.keyframe_points[index],fast=True)
        fc.update()
for t,q in zip(new_times,route):
    b.set_joint_pose(right,q,old[214]['grips'][1],t)
fcs=motion.rotation_curves()
# Setting the insertion preference does NOT convert existing Blender key data.
# Explicitly set each actual rotation key, including newly inserted keys.
interpolation_changes={}
for arm in fcs:
    for joint in arm:
        for (path,channel),fc in joint.items():
            if path!='rotation_quaternion':continue
            for key in fc.keyframe_points:
                if key.interpolation!='LINEAR':
                    interpolation_changes[key.interpolation]=interpolation_changes.get(key.interpolation,0)+1
                key.interpolation='LINEAR'
            fc.update()
continuity=motion.rate_audit(fcs)
report=dict(version='v0.2-candidate',source_scene_sha256=hashlib.sha256((source/'sweep_blocks_smooth.blend').read_bytes()).hexdigest(),
            removed_stale_rotation_keys=removed,explicit_interpolation_changes=interpolation_changes,
            retiming=dict(old_times=times,new_times=new_times,
            route_q=[q.tolist() for q in route],max_joint_arclength=total),continuity=continuity)
(OUT/'repair_report.json').write_text(json.dumps(report,indent=2))
if not continuity['passed']:
    print('REFERENCE_REJECTED',json.dumps(continuity),flush=True)
    raise SystemExit(0)
trace=[dict(frame=f,q=[motion.read_q(f,i,fcs,old).tolist() for i in (0,1)],grips=old[f-1]['grips']) for f in range(1,1001)]
(OUT/'joint_trace.json').write_text(json.dumps(trace))
for name in ('grasp_calibration.json','transfer_plan.json'):
    shutil.copy2(source/name,OUT/name)
scene.frame_set(1)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'reference.blend'),compress=True)
print('REPAIRED',json.dumps(dict(removed_keys=len(removed),continuity=continuity)),flush=True)

"""Confirm rate-diagnostic FK against evaluated Blender poses, physics disabled."""
import json
import math
import sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0,str(HERE))
import bpy
import numpy as np
from mathutils import Euler,Quaternion
import plan_b
import build_official_scene as b

scene = bpy.context.scene
scene.rigidbody_world.enabled = False
ref = json.loads((ROOT/'output_smooth/joint_trace.json').read_text())
fcs = [plan_b.curves(bpy.data.objects['Right_'+j.name]) for j in b.ARM_JOINTS]
rows = []
for t in (215.25,215.375,544.25,544.4421997070312,544.4710998535156,544.5):
    q = []
    for i,spec in enumerate(b.ARM_JOINTS):
        rot = Quaternion([fcs[i]['rotation_quaternion',k].evaluate(t) for k in range(4)]).normalized()
        var = Euler(spec.rpy,'XYZ').to_quaternion().conjugated()@rot
        a = 2*math.atan2(float(np.dot([var.x,var.y,var.z],spec.axis)),var.w)
        a += 2*math.pi*round((ref[int(t)-1]['q'][1][i]-a)/(2*math.pi))
        q.append(a)
    scene.frame_set(int(t),subframe=t-int(t))
    deps = bpy.context.evaluated_depsgraph_get()
    flange = np.array(bpy.data.objects['Right_joint6'].evaluated_get(deps).matrix_world)
    predicted = b.fk(q,np.array(bpy.data.objects['Right_root'].matrix_world))
    tool = np.array(bpy.data.objects['broom_driver'].evaluated_get(deps).matrix_world)
    rows.append(dict(frame=t,q=q,flange_fk_max_matrix_error=float(np.max(np.abs(flange-predicted))),
                     broom_world_matrix=tool.tolist()))
for a,c in zip(rows,rows[1:]):
    if c['frame']-a['frame']>1:
        continue
    ra=np.array(a['broom_world_matrix'])[:3,:3]
    rc=np.array(c['broom_world_matrix'])[:3,:3]
    ua,_,va=np.linalg.svd(ra)
    uc,_,vc=np.linalg.svd(rc)
    angle=math.degrees(math.acos(float(np.clip((np.trace((ua@va).T@(uc@vc))-1)/2,-1,1))))
    c['broom_angle_from_previous_deg']=angle
    c['broom_angular_speed_from_previous_deg_s']=angle*25/(c['frame']-a['frame'])
target = HERE/'b_results/reference_pose_diagnostic.json'
with target.open('x') as f:
    json.dump(dict(physics_enabled=False,read_only_scene=True,poses=rows),f,indent=2)
print('POSE_DIAGNOSTIC',json.dumps([dict(frame=r['frame'],fk_error=r['flange_fk_max_matrix_error'],
      angular_speed=r.get('broom_angular_speed_from_previous_deg_s')) for r in rows]),flush=True)

"""Measure actual animated broom motion, including fractional keys."""
import json,math
from pathlib import Path
import bpy,numpy as np
out=Path(bpy.data.filepath).parent
ranges=[(320,350),(420,445),(525,555),(625,650),(730,760),(830,890)]
rows=[]
for start,end in ranges:
    tilt=0.;speed=0.;total=0.;previous=None
    for moment in np.arange(start,end+.01,.25):
        bpy.context.scene.frame_set(int(moment),subframe=float(moment%1))
        r=np.array(bpy.data.objects['broom_driver'].evaluated_get(bpy.context.evaluated_depsgraph_get()).matrix_world)[:3,:3]
        tilt=max(tilt,math.degrees(math.acos(float(np.clip(r[2,1],-1,1)))))
        if previous is not None:
            delta=math.degrees(math.acos(float(np.clip((np.trace(r@previous.T)-1)/2,-1,1))))
            speed=max(speed,delta*100);total+=delta
        previous=r
    rows.append({'start':start,'end':end,'max_tilt_deg':tilt,'max_speed_deg_s':speed,'total_rotation_deg':total})
(out/'transfer_motion.json').write_text(json.dumps(rows,indent=2))
print('TRANSFER_MOTION',json.dumps(rows),flush=True)

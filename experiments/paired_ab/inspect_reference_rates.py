"""Read-only nominal animation diagnosis; no simulation or seed initialization."""
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

reference = json.loads((ROOT/'output_smooth/joint_trace.json').read_text())
curves = [[plan_b.curves(bpy.data.objects[side+'_'+j.name]) for j in b.ARM_JOINTS]
          for side in ('Left','Right')]
times = {1+k/4 for k in range(3997)}
for arm in curves:
    for joint in arm:
        for fc in joint.values():
            times.update(float(k.co.x) for k in fc.keyframe_points if 1<=k.co.x<=1000)
times = sorted(times)
samples = sorted(set(times)|{(a+c)/2 for a,c in zip(times,times[1:])})
def read_q(t,arm):
    anchors = reference[min(999,max(0,int(t)-1))]['q'][arm]
    angles = []
    for i,spec in enumerate(b.ARM_JOINTS):
        rot = Quaternion([curves[arm][i]['rotation_quaternion',k].evaluate(t) for k in range(4)]).normalized()
        var = Euler(spec.rpy,'XYZ').to_quaternion().conjugated()@rot
        a = 2*math.atan2(float(np.dot([var.x,var.y,var.z],spec.axis)),var.w)
        angles.append(a+2*math.pi*round((anchors[i]-a)/(2*math.pi)))
    return np.array(angles)
violations = []
maxima = [dict(rate=0),dict(rate=0)]
previous = [read_q(samples[0],i) for i in (0,1)]
for a,t in zip(samples,samples[1:]):
    q = [read_q(t,i) for i in (0,1)]
    for i in (0,1):
        delta = np.abs(q[i]-previous[i])
        joint = int(np.argmax(delta))
        rate = float(delta[joint]/(t-a))
        record = dict(start_frame=a,end_frame=t,arm=('Left','Right')[i],joint=joint+1,
                      rate=rate,delta_rad=float(delta[joint]))
        if rate>maxima[i]['rate']:
            maxima[i] = record
        if delta[joint]>.48*(t-a)+1e-5:
            violations.append(record)
    previous = q
result = dict(purpose='Read-only diagnosis after nominal B control rejection; NOT a rerun',
              samples=len(samples),arm_maxima=maxima,threshold_rad_per_frame=.48,
              violation_intervals=violations)
target = HERE/'b_results/reference_rate_diagnostic.json'
with target.open('x') as f:
    json.dump(result,f,indent=2)
print('REFERENCE_RATE_DIAGNOSTIC',json.dumps(dict(arm_maxima=maxima,violations=len(violations),
      first=violations[0] if violations else None)),flush=True)

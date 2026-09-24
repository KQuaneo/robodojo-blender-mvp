"""Blender animation reading and key-aware continuity audit, no scene writes."""
import math
import bpy
import numpy as np
from mathutils import Euler,Quaternion
import build_official_scene as b

def curves(obj):
    action = obj.animation_data.action if obj.animation_data else None
    return {(fc.data_path,fc.array_index):fc for layer in action.layers
            for strip in layer.strips for bag in strip.channelbags for fc in bag.fcurves} if action else {}

def rotation_curves():
    return [[curves(bpy.data.objects[s+'_'+j.name]) for j in b.ARM_JOINTS] for s in ('Left','Right')]

def read_q(t,arm,fcs,anchors):
    result=[]
    anchor=anchors[min(999,max(0,int(t)-1))]['q'][arm]
    for i,j in enumerate(b.ARM_JOINTS):
        rot=Quaternion([fcs[arm][i]['rotation_quaternion',k].evaluate(t) for k in range(4)]).normalized()
        variable=Euler(j.rpy,'XYZ').to_quaternion().conjugated()@rot
        a=2*math.atan2(float(np.dot([variable.x,variable.y,variable.z],j.axis)),variable.w)
        result.append(a+2*math.pi*round((anchor[i]-a)/(2*math.pi)))
    return np.array(result)

def audit_times(fcs,last=1000):
    times={1+k/4 for k in range(max(0,(last-1)*4+1))}
    for arm in fcs:
        for joint in arm:
            for fc in joint.values():
                times.update(float(p.co.x) for p in fc.keyframe_points if 1<=p.co.x<=last)
    ordered=sorted(times)
    return sorted(times|{(a+c)/2 for a,c in zip(ordered,ordered[1:])})

def rate_audit(fcs):
    """Exact angular derivative maximum for piecewise LINEAR quaternion chords."""
    maximum=dict(rate_rad_per_frame=0.)
    violations=[]
    segments=0
    for arm,joints in enumerate(fcs):
        for joint,fcmap in enumerate(joints):
            channels=[fcmap['rotation_quaternion',k] for k in range(4)]
            knots=sorted({float(p.co.x) for fc in channels for p in fc.keyframe_points if 1<=p.co.x<=1000})
            assert all(p.interpolation=='LINEAR' for fc in channels for p in fc.keyframe_points), 'Nonlinear quaternion segment'
            for a,c in zip(knots,knots[1:]):
                u=np.array([fc.evaluate(a) for fc in channels],float)
                v=np.array([fc.evaluate(c) for fc in channels],float)
                delta=v-u;dd=float(delta@delta)
                t=float(np.clip(-float(u@delta)/dd,0,1)) if dd>1e-20 else 0.
                minimum=float((u+t*delta)@(u+t*delta))
                # Stable Gram determinant via orthogonal component.
                uu=float(u@u)
                perpendicular=v-u*float(u@v)/uu
                determinant=uu*float(perpendicular@perpendicular)
                rate=2*math.sqrt(max(0,determinant))/minimum/(c-a) if minimum>1e-12 else float('inf')
                item=dict(arm=('Left','Right')[arm],joint=joint+1,start_frame=a,end_frame=c,
                          rate_rad_per_frame=rate,min_quaternion_norm_sq=minimum)
                segments+=1
                if rate>maximum['rate_rad_per_frame']:maximum=item
                if rate>.48001:violations.append(item)
    return dict(passed=not violations,segments=segments,maximum=maximum,violations=violations,
                bound_rad_per_frame=.48,method='analytic normalized-linear quaternion segment derivative')

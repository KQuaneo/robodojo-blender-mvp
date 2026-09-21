"""Offline physics-only policy trials, NOT a verified robot demonstration."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import bpy,numpy as np
from mathutils import Vector,Matrix
import build_verified_scene as v
source=bpy.data.filepath;out=Path(source).parent
poses=[]
for f in range(1,1001):
    bpy.context.scene.frame_set(f)
    poses.append(np.array(bpy.data.objects['broom_driver'].matrix_world))
verts=np.array([v.co[:] for v in bpy.data.objects['broom'].data.vertices])
lo,hi=verts.min(0),verts.max(0)
corners=np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
flight=[(215,240),(320,350),(420,445),(525,555),(625,650),(730,760),(830,890)]
blue_only='--blue-only' in sys.argv
bumps=[(730,760,830,890)] if blue_only else [(320,350,420,445),(525,555,625,650),(730,760,830,890)]
results=[]
for height in ((-.016,-.012,-.008,-.004,0,.004) if blue_only else (0,.004,.008,.012,.016,.020)):
    bpy.ops.wm.open_mainfile(filepath=source)
    scene=bpy.context.scene;driver=bpy.data.objects['broom_driver']
    driver.constraints.clear();driver.animation_data_clear()
    for f,original in enumerate(poses,1):
        m=original.copy();weight=0.
        for begin,full,release,end in bumps:
            if begin<=f<=end:
                weight=min(1,(f-begin)/(full-begin),(end-f)/(end-release))
                weight=max(0,weight);weight=weight*weight*(3-2*weight)
        m[2,3]+=height*weight
        if any(a<=f<=c for a,c in flight):
            bottom=(corners@m[:3,:3].T+m[:3,3])[:,2].min()
            m[2,3]+=max(0,.812-bottom)
        v.keymat(driver,f,m)
    for f in range(1,1001):scene.frame_set(f)
    deps=bpy.context.evaluated_depsgraph_get()
    positions=[np.array(bpy.data.objects[f'cube_{i}'].evaluated_get(deps).matrix_world.translation) for i in range(3)]
    pan=bpy.data.objects['broom_shovel'];center=np.array(pan.matrix_world@Vector((0,-.053,-.015)))
    distances=[float(np.linalg.norm(p[:2]-center[:2])) for p in positions]
    result={'height':height,'all_in_circle':all(d<.081 for d in distances),'distances':distances,'positions':[p.tolist() for p in positions]}
    results.append(result);print('PHYSICS_TRIAL',json.dumps(result),flush=True)
    (out/('blue_height_trials.json' if blue_only else 'contact_height_trials.json')).write_text(json.dumps(results,indent=2))

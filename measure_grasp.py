import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import bpy,numpy as np
from mathutils import Vector
import build_official_scene as b
from collision_geometry import Geometry
out=Path(bpy.data.filepath).parent
data=json.loads((out/'grasp_calibration.json').read_text())
tool=Geometry(bpy.data.objects['broom']);gaps=[]
for arm,side in enumerate(('Left','Right')):
    ee=np.array(data['tool_to_flange'][arm]);opening=data['openings'][arm];distances=[]
    for joint in b.GRIPPER_JOINTS:
        obj=bpy.data.objects[f'{side}_{joint.child}']
        a=np.array([v.co[:] for v in obj.data.vertices]);obj.data.calc_loop_triangles()
        centers=np.array([a[list(t.vertices)].mean(0) for t in obj.data.loop_triangles])
        a=np.concatenate((a,centers));a=a[a[:,0]>.054]
        m=ee@b.np_transform(np.array(joint.xyz)+np.array(joint.axis)*opening)
        a=a@m[:3,:3].T+m[:3,3]
        distances.append(min(tool.tree.find_nearest(Vector(p))[3] for p in a))
    gaps.append(distances)
data['sampled_finger_surface_gaps_m']=gaps
(out/'grasp_calibration.json').write_text(json.dumps(data,indent=2))
print('FINGER_GAPS',gaps)

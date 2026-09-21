import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import bpy,numpy as np
from collision_geometry import Geometry,intersects,robot_pair_allowed
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.mesh.primitive_cube_add(size=2)
a=Geometry(bpy.context.object)
bpy.ops.mesh.primitive_cube_add(size=.4)
b=Geometry(bpy.context.object)
identity=np.eye(4);m=np.eye(4)
assert intersects(a,identity,b,m),'Containment must be detected'
m[0,3]=1
assert intersects(a,identity,b,m),'Surface crossing must be detected'
m[0,3]=1.201
assert not intersects(a,identity,b,m),'Separated objects must remain valid'
assert robot_pair_allowed('Left_link3','Left_link4')
assert not robot_pair_allowed('Left_link7','Left_link8')
assert not robot_pair_allowed('Left_link3','Right_link3')
print('PASS: separation, penetration, containment and narrow adjacency exclusions')

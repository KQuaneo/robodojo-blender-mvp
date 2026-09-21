"""Triangle-mesh collision queries; adjacent URDF links alone are exempt.

Planning/auditing, not a force/contact solver. Tools are never exempted from
finger checks: intended grasps must maintain a small non-penetrating gap.
"""
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree

class Geometry:
    def __init__(self,obj):
        self.name=obj.name
        self.vertices=np.array([v.co[:] for v in obj.data.vertices],dtype=float)
        obj.data.calc_loop_triangles()
        self.faces=[tuple(t.vertices) for t in obj.data.loop_triangles]
        self.tree=BVHTree.FromPolygons(self.vertices.tolist(),self.faces,all_triangles=True)
        lo=self.vertices.min(0);hi=self.vertices.max(0)
        self.corners=np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
    def bounds(self,m):
        a=self.corners@m[:3,:3].T+m[:3,3]
        return a.min(0),a.max(0)

def intersects(a,ma,b,mb):
    alo,ahi=a.bounds(ma);blo,bhi=b.bounds(mb)
    if np.any(ahi<blo-1e-7) or np.any(bhi<alo-1e-7):return False
    # Transform the smaller mesh, keeping the larger cached BVH in local space.
    if len(a.faces)>len(b.faces):a,b,ma,mb=b,a,mb,ma
    rel=np.linalg.inv(mb)@ma
    v=a.vertices@rel[:3,:3].T+rel[:3,3]
    tree=BVHTree.FromPolygons(v.tolist(),a.faces,all_triangles=True)
    if tree.overlap(b.tree):return True
    # Surface intersections alone miss an object completely enclosed by another.
    def inside(bvh,p):
        votes=0
        for direction in ((1,.371,.193),(.217,1,.419),(.323,.157,1)):
            d=Vector(direction).normalized();origin=Vector(p);hits=0
            for _ in range(128):
                location,normal,index,distance=bvh.ray_cast(origin,d)
                if location is None:break
                hits+=1;origin=location+d*1e-6
            votes+=hits%2
        return votes>=2
    for points,bvh in ((v,b.tree),(b.vertices,tree)):
        for i in (0,len(points)//2,len(points)-1):
            if inside(bvh,points[i]):return True
    return False

def robot_pair_allowed(a,b):
    sa,la=a.split('_',1);sb,lb=b.split('_',1)
    if sa!=sb:return False
    def index(s):return 0 if s=='base_link' else int(s[4:])
    ia,ib=index(la),index(lb)
    return (abs(ia-ib)==1 and max(ia,ib)<=6) or (min(ia,ib)==6 and max(ia,ib) in (7,8))

class CollisionWorld:
    def __init__(self,objects):
        self.objects={o.name:o for o in objects}
        self.geometry={n:Geometry(o) for n,o in self.objects.items()}
        self.robot_names=[n for n in self.objects if n.startswith(('Left_','Right_'))]
        self.pairs=[]
        for i,a in enumerate(self.robot_names):
            for b in self.robot_names[i+1:]:
                if not robot_pair_allowed(a,b):self.pairs.append((a,b))
            for b in (n for n in self.objects if n not in self.robot_names):
                # Base mounting face rests on the table by design.
                if b=='Table' and a.endswith('base_link'):continue
                self.pairs.append((a,b))
    def check(self,matrices=None):
        if matrices is None:matrices={n:np.array(o.matrix_world) for n,o in self.objects.items()}
        result=[]
        for a,b in self.pairs:
            if intersects(self.geometry[a],matrices[a],self.geometry[b],matrices[b]):result.append((a,b))
        return result

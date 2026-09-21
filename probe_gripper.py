import bpy,numpy as np
for n in ['Left_link6','Left_link7','Left_link8','broom','broom_shovel']:
    o=bpy.data.objects[n];a=np.array([v.co[:] for v in o.data.vertices])
    print('BOUNDS',n,a.min(0),a.max(0),flush=True)
    if n.endswith(('link7','link8')):
        for x in [.02,.035,.05,.06,.07,.08]:
            v=a[(a[:,0]>x-.005)&(a[:,0]<x+.005)]
            if len(v):print('SLICE',n,x,v.min(0),v.max(0),flush=True)

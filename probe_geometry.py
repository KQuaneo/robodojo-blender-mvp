import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_official_scene as b
import bpy, numpy as np
for name in ['broom', 'broom_shovel']:
    obj=b.import_usdz(b.ASSETS/'objects'/name/'object.usdz',name)
    a=np.array([v.co[:] for v in obj.data.vertices])
    print('GEOMETRY',name,'scale',obj.scale[:], 'bounds',a.min(0),a.max(0))
    for z in np.linspace(a[:,2].min(),a[:,2].max(),8)[:-1]:
        sub=a[(a[:,2]>=z)&(a[:,2]<z+(a[:,2].max()-a[:,2].min())/7)]
        print('SLICE',z,sub.min(0),sub.max(0))

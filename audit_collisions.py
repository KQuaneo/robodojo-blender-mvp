import sys,json,collections
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import bpy,numpy as np
from collision_geometry import CollisionWorld

argv=sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else []
stride=int(argv[0]) if argv else 5
out=Path(__file__).resolve().parent/'output_collision'
out.mkdir(exist_ok=True)
names=[f'{s}_{l}' for s in ('Left','Right') for l in ['base_link']+[f'link{i}' for i in range(1,9)]]
names+=['broom','broom_shovel','Table']
world=CollisionWorld([bpy.data.objects[n] for n in names])
print('GEOMETRY',[(n,len(g.faces)) for n,g in world.geometry.items()],flush=True)
counts=collections.Counter();events=[]
for frame in sorted(set(range(1,1001,stride))|{65,190,255,925,1000}):
    bpy.context.scene.frame_set(frame);bpy.context.view_layer.update()
    hits=world.check()
    if hits:
        events.append({'frame':frame,'pairs':hits})
        counts.update(' / '.join(p) for p in hits)
    if frame%100==1:print('AUDIT',frame,'hits',hits,flush=True)
report={'stride':stride,'counts':dict(counts),'events':events}
(out/'baseline_collisions.json').write_text(json.dumps(report,indent=2))
print('SUMMARY',json.dumps(dict(counts)),flush=True)

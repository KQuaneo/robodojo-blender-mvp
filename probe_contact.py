import bpy,numpy as np
from mathutils import Vector
p=bpy.data.objects['broom_shovel'];a=np.array([v.co[:] for v in p.data.vertices])
for y in np.arange(-.14,-.03,.01):
    s=a[(np.abs(a[:,0])<.06)&(a[:,1]>=y)&(a[:,1]<y+.01)]
    if len(s):print('PAN_FLOOR',y,s[:,2].min(),s[:,2].max(),flush=True)
for f in range(1,446):
    bpy.context.scene.frame_set(f)
    if f in (350,360,365,370,375,380,385,390,400,430,435,440,445):
        d=bpy.context.evaluated_depsgraph_get()
        head=bpy.data.objects['broom_collision'].evaluated_get(d).matrix_world
        visual=bpy.data.objects['broom_driver'].evaluated_get(d).matrix_world
        expected=visual@Vector((0,-.014,.071))
        cube=bpy.data.objects['cube_0'].evaluated_get(d).matrix_world.translation
        green=bpy.data.objects['cube_1'].evaluated_get(d).matrix_world.translation
        print('CONTACT',f,'head',head.translation[:],'cube_local',(head.inverted()@cube)[:],'shape_rotation',head.to_quaternion()[:],'parent_error',(expected-head.translation).length,'green',green[:],flush=True)

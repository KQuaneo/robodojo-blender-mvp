import bpy
from mathutils import Vector
for f in range(1,826):
    bpy.context.scene.frame_set(f)
    if f >= 775 and f % 2 == 0:
        deps=bpy.context.evaluated_depsgraph_get()
        h=bpy.data.objects['broom_collision'].evaluated_get(deps).matrix_world
        c=bpy.data.objects['cube_2'].evaluated_get(deps).matrix_world.translation
        print('BLUE', f, 'head', h.translation[:], 'cube', c[:], 'local', (h.inverted()@c)[:], flush=True)

"""Render only an independently qualified run, then verify replay consistency."""
import json
from pathlib import Path
import bpy
from mathutils import Vector

out = Path(bpy.data.filepath).parent
report = json.loads((out / 'collision_report.json').read_text())
assert report['qualified'], 'An unqualified run must not be rendered as accepted'
reference = json.loads((out / 'trace.json').read_text())[-1]['positions']
frames = out / 'video_frames'
frames.mkdir(exist_ok=True)
scene = bpy.context.scene
scene.render.engine = 'BLENDER_EEVEE'
scene.render.resolution_x = 960
scene.render.resolution_y = 540
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'PNG'
for frame in range(1, 1001):
    scene.frame_set(frame)
    if frame % 5 == 0:
        scene.render.filepath = str(frames / f'frame_{frame // 5:04d}.png')
        bpy.ops.render.render(write_still=True)
    if frame % 100 == 0:
        print('VIDEO', frame, flush=True)
depsgraph = bpy.context.evaluated_depsgraph_get()
errors = [(bpy.data.objects[f'cube_{i}'].evaluated_get(depsgraph).matrix_world.translation - Vector(p)).length for i, p in enumerate(reference)]
assert max(errors) < 1e-5, f'Render replay diverged: {errors}'
(out / 'video_replay.json').write_text(json.dumps({'verified': True, 'final_position_errors_m': errors, 'playback_speed': '4x', 'frames': 200}, indent=2))

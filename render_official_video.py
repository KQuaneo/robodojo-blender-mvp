"""Render a time-compressed preview while evaluating all 1,000 physics steps."""

from pathlib import Path
import sys

import bpy


argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
output_dir = Path(argv[0])
frames_dir = output_dir / "video_frames"
frames_dir.mkdir(parents=True, exist_ok=True)

scene = bpy.context.scene
scene.render.engine = "BLENDER_EEVEE"
scene.render.resolution_x = 960
scene.render.resolution_y = 540
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"

# Denser sampling during the three physical sweeps, lighter sampling elsewhere.
selected = set(range(1, 330, 6))
selected.update(range(330, 910, 3))
selected.update(range(910, 1001, 4))

shot_index = 0
for frame in range(1, 1001):
    scene.frame_set(frame)
    if frame not in selected:
        continue
    scene.render.filepath = str(frames_dir / f"shot_{shot_index:04d}.png")
    bpy.ops.render.render(write_still=True)
    shot_index += 1

print(f"VIDEO_FRAME_COUNT={shot_index}")


#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
blender --background --factory-startup --python-exit-code 1 --python "$PROJECT_DIR/test_collision_geometry.py"
blender --background --factory-startup --python-exit-code 1 --python "$PROJECT_DIR/test_official_checks.py"
blender --background --factory-startup --python-exit-code 1 --python "$PROJECT_DIR/plan_collision_scene.py"
blender --background "$PROJECT_DIR/output_collision/sweep_blocks_collision.blend" --python-exit-code 1 --python "$PROJECT_DIR/verify_collision_scene.py"
blender --background "$PROJECT_DIR/output_collision/sweep_blocks_collision.blend" --python-exit-code 1 --python "$PROJECT_DIR/render_collision_video.py"
ffmpeg -y -framerate 20 -i "$PROJECT_DIR/output_collision/video_frames/frame_%04d.png" -frames:v 200 -c:v libx264 -pix_fmt yuv420p -crf 20 -movflags +faststart "$PROJECT_DIR/output_collision/sweep_blocks_collision.mp4"

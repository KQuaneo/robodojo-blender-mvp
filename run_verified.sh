#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
blender --background --factory-startup --python-exit-code 1 --python "$PROJECT_DIR/test_official_checks.py"
blender --background --factory-startup --python-exit-code 1 --python "$PROJECT_DIR/build_verified_scene.py"
blender --background "$PROJECT_DIR/output_verified/sweep_blocks_verified.blend" --python-exit-code 1 --python "$PROJECT_DIR/verify_render.py" -- --video
ffmpeg -y -framerate 20 -i "$PROJECT_DIR/output_verified/video_frames/frame_%04d.png" -c:v libx264 -pix_fmt yuv420p -crf 20 -movflags +faststart "$PROJECT_DIR/output_verified/sweep_blocks_verified.mp4"

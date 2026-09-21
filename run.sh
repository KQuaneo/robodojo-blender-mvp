#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$PROJECT_DIR/output"

blender --background --factory-startup \
  --python "$PROJECT_DIR/build_scene.py" -- \
  --output-dir "$PROJECT_DIR/output"

test -s "$PROJECT_DIR/output/result.json"
test -s "$PROJECT_DIR/output/sweep_blocks_mvp.blend"
test -s "$PROJECT_DIR/output/success_frame.png"
test -s "$PROJECT_DIR/output/handoff_frame.png"
test -s "$PROJECT_DIR/output/sweep_frame.png"

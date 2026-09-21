#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
OUTPUT_DIR="$PROJECT_DIR/output_official"
mkdir -p "$OUTPUT_DIR"

blender --background --factory-startup \
  --python "$PROJECT_DIR/build_official_scene.py" -- \
  --output-dir "$OUTPUT_DIR"

test -s "$OUTPUT_DIR/result.json"
test -s "$OUTPUT_DIR/sweep_blocks_official_assets.blend"
test -s "$OUTPUT_DIR/success_frame.png"
test -s "$OUTPUT_DIR/handoff_frame.png"
test -s "$OUTPUT_DIR/sweep_frame.png"


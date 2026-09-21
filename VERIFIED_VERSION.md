# Sweep Blocks — repaired Blender adaptation

Run `bash run_verified.sh` from this directory. Requires local Blender and FFmpeg.
The old `output_official` files are retained and are not the repaired result.

## What changed

- Six-dimensional IK (position and orientation) replaces position-only IK.
- The tool is computed from the active hand's forward kinematics and a fixed
  grasp transform at every frame. Ownership switches from left to right at 190.
- Brush collision geometry is aligned with the official brush head, including
  its true local axes. The official visual no longer has an arbitrary offset.
- The official dustpan itself is a passive triangle-mesh collision body.
- Cubes remain dynamic, with no position/rotation animation or keyframes.
- Sweeps route through the dustpan mouth rather than through its walls.
- `sweep_feedback.json` records measured staging positions from an earlier
  physics run. The second push aims from those measured positions, correcting
  sliding observed in the initial open-loop run. This is an offline
  simulate/check/revise loop for the fixed instance, not an online policy.
- A separate process replays the saved scene and checks evaluated transforms.
- Final verification also ray-casts against the actual dustpan mesh below each
  cube, checks the cube-bottom gap, checks tool attachment, and verifies that
  all twelve arm joints return to their initial orientations.
- `official_checks.py` loads selected original RoboDojo predicate bodies without
  modifying them, via AST, and supplies Blender state through a small adapter.
  The original source file's SHA-256 is recorded in the report.
- One passing state and five failing controls test the success checks.

## Evidence

`output_verified/result.json` reports actual first success, final conditions,
IK error, and measured grasp-transform error. `trace.json` has all 1,000 states.
The video is 4x real-time: 25 simulation frames/second, one image every five
frames, encoded at 20 images/second. Nothing is skipped in the physics replay.

## Important boundaries

This is a Blender/Bullet adaptation, not an Isaac Sim run and not a VLA policy.
Robot joints, grip ownership and tool motion are kinematic/scripted. Robot
self-collision and robot/environment collision planning are not implemented.
The brush head uses a fitted rigid box, not deformable bristles; the handle
does not have a separate collision body. The pan is fixed to the table.

The checker executes the original support-circle branch of the task's OR group,
plus the other four predicates. A pass is sufficient for that group; a failure
is conservative because the alternative functional-bbox branch is not tested.
Layout access and quaternion-distance helpers are adapted to Blender. This is
not the full original RewardManager/Isaac environment. The support metadata is
the fixed instance's center `(0, -0.053, -0.015)` and radius `0.081` metres.

Physics time steps and solver parameters differ from Isaac Sim. The first
success frame is measured rather than chosen to match a social-media post.

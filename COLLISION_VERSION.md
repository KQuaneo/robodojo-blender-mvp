# Collision-aware revision

The previous `output_verified/` artifact is retained. This revision writes to
`output_collision/`; it is accepted only if `collision_report.json` has
`qualified: true`. A saved scene alone is not evidence of success.

## Changes

- Real triangle-mesh tests cover non-adjacent self-collision, both arms, tools,
  the table, and dynamic cubes during independent replay. Fully enclosed shapes
  are also tested. Only adjacent URDF mating links and the fixed base/table
  mounting interface are excluded; fingers are not exempt from tool checks.
- Gripper travel is calibrated against the tool mesh within the URDF's
  non-negative 0–44 mm range. The old negative travel crossed the fingers.
- Left and right hands use separate grasp positions. The pan is held from its
  handle end, keeping the left wrist out of the right arm's path.
- Cartesian contact strokes use collision-filtered numerical IK. Free motion
  uses checked joint-space segments, joint-order detours and RRT-Connect.
- Every searched path corner is retained as an animation key, including
  fractional frames. Copy-transform constraints maintain the fixed grasp
  between frames, not just at the integer physics samples.
- Each push is aligned to a brush collision face, avoiding sideways sliding
  from an oblique contact normal. Only collision-free brush orientations are
  accepted.
- Non-sweeping right-hand transfers keep the whole broom above 0.812 m,
  clearing the table and unswept cubes. Entry pushes are raised by 16 mm
  (8 mm for the third approach) to negotiate the actual dustpan mesh's raised
  lip, selected by physical trials.

## Verification

Latest accepted run: first task success at frame 986/1000; all 3,997 mesh
samples passed with zero robot collision events. All three cubes are supported
by the real pan mesh, joint positions satisfy URDF bounds, and tool attachment
translation error is below 0.00012 mm. See `output_collision/collision_report.json`.

`test_collision_geometry.py` checks separation, intersection, containment, and
the narrow collision-exclusion policy. `test_official_checks.py` tests a passing
state and five failing states against original predicate bodies.

`verify_collision_scene.py` independently replays the scene and samples robot
geometry every quarter frame (10 ms at 25 Hz). It checks task predicates, tool
attachment, absence of cube animation, and actual pan support by ray casting.
The `--quick` option checks fewer frames and can never set `qualified: true`.
The full verifier exits with an error if any acceptance condition fails.
`run_collision.sh` gates video generation on that result. Rendering replays
all 1,000 physical frames and checks final cube positions against the audited
run. The 200-frame, 20 fps preview plays the 40-second episode at 4× speed.

This remains a kinematic Blender simulation, not force-based grasping or an
Isaac Sim run. The collision audit is sampled, not a mathematical proof of
continuous collision freedom. The brush uses a fitted rigid collision box;
bristles are not deformable. Passing the task alone does not qualify a run.

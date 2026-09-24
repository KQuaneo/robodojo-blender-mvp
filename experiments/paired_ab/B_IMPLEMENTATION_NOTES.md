# B implementation checklist (not an implemented or evaluated method)

The experimental definition is frozen in `PROTOCOL.md`. This checklist does not
change it. No B results may be reported until the planner, executor, nominal
control and independent validators below are implemented and verified.

## Inputs and immutable references

- Reuse exact full offsets from the previous `manifest_v2.json` for seeds 0–19.
- Preserve calibrated tool-to-flange transforms, gripper apertures, ownership
  windows and task predicates; condition acquisition targets on tool poses.
- Read reference sweep from actual Blender animation, including fractional
  keyframes. `joint_trace.json` contains only integer frames and is insufficient
  to reconstruct all collision-avoiding turns or smooth transfers.
- Compare the broom's world pose against baseline from frame 240 onward. Keep
  right-arm sweep animation untouched and verify pose equivalence separately.
  Moving the pan must not translate or rotate sweep waypoints.
- Driver initialization offsets must not leak into the post-release park pose.

## Planning responsibilities

| Motion | Conditioning / required boundary |
|---|---|
| Home to broom pre-grasp | Shifted broom pose; continuous from actual home |
| Broom approach and grasp | Same calibrated local grasp; gate before attachment |
| Broom retreat and handoff approach | Carry geometry, both arms and environment |
| Handoff and retreat | Both flange transforms agree at ownership switch |
| Pan pre-grasp, grasp and hold | Shifted pan pose; calibrated local pan grasp |
| Transition to sweep start | Join exact baseline WORLD pose without teleport |
| Later pan release and home return | No discontinuity from shifted hold pose |

Every motion requires joint position limits, continuity, robot self/inter-arm,
robot/tool/environment and carried-tool/environment checks. The two arms must
be evaluated at the SAME time; planning one while pretending the other remains
stationary is not sufficient. Distinguish intended support/contact from
penetration explicitly; do not add ad hoc collision exemptions after failures.

Existing `plan_collision_scene.py::Planner` is a starting point, not a complete
B validator: `CollisionWorld` generates robot pairs, not general tool/environment
pairs; the planner's world omits cubes; some transfer checks assume a stationary
other arm. Its seed-cache and output paths must be redirected to trial-local
directories so no frozen evidence is overwritten. Do not persist learned warm
starts across diagnostic seeds.

## Failure and execution discipline

- Declare deterministic search budget, seeds, continuity bound and collision
  sampling density before B diagnostic execution. Record all candidates and
  rejection reasons. Search exhaustion is a stage-specific planning outcome.
- Never replace an unsuccessful plan with old baseline motion and call it B.
- Validate the complete proposed path before executing it; store proposed-path
  versus executed-path reports separately. Record overlapping stage failures
  without hiding an earlier collision under a later grasp failure.
- Compute acquisition gates from the NEW planned/executed flange transforms,
  not the old `output_smooth/joint_trace.json`.
- Execute accepted plans in fresh Blender processes with dynamic cubes. Apply
  initialization exactly once. Reuse original predicate and acceptance gates.
- Pass a nominal B control before dispatching the 20 diagnostic trials. An
  infrastructure exception is not a scientific planning failure or a retry.

## Reporting

Join per-seed B results to `a_results/paired.json`, retaining original offsets
and A effective offsets. The main acquisition-adaptation comparison is B versus
the previous all-random Frozen Script, not B versus nominal-tool A alone.
Do not generate or inspect held-out seeds 100–119 during this work.

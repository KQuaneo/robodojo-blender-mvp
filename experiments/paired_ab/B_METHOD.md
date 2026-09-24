# B local trajectory retargeting v1

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: run
- Origin Date: 2026-09-23
- Verification Status: UNVERIFIED
- Version Label: B_local_retarget_v1

This implements the frozen B scope with a bounded **local** planner, not a
complete global motion planner. It may reject feasible scenes. It must never
report such rejection as a successful plan or silently execute the reference.

## Candidate generation (before any B diagnostic results)

Use baseline animation, including all fractional joint keys, as the reference
path. Build a time grid containing those keys and every quarter frame (0.01 s).
Preserve original joint-angle branches using the integer reference trace.

Retarget the LEFT flange world translation along the whole acquisition path:

- Broom offset weight: smoothstep 0→1 on frames 1–35, 1 through 115,
  smoothstep 1→0 on 115–180, zero thereafter. This includes pre-grasp, grasp,
  pickup retreat and the connection to the fixed nominal handoff.
- Pan offset weight: zero through 225, smoothstep 0→1 on 225–250,
  1 through 900, smoothstep 1→0 on 900–1000. This includes pan pre-grasp,
  grasp, hold, release retreat and connection to home.
- Orientations and gripper schedules remain calibrated baseline references.
- Right-arm nominal handoff and transition targets are unchanged. Their full
  paths remain candidate paths subject to the SAME simultaneous two-arm safety
  checks in the perturbed scene. Retaining a validated candidate is not fallback
  after failure; an unsafe candidate is rejected.
- The right-arm and broom WORLD-coordinate sweep from frame 240 onward is
  frozen. Initial broom offset is removed from the unheld driver at frame 65
  so it cannot leak into the post-release park pose.

At each retargeted knot, numerical IK uses at most two deterministic seeds:
the reference configuration then the preceding accepted configuration. Each
seed gets the existing `single_ik` solver's 180-iteration budget. Require
position error ≤0.3 mm and rotation error ≤0.003 rad, then joint limits and
continuity. No cross-trial warm-start cache or random seeds. No sweep target
adaptation, obstacle-conditioned waypoint detours, RRT, or policy tuning.

## Validation / failure boundaries

Check both arms together, with their actual candidate poses and gripper
openings, at all knots and interval midpoints during acquisition (through frame
270), pan release/return (880–1000), and the complete held-pan interval. Joint
limits are the same URDF limits as baseline. Maximum joint change is 0.48 rad
per frame (12 rad/s), scaled to each sample interval; this is an explicit
continuity bound, not a torque/dynamics guarantee. A 0.00001-rad absolute
numerical tolerance covers float32 keyframe-time/rotation interpolation noise.

Check robot self/inter-arm and robot/tool/environment meshes throughout.
Before frame 240 also check carried/unheld broom against pan, cubes and table.
Table support permits at most 0.1 mm shallow visual-mesh overlap, recorded
numerically, because the nominal visual mesh rests 0.013 mm below the table.
All robot/tool intersections remain disallowed. Intended sweep tool/cube and
tool/pan contacts from frame 240 onward are not acquisition-planning obstacles;
the frozen sweep is evaluated by the unchanged task/robot-collision audit.
Pan is stationary on the table; validate its initialization and robot contacts,
not a fictitious transported-pan trajectory. Cube transforms during planning
are initial poses only; execution audits use simulated dynamic cube poses.

First rejected planning sample maps to pickup (≤115), handoff (≤225),
pan (≤270), or transition/continued safe connection (>270); failures on the
right-arm connection at 215–240 map to `transition_to_sweep_failure`.
Store frame, stage, IK residual/limit/continuity/collision evidence explicitly.
Do not execute a rejected proposal. Infrastructure exceptions remain separate.

Before diagnostic dispatch, a nominal control must pass planner validation and
fresh-process execution. Save accepted plans, joint traces and plan hashes.
Execution uses new flange trajectories for gates, the original calibrated
pan-to-flange transform, the original predicates and acceptance thresholds.
Fresh-process execution prevents planning-time physics cache contamination.

Command: `python3 experiments/paired_ab/run_b.py`. Sequential trials, separate
600-second planner and executor process limits, monitoring every 30 seconds,
no crash retries. Seeds 0–19 only; 100–119 remain unopened.

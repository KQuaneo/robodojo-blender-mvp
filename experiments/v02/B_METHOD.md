# B v0.2 local acquisition retargeting

This is a new method version paired with the repaired v0.2 reference, not a
rerun or replacement of the preserved v0.1 B control evidence.

Retain the whole-path local retargeting definition in
`../paired_ab/B_METHOD.md`: exact original dev offsets, calibrated grasp frames,
fixed ownership timing, fixed world-coordinate sweep, deterministic at-most-two
IK seeds per knot, 180 iterations per seed, ≤0.3 mm / 0.003 rad IK residuals,
joint limits, simultaneous robot/tool/environment validation, and no fallback.
The broom-offset and pan-offset weighting schedules are unchanged.

Version-specific corrections:

- Load the repaired v0.2 reference and its new joint trace, not v0.1.
- Explicitly set every newly generated left rotation key to LINEAR. Do not
  rewrite gripper actions. Leave right-arm and driver reference actions intact.
- Verify exact angular-rate maxima on all LINEAR quaternion segments, bound
  0.48 rad/frame plus 0.00001 numerical tolerance, before sampled geometry audit.
- Pass the offending arm into continuity failure classification: the right-arm
  215–240 connection maps to `transition_to_sweep_failure`.
- Execute in a fresh process and use the same key-aware collision/pose audit as
  v0.2 A. Record planning and executed-path evidence separately.
- Within a trial only, cache deterministic IK outputs by exact target-matrix
  bytes and seed-vector bytes. Recheck limits/continuity for every time knot.
  This avoids identical repeated solves during the stationary pan hold; it
  neither changes candidate results nor shares information across seeds.

The planner is deliberately bounded and local. It can reject scenes that a
larger/global search could solve. It never translates or retargets the sweep to
the perturbed pan/cubes. Nominal control must qualify before seeds 0–19; reserve
100–119 without inspecting or running them. Script and reference hashes freeze
when the B batch starts. No method changes after diagnostic dispatch.

Dispatch command: `python3 experiments/v02/run_b_parallel.py`. A fresh nominal
control runs alone first; afterwards at most two isolated trial processes may
run concurrently. Each trial plans before executing, with independent exact-input
IK caches and output directories. Process limits remain 600 seconds per phase.
This B-specific dispatcher preserves the already frozen A dispatcher unchanged.

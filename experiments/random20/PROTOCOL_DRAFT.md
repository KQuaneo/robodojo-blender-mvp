# Frozen-policy layout perturbation experiment — approved

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: plan
- Origin Date: 2026-09-23
- Verification Status: UNVERIFIED — no randomized experiments executed
- Version Label: random20_protocol_approved_v1
- User approval: confirmed with "ok" before implementation/execution.

## Frozen baseline

- Git tag: `v0.1-scripted-baseline`
- Commit: `58d0d5cf9010fc9291e30d9a9522a401362c7a5f`
- Scene: `output_smooth/sweep_blocks_smooth.blend`
- Existing evidence: success 986/1000; 3,997 sampled robot collision checks;
  zero detected robot collision events. These are historical results, not a
  fresh control rerun and not evidence of layout generalization.
- Preserve baseline code, scene, calibration, report and trajectories unchanged.

## Question and scope

Measure robustness of the frozen, open-loop trajectory to initial object
translations, not generalization of a scene-conditioned planner or an LLM.
No parameter tuning, replanning, automatic repair, seed selection by outcome,
or retrying a failed experiment is permitted.

## Approved design

- One fresh unperturbed control plus 20 trials with seeds 0..19.
- Randomize the three cubes, broom and dustpan independently in world X/Y,
  uniform [-0.020, +0.020] metres per axis; retain orientation.
- Keep the original support-consistent Z positions. Arbitrary Z perturbations
  would create floating or initially penetrating objects and confound this
  tabletop experiment. This is an XY perturbation test, not arbitrary XYZ.
- Check table bounds and initial intersections before execution. Log invalid
  initializations and retain them in the 20 attempted trials; never silently
  replace seeds. Report the valid-layout denominator separately.
- Write and hash all sampled layouts and the final protocol before running.
- Keep the same 1,000-step budget, physics parameters and actual joint keys,
  including fractional keys. Do not mutate cube transforms after initialization.

## Mandatory attachment safeguard

The frozen scene uses scheduled COPY_TRANSFORMS ownership switches. Merely
moving its initial broom then replaying would snap the broom into the hand.
The harness must represent the perturbed unheld pose independently and test
pickup alignment before enabling any attachment; missed pickup is a failure,
not permission to teleport. Apply analogous alignment checks at handoff and
the scripted pan hold. Log translation and rotation residuals independently.
Approved fixed alignment tolerances: 2 mm / 1 degree.
These are evaluator gates, not changes to the policy or proof of force closure.
Use the same gates for the fresh unperturbed control.

## Outcomes and audit

Record initialization validity, pickup/handoff/pan-hold outcome, executed step
count, termination reason, task conditions, cube poses, support and collision
events. Separate policy failure, invalid initialization, engine error and
timeout. Early terminated runs are not eligible for full-episode qualification;
do not label them as 3,997-sample collision-free episodes.

For completed runs, independently apply the existing adapted task predicates,
pan-support checks and 10-ms robot collision audit. Preserve hash identities of
policy artifacts and log harness version and all attachment-gate interventions.
The existing `max_grasp_gap_m` field is attachment translation residual, not
finger contact gap or physical grasp accuracy; use unambiguous new labels.

Report counts, all per-seed outcomes and failure categories, not a hypothetical
success rate. Twenty perturbations are a small descriptive test of this exact
distribution. If most fail pickup, do not infer sweep-strategy generalization.

## Execution and monitoring

- Entry command: `python3 experiments/random20/run_batch.py manifest_v2.json`.
- Working directory: project root.
- Planned outputs: `experiments/random20/results/` (layouts, per-trial JSON,
  process logs, summary CSV/Markdown and artifact hashes).
- Monitor process status and these logs every 30 seconds.
- Proposed timeout: 10 minutes per trial; notify before timeout termination.
- Crashes are retained and reported; no automatic rerun or silent repair.
- Do not publish new results externally without authorization.

## Preflight record (not experimental trials)

An initial control-only initialization check used transformed local bounding-box
corners and conservatively reported the broom below the table. This was not a
valid mesh-penetration test for a rotated non-box object. The harness now uses
transformed mesh vertices for exact world extrema. The failed preflight and
original manifest are retained; `manifest_v2.json` freezes the corrected harness
before formal trials. Random draws and policy artifacts are unchanged.

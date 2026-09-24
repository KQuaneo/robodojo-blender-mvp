# Paired diagnostic protocol v1 — frozen before A execution

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: run
- Origin Date: 2026-09-23
- Verification Status: UNVERIFIED
- Version Label: paired_ab_protocol_v1

## Scope and inputs

Frozen Script baseline: `v0.1-scripted-baseline` / `58d0d5c`.
Source draws: `experiments/random20/results/manifest_v2.json`, reused exactly,
not regenerated. Development / paired diagnostic seeds: 0–19. These are seen
seeds, not a test set. Reserve 100–119 for final held-out comparison; do not
generate their layouts, inspect, execute, or tune against them before methods
are frozen and final testing is explicitly authorized.

### A: cube-only random

Measure only robustness of the fixed sweep strategy to cube layout changes.
Copy each source seed's three cube XY offsets exactly. Set broom and pan
offsets to zero. Preserve baseline scene, all robot trajectories, attachment
timing, success predicate and collision/grasp audit. Run the existing
`experiments/random20/run_trial.py` unchanged, including all acquisition gates.
Collision or attachment failures remain failures even though tools are nominal.

### B: scene-conditioned acquisition, world-fixed sweep

Copy ALL source offsets exactly, including tools. Adapt broom/pan acquisition
and safe connections only. Preserve the broom's baseline WORLD-coordinate sweep
trajectory, orientation and timing from frame 240 onward; do not translate it
with the pan or adapt it to cubes. Preserve cube physics and success predicate.
Replan and validate the entire chain: current/home → pre-grasp → grasp → retreat
→ handoff approach → handoff → retreat → pan pre-grasp → pan grasp → transition
to frozen sweep start. Validate joint bounds, self/inter-arm collision,
tool/environment collision and continuity for every segment. Planning failure
is an outcome, never a silent fallback. Planned paths and executed paths must
be audited separately. B implementation parameters must be recorded and frozen
before its diagnostic batch. No B2/C0, sweep adaptation, or CaP in this protocol.

## Outcome taxonomy

1. `invalid_initialization`
2. `pickup_planning_failure`
3. `missed_pickup`
4. `handoff_planning_failure`
5. `missed_handoff`
6. `pan_planning_failure`
7. `missed_pan_hold`
8. `transition_to_sweep_failure`
9. `collision`
10. `task_failure`
11. `success`

Preserve raw results and all co-occurring failures. Primary outcome for an
executed run is invalid initialization first, otherwise an earlier audited
collision takes precedence over a later failed gate, otherwise gate failure,
otherwise collision, otherwise task/acceptance failure or success. Planning
failures are stage-specific and execution must not start with an invalid plan.
A introduces no planner. Map legacy `task_or_acceptance_failure` to
`task_failure` without altering raw evidence or acceptance criteria.

## Execution and monitoring

A command: `python3 experiments/paired_ab/run_a.py` from repository root.
One fresh nominal control must qualify before starting seeds. At most two
independent Blender processes, isolated output directories, 600-second hard
timeout per trial, process/log checks every 30 seconds. No retry or overwrite
of failed/crashed runs. Process errors halt new trial dispatch; already running
trials may finish. Preserve raw traces, geometry events, process logs, input
and script SHA-256 fingerprints. Report infrastructure failure separately from
the scientific taxonomy. No automatic GitHub upload.

## Comparisons and limits

Report same seed, original cube/tool offsets, A effective offsets, A outcome,
B outcome and previous Frozen Script outcome. A uses NOMINAL tools; B and the
previous all-random Frozen Script share perturbed tools. Thus A versus B is not
a clean causal estimate of acquisition adaptation. The primary paired contrast
for that effect is B versus previous all-random Frozen Script. A isolates cube
robustness. Counts are descriptive, not evidence of unseen generalization.
Blender kinematic acquisition and sampled geometry checks are not force closure,
continuous collision detection, or an official Isaac-runtime benchmark result.

Sequence: Frozen Script → A → B → method freeze → authorized held-out testing
→ future CaP. Do not revise definitions after observing outcomes.

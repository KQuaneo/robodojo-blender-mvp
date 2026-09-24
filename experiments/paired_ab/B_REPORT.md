# B control rejection and frozen-reference diagnostic

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: run
- Origin Date: 2026-09-23
- Verification Status: ANALYZED
- Version Label: B_control_rejection_v1

## Status

Command: `python3 experiments/paired_ab/run_b.py`.
The nominal B planning control was rejected after 35.10 seconds / 1,715
accepted validation samples. Blender exited normally (0); the batch deliberately
stopped (1). No control execution, no diagnostic seed execution or planning,
no retry, and no held-out layout generation occurred. There is NO B success rate.

The first rejected interval was frame 215.25 → 215.375, Right joint 2:
0.0603437 rad change over 0.125 frame, or 0.482750 rad/frame, exceeding the
predeclared 0.48 rad/frame bound plus numerical tolerance. This is a planner
continuity rejection, NOT evidence that this sample collided.

### Classification defect retained in raw evidence

The raw planner emitted `handoff_planning_failure`. Independent diagnosis
identifies the right-arm handoff-to-sweep connection, so the intended taxonomy
category is `transition_to_sweep_failure`. The generic continuity branch called
the stage classifier without arm information. Raw files have NOT been rewritten;
`b_results/control_diagnostic_outcome.json` records the correction separately.
Fix this classifier in a new implementation version before another B run.

## Read-only reference diagnostics (not experiment retries)

Two separate inspections loaded the unchanged baseline:

1. `inspect_reference_rates.py`: evaluated animation curves at all original
   joint keys, quarter frames and intervening midpoints. It found 34 sample
   intervals exceeding the new continuity bound. These are intervals, not 34
   distinct incidents.
2. `inspect_reference_pose.py`: disabled rigid-body physics and checked actual
   evaluated Blender flange/tool matrices at selected frames. FK reconstruction
   agreed to within 5.8e-7 in matrix entries, confirming that the observed
   excursions are not merely joint-angle extraction artifacts.

A particularly large excursion occurs around frame 544.4422, inside a frozen
free-transfer interval. Between frame 544.47110 and 544.5, Right joint 6 changes
about 0.782577 rad in 0.0289001 frame. The evaluated broom rotates at an average
of approximately 42,602 degrees/second over that short interval. This is a
kinematic animation artifact, not a measured physical actuator capability.

The original audits sampled quarter frames and therefore do not establish a
bound between those samples. Their reported transfer angular-speed maxima must
be described as maxima at that sampling grid, NOT continuous-time maxima.
The new diagnostic does not itself establish whether extra collision events
occur at those unsampled poses; that requires a separate geometry audit.

## Consequences and required decision

- A remains **7/20 success under the unchanged original acceptance checks**;
  13 task failures, all acquisition gates passed and zero collisions detected
  on the original sampling grid. Its data are not silently invalidated or rerun,
  but do not establish inter-sample safety/smoothness.
- `PROTOCOL.md`, the baseline artifacts, the B v1 method and raw control evidence
  remain unchanged. No threshold was relaxed and no fallback path was executed.
- Merely fixing the first transition would not remove the later frozen-sweep
  reference excursions under B v1's full-path continuity check.
- A repair to the frozen sweep would be a new baseline/version, not the same B
  experiment. Obtain an explicit decision before making that change. Prefer
  preserving v0.1 as historical evidence and creating a separately versioned
  corrected reference if smoothness/safety of the full trajectory is required.
- Alternatively, retaining the exact v0.1 reference requires explicit disclosure
  of its frozen-segment defect and a separately specified scope for B continuity
  checks. Do not silently narrow those checks to pass the control.

Evidence: `b_results/control/plan/plan.json`, control process record and log,
`reference_rate_diagnostic.json`, `reference_pose_diagnostic.json`, and manifest.
No new results were uploaded to GitHub.

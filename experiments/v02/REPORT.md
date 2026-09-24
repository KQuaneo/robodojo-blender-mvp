# v0.2 paired diagnostic results

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: run
- Origin Date: 2026-09-24
- Verification Status: ANALYZED
- Version Label: v02_paired_diagnostics

## Descriptive outcomes (20 seen seeds each)

- Frozen Script, all objects perturbed: `{"collision": 18, "missed_pickup": 2}`
- A, cubes perturbed and tools nominal: `{"task_failure": 13, "success": 7}`
- B, all objects perturbed with local acquisition planning: `{"task_failure": 17, "success": 2, "pan_planning_failure": 1}`

All three groups passed a fresh nominal control. Independent audit confirmed
exact source offsets, identical v0.2 reference hashes, unchanged frozen scripts,
consistent trace/event/sample counts, and no execution after planning rejection.
Accepted B plans retained the right-arm/driver reference actions and world-fixed
sweep poses. No held-out seed layout was generated or evaluated.

A seeds 10 and 11 originally timed out during audit. Their original logs and
process records remain preserved; the user explicitly authorized one new attempt
per seed with the same 600-second limit. The selected scientific results come
from `attempt_02/execution`, with provenance in each outcome record and
`results_a/resume_authorization.json`. Timeouts are infrastructure events, not
additional task failures or extra seeds in the denominator.

| Seed | v0.2 Frozen Script | A | B |
|---|---|---|---|
| 0 | collision | task_failure | task_failure |
| 1 | collision | task_failure | success |
| 2 | collision | task_failure | task_failure |
| 3 | collision | success | task_failure |
| 4 | collision | success | task_failure |
| 5 | collision | task_failure | task_failure |
| 6 | collision | task_failure | task_failure |
| 7 | collision | task_failure | task_failure |
| 8 | collision | success | success |
| 9 | collision | task_failure | task_failure |
| 10 | collision | success | task_failure |
| 11 | collision | task_failure | pan_planning_failure |
| 12 | collision | task_failure | task_failure |
| 13 | collision | success | task_failure |
| 14 | missed_pickup | task_failure | task_failure |
| 15 | collision | task_failure | task_failure |
| 16 | collision | success | task_failure |
| 17 | collision | task_failure | task_failure |
| 18 | collision | success | task_failure |
| 19 | missed_pickup | task_failure | task_failure |

## Interpretation boundaries

B versus the SAME-v0.2 Frozen Script is the paired acquisition-adaptation
comparison. A uses nominal tools, so B versus A alone is not an isolated causal
contrast. Planning failures are first-class outcomes, not missing trials; the
planner is local/bounded and does not establish global infeasibility.

These seeds were already seen during development. No confidence interval,
significance test, held-out performance or generalization claim is made.
The corrected reference and stronger audit differ from v0.1; historical data
remain preserved, not silently overwritten or pooled with v0.2.

Kinematic attachment is not force closure. Joint-rate bounds concern kinematic
interpolation, not physical actuator dynamics. Mesh collision checks remain
sampled, now including every rotation key and interval midpoints. The original
Blender predicate adapter is not the official Isaac runtime.

Evidence: per-group manifests, process logs/records, all raw plan/execution
files, `paired_results.json`, and `result_audit.json`.

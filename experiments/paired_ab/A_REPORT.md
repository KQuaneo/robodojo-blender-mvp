# A: cube-only paired diagnostic

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: run
- Origin Date: 2026-09-23
- Verification Status: ANALYZED
- Version Label: paired_A_v1

## Execution

Command: `python3 experiments/paired_ab/run_a.py` from repository root.
Status: completed; 1 qualified nominal control and 20 diagnostic seeds.
Control: first predicate success at frame 986,
3997 collision samples,
0 collision events.
Exact control final-position match with the previous fresh control: True.
Per-process durations ranged from 75.1
to 491.9 seconds. All 21 processes
exited normally; no hard timeout or automatic retry occurred.

Outcomes: `{"task_failure": 13, "success": 7}`.
All three acquisition gates passed in 20/20 diagnostic runs.
Counts of failed final predicate components: `{"all_in_support_circle": 13}`.
Counts describe this seen seed set only; no significance or generalization claim.

| Seed | Primary outcome | Collision samples with hits | First predicate success frame |
|---|---|---:|---:|
| 0 | task_failure | 0 | None |
| 1 | task_failure | 0 | None |
| 2 | task_failure | 0 | None |
| 3 | success | 0 | 986 |
| 4 | success | 0 | 986 |
| 5 | task_failure | 0 | None |
| 6 | task_failure | 0 | None |
| 7 | task_failure | 0 | None |
| 8 | success | 0 | 986 |
| 9 | task_failure | 0 | None |
| 10 | success | 0 | 986 |
| 11 | task_failure | 0 | None |
| 12 | task_failure | 0 | None |
| 13 | success | 0 | 986 |
| 14 | task_failure | 0 | None |
| 15 | task_failure | 0 | None |
| 16 | success | 0 | 986 |
| 17 | task_failure | 0 | None |
| 18 | success | 0 | 986 |
| 19 | task_failure | 0 | None |

`first_success_frame` is the first predicate-only success, not independently
qualified full-episode success. Collision counts are sampling events, not
distinct impacts. Raw results retain all co-occurring gate/collision evidence.

## Reproducibility and boundaries

Independent audit confirmed exact reuse of prior cube offsets, nominal tools,
unchanged baseline/executor/harness hashes, trace lengths and collision counts.
See `a_results/result_audit.json`, per-seed raw files and process records.
`a_results/paired.json` pairs A with prior Frozen Script; B is explicitly not run.
No retries were performed. No held-out layout was generated or evaluated.

B's experiment definition is frozen in `PROTOCOL.md`; at A completion its
planner had not been evaluated. Subsequent B work is recorded separately.
The B comparison must retain the world-fixed sweep
and explicitly report planning failures. A uses nominal tools, so B versus A
alone cannot isolate acquisition adaptation; compare B with the previous
all-random Frozen Script using the same full offsets.

Limitations: gated kinematic attachment rather than force closure; finite-time
sampled robot geometry audits, not continuous collision proof; Blender adapter,
not the official Isaac runtime. This is a diagnostic, not an unseen benchmark.
No results have been uploaded to GitHub in this experiment.

## Subsequent diagnostic caveat

B's keyframe-aware nominal-reference inspection subsequently confirmed short
pose excursions between the original quarter-frame samples. The 7/20 count
above remains the result under the frozen checks, not a claim of continuous
trajectory safety or smoothness. See `B_REPORT.md`; no A evidence was altered
and no baseline repair or experiment retry was performed.

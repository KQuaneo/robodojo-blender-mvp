# v0.2 construction status

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: run
- Origin Date: 2026-09-23
- Verification Status: UNVERIFIED
- Version Label: v02_build_attempt_01

The first build process exited 1 during its LINEAR-only continuity assertion.
It did not save a Blender scene or start a physical/diagnostic trial. The fresh
`baseline/` directory remains empty; raw log/process record are retained in
`logs/repair.log` and `logs/repair.process.json`. No automatic retry occurred.

Read-only inspection of the unchanged source found 62,832 rotation keys, all
BEZIER. The old builder set the insertion preference to LINEAR but actual key
data remained BEZIER. The revised v0.2 builder explicitly sets every new candidate
rotation key's interpolation, then analytically validates each quaternion chord.
It also requires an explicit fresh output directory to prevent overwriting.

The user subsequently authorized continuation. Attempt 02 completed in the fresh
`baseline_candidate_02/` directory. It removed 192 scalar keys at 8 stale times,
explicitly converted 62,640 rotation keys to LINEAR, and passed the analytic
continuity bound (maximum 0.337768 rad/frame over 15,648 segments).

The fresh nominal replay qualified at frame 986, with 8,013 key-aware collision
samples and zero hits. Final cube positions exactly match the earlier v0.1
nominal control. The five synthetic analytical-continuity regression tests pass.
Evidence is in `results_control/` and `baseline_candidate_02/repair_report.json`.
A/B diagnostic reruns are in progress; v0.1 and held-out seeds remain untouched.

A initially completed seeds 0–9 (3 success, 7 task_failure). Seeds 10 and 11
subsequently hit the 600-second hard-timeout policy; the monitor recorded about
726 seconds when it next ran and terminated them. The underlying scheduling or
sleep cause is not established. These are infrastructure events, not scientific
task failures. The user explicitly authorized one retry each in `attempt_02`
directories, retaining the original logs and time limit, then continuation of
previously unstarted seeds 12–19. See `results_a/resume_authorization.json`.

Update 2026-09-24: A completed all 20 diagnostic seeds: 7 success and 13
task_failure. Both explicitly authorized retries completed; the original timeout
records remain intact. The same-v0.2 all-object Frozen Script comparator also
completed: 18 collision and 2 missed_pickup, with a successful nominal control.
These collision labels use the frozen first-failure priority, not the older raw
grasp-gate-only labels. `summarize.py --partial` verified both groups' exact
inputs, frozen artifact hashes, and execution evidence. B nominal planning has
now started; no held-out seeds have been dispatched.

B progress: nominal planning and independent execution both passed. Seeds 0–3
completed (seed 1 success; seeds 0, 2, 3 task_failure). Seeds 4 and 5 passed
planning and entered execution. The batch remains active with at most two
isolated processes, 600 seconds per phase, and no automatic infrastructure
retries. These are partial results, not the final B outcome distribution.

Final update 2026-09-24: B completed all 20 diagnostic seeds: 2 success (1, 8),
17 task_failure, and 1 pan_planning_failure (11). The rejected plan was not
executed. All 19 accepted plans completed execution without detected collisions.
The batch exited normally with frozen hashes unchanged. The earlier progress
paragraph above is retained as history; see `REPORT.md` and `result_audit.json`
for the complete paired results. No held-out trials were dispatched.

Successful second build command:

```sh
python3 experiments/v02/monitored.py experiments/v02/logs/repair_attempt_02.log blender --background output_smooth/sweep_blocks_smooth.blend --python-exit-code 1 --python experiments/v02/repair_reference.py -- --output experiments/v02/baseline_candidate_02
```

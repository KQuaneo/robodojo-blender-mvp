# v0.2 corrected-reference development and paired diagnostics

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: run
- Origin Date: 2026-09-23
- Verification Status: UNVERIFIED
- Version Label: corrected_reference_v02

User authorized a NEW reference version after the v0.1 sub-frame excursion
diagnostic. Preserve all v0.1 artifacts and evidence unchanged. No GitHub upload
or held-out execution is authorized by this run.

## Reference repair (nominal scene only)

Remove stale non-quarter-frame RIGHT-arm rotation keys inside the six previously
smoothed free-transfer ranges. Retain all intended quarter-frame waypoints and
contact strokes. Delete by reverse indices, not stale references to a mutating
Blender key collection. Record every removed key. Reparameterize only the
right-arm 215–240 handoff-to-sweep connection, preserving its joint-space route,
endpoints and time window, allocating time by maximum joint displacement per
segment. This reduces a local speed spike without increasing the 0.48 rad/frame
continuity limit. Do not change cubes, grasp calibration, ownership timing,
contact sweep geometry, success predicates, physics parameters or episode budget.

Construction diagnostic: the first build's LINEAR-only validator rejected the
source interpolation assumption before any candidate scene was saved. A read-only
inspection found all 62,832 source rotation keys were BEZIER. Explicitly convert
the new candidate's actual rotation keys to LINEAR, not merely Blender's default
insertion preference. Integer waypoints remain fixed, but between-key paths change;
therefore require fresh physical replay and collision checks. Preserve the failed
build log and use a new candidate output directory; do not overwrite or auto-retry.

Validate nominal reference before freezing it. The new audit grid includes all
joint rotation keys, quarter frames and interval midpoints. In addition compute
the exact maximum angular rate of each LINEAR quaternion segment analytically;
reject rate >0.48 rad/frame +1e-5 numerical tolerance or degenerate interpolation.
Use the original independent task, grasp, mesh-collision and orientation criteria.
These remain sampled collision checks, not a continuous collision proof.

## Paired diagnostics after reference acceptance

Reuse EXACT prior seed 0–19 offsets from random20/manifest_v2.json. A: cubes only,
nominal tools, frozen v0.2 trajectory. B: all offsets, local whole-path acquisition
retargeting, world-fixed v0.2 sweep. Preserve the previous A/B causal definitions
and eleven-category taxonomy; correct B's arm-aware transition classification.
No cross-seed warm starts, fallback or tuning after diagnostic dispatch.

Freeze reference and script hashes before each batch. Fresh nominal controls
must qualify before any diagnostic seeds. All failures and raw traces retained.
Use at most two independent processes; monitor every 30 seconds, maximum 600
seconds per process, no crash retries. Explicitly distinguish scientific
rejection from infrastructure errors. Seeds 100–119 remain reserved and unopened.

Historical v0.1 Frozen Script is not a same-policy causal comparator for v0.2 B.
Any comparison to it must disclose BOTH reference repair and acquisition
adaptation. A versus B still differs in tool initialization, so is diagnostic,
not an isolated treatment-effect estimate. Do not claim unseen generalization.

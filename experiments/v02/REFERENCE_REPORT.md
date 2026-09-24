# Corrected v0.2 reference

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Mode: run
- Origin Date: 2026-09-23
- Verification Status: ANALYZED
- Version Label: v02_reference

Reference: `baseline_candidate_02/reference.blend`. The directory name retains
construction provenance; this candidate passed nominal validation before A.
Its scene and metadata SHA-256 values are frozen in each batch manifest.

Manifest schema note: `baseline_commit`, `harness`, seed-generator and original
approval fields were inherited from the source random20 manifest for provenance.
They identify the v0.1 source history, NOT a git commit of this new reference.
For v0.2 identity use `reference_dir` and its entries in `artifacts`; new executor
and motion-audit hashes are also in `artifacts`. The `audit` field and each raw
result's explicit `audit_grid_frames` describe the new grid, rather than the
inherited nominal `audit_dt_s` value. No frozen manifest was rewritten to hide
this inherited metadata.

## Changes and checks

- Removed 192 scalar rotation keys encoding 8 stale sub-frame waypoints inside
  previously smoothed free transfers. Preserved intended quarter-frame keys.
- Reparameterized the right-arm 215–240 route within the same window, preserving
  joint-space route nodes and endpoints.
- Explicitly converted actual rotation keys to LINEAR. The first construction
  attempt discovered the old 62,832 source keys were BEZIER despite the default
  preference being set to LINEAR. That failed attempt and log remain preserved.
- Analytically checked 15,648 quaternion interpolation segments: maximum
  0.337767884 rad/frame, below the unchanged 0.48 continuity bound.
- Five synthetic regression tests passed, including a known analytical maximum,
  zero motion, excessive rate, antipodal degeneracy and BEZIER rejection.
- Fresh nominal physical replay: qualified, predicate success at frame 986,
  all acquisition gates passed, 8,013 mesh-collision samples, zero hits.
- Bristles-down transfer sampled maxima: tilt 0.02798 degrees, angular speed
  91.76984 degrees/second. Not continuous tool-angular-speed bounds.
- Nominal final cube coordinates exactly match the earlier v0.1 control.

The analytical bound is for individual kinematic joints, not torque/actuator
feasibility. Geometry remains sampled, although every rotation key and midpoint
is now included. This is not a formal continuous-collision or force-closure proof.

No contact target geometry, object initialization, physics parameters, calibration,
ownership timing, predicates or 1,000-frame episode budget were changed. The
interpolated paths and transition timing DID change; this is explicitly a new
reference, not a retroactive repair to v0.1 results.

## Comparison discipline

Re-run A and B with unchanged source draws. Also run a v0.2 all-object-perturbed
Frozen Script comparator so B's acquisition effect is not confounded with the
reference repair. A uses nominal tools and isolates cube-layout robustness.
All seed 0–19 results remain development/paired diagnostics. Held-out 100–119
have not been generated or inspected.

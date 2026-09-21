# Bristles-down transfer revision

Run `bash run_smooth.sh` from this directory. Input is the preserved accepted
`output_collision/sweep_blocks_collision.blend`; output is in `output_smooth/`.

The six free transfers between sweeping strokes and to the final parking
approach now interpolate tool position with smoothstep timing and tool
orientation along the shortest quaternion arc. Both endpoints have the
bristles facing down, so the transfer changes heading without flipping the
brush. Sequential IK keeps the same continuous joint branch. Old fractional
joint-space path corners are removed; new poses are keyed every quarter frame.
No unconstrained joint-space fallback is allowed if a transfer fails.

The original handoff and contact strokes are retained. Feasibility screening
checks full robot meshes, carried tool/table/pan contact, clearance and joint
continuity. The independent full verifier checks 3,997 time samples and gates
acceptance on at most 2 degrees of tilt and 120 degrees/second angular speed
during these transfers, in addition to all existing collision/task checks.

Accepted result: task success at frame 986/1000, zero robot collision events
in 3,997 samples, maximum measured transfer tilt 0.04 degrees and angular
speed 91.43 degrees/second. All three dynamic, unkeyframed cubes are supported
by the pan. Render replay is separately checked against final cube positions.

This remains scripted kinematic grasping and sampled collision validation,
not force-based grasping or a continuous-time collision proof. The 10-second
preview is 4× playback speed, so visible speed is faster than simulation time.

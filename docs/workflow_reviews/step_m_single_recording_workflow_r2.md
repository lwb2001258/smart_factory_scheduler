# Single-recording workflow review R2 — evidence and performance validity

The recording result already contains launcher-measured
`webots_wall_seconds`/`sim_to_wall_ratio` plus supervisor and joint-planning
wall-time distributions. Reading those values from the same run directly
matches the requested acceptance basis and preserves comparability because
scenario, seed, duration, diagnostic recording, and artifact selection remain
fixed. Robot efficiency, collision safety, visual evidence, and simulator
performance now share one run, so there is no cross-run nondeterminism.

Removing the duplicate run halves validation time and cannot affect robot
control behavior. The workflow records `performance_source=recording_run` to
make the provenance explicit. Focused tests and compilation pass.

Verdict: PASS

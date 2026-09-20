# Step O rollback review R2 — scope and regression safety

## Independent review

- Removed only Step O runtime constants, acquisition points, writer/watchdog gates, idempotent recovery gates, and matching tests.
- Restored the pre-Step-O immediate recovery-lease clearing behavior; the accepted fallback waypoint-offset fix remains intact.
- Did not alter collision thresholds, local/supervisor safety shields, scheduler policy, task goals, fallback offsets, recording workflow, or metrics.
- The prior Step L recording is valid behavior evidence for the restored runtime: 10 tasks, 1.00/min, 108.0 s longest plateau, 0 safety violations, and 1.598× recording-run simulation ratio.
- A fresh full test run is required after rollback. The next candidate must avoid a fixed global timing delay and instead constrain route geometry/direction at the joint handoff.

Verdict: PASS

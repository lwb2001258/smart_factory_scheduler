# Step O scheme review R2 — liveness, safety, and performance

## Failure-mode review

- **Stale commitment:** identity includes the current business goal and active route source; either changing invalidates the gate. The deadline is absolute rather than renewed by repeated conflict scans.
- **No-motion recovery:** the 5.0 s gate expires before the existing 8.0 s physical hard-stall deadline. The watchdog can then select another validated escape; it is not permanently suppressed.
- **Collision during commitment:** speed scaling, local peer braking, the dynamic safety shield, and emergency stop commands are not route writes and remain authoritative.
- **Fleet freeze:** only the committed robot is omitted from the joint writer set. Its route is represented by the existing per-slot occupancy band; peers continue moving/planning.
- **Same-source churn:** the public recovery entry points must return success without dispatch while their commitment is active. The generation and route metrics must not increase.
- **Arrival:** physical arrival must release the gate before normal pickup/delivery/home state transition so a completed maneuver cannot delay useful work.

## Performance review

- The proposal adds constant-time field checks to existing dispatch/recovery paths and no hot-loop search, file I/O, or IPC.
- Compared with the rejected 12 s design, the maximum exclusion interval falls by 58.3% and remains shorter than the liveness escalation threshold.
- Acceptance must come from a 600 s recorded Webots run. Required evidence is fewer conflict-source chains/rapid overrides without regression in completed tasks, longest completion plateau, minimum separation, violations, or recording-run simulation ratio.
- If task throughput does not materially improve or oscillatory chains persist, the change is rejected before any separate performance run; performance is read from the same recording as requested.

Verdict: PASS

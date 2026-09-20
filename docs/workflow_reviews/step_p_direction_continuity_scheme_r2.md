# Step P scheme review R2 — liveness and planner performance

## Failure-mode review

- **Narrow aisle / only reverse is possible:** immediate-feasibility probing finds no non-reverse move and leaves the original move set untouched.
- **Locally valid direction later dead-ends:** `_plan_order` retries the affected robot without continuity using only the remaining wall-clock budget.
- **Goal legitimately reverses:** no preferred vector is supplied when the active transaction's goal identity differs from the current business goal.
- **Stale telemetry:** controller target is used only with a matching path version and meaningful displacement; otherwise the current remaining waypoint is selected.
- **Collision pressure:** vertex, edge, footprint, endpoint, and final validation checks execute exactly as before. Continuity never bypasses a conflict.
- **Stopping instead of reversing:** when a safe non-reverse moving step exists, wait is excluded with the reverse move for the first two slots, supporting stable motion rather than replacing backtracking with parking.

## Performance review

- Normal cost is a few dot products over the fixed five-move set. No new Webots message, file I/O, grid copy, or global search is introduced.
- A second low-level search occurs only after a direction-constrained search actually fails and shares the same absolute planner deadline.
- Preferred directions must be part of the candidate-cache key so a cached plan cannot silently reuse a route generated under a different continuity vector.
- Acceptance requires a new 600 s Webots recording with materially improved robot efficiency. Physical reversal count and route-source chains are diagnostic evidence; task throughput and completion plateau remain the retain/rollback decision.

Verdict: PASS

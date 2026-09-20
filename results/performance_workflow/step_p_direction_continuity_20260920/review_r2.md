# Step P implementation code review R2 — liveness, integration, and cost

## Independent checks

- New API parameters are optional at every layer; legacy callers and all safety tiers retain their prior behavior when no vector is supplied.
- The retry reuses the already-built static heuristic and reservations and cannot run beyond `max_seconds`; it does not create an unbounded second planning budget.
- A locally blocked forward/side step leaves the original five-move set intact immediately. A later dead-end triggers the explicit unconstrained retry.
- The filter selects a moving step instead of replacing a reversal with a wait, directly addressing the user's stop/start and back-and-forth observation.
- No Webots message frequency, controller speed law, scheduler, task assignment, route writer, or collision threshold changed.
- Candidate safety remains checked after planning, and dynamic/local safety shields remain fully authoritative during execution.

## Verification reviewed

- Direction preference test: safe non-reversing first cell selected even when the task goal lies behind.
- Escape test: reverse selected when all continuing/wait cells are blocked at the next slot.
- Cache regression test: opposite direction vectors do not hit the same cached candidate.
- Goal-identity and controller-target tests cover supervisor vector derivation.
- Focused planner/coordinator/supervisor suite: 165 passed plus 4 subtests.
- Full suite: 420 passed plus 4 subtests; Python compilation passed.

Verdict: PASS

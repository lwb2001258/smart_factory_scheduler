# Step O implementation code review R1 — ownership and lifecycle

## Scope reviewed

- Recovery commitment acquisition in `_dispatch_plan`.
- Route-writer preflight in `_route_write_allowed` and the joint transaction preflight.
- Idempotent gates in priority-yield, escape, reverse, and joint-stall recovery entry points.
- Arrival, priority-state, goal-change, expiry, and joint-activation cleanup.

## Findings

1. Acquisition occurs only after `navigate` send succeeds, so a failed IPC send cannot create a phantom route lock.
2. The identity requires both a recognized conflict source and the same business goal. A task/charging/home goal change invalidates the commitment without waiting for the deadline.
3. The absolute deadline is not extended by conflict scans because active entry calls return before planning or dispatch. Route generation and override metrics therefore remain stable during the window.
4. The joint transaction calls `_route_write_allowed` before retiring an activated predecessor, so a blocked successor cannot destroy the currently executable route.
5. Review found that the watchdog's second classification loop could still label a committed maneuver emergency/stalled even though its first hard-stall loop skipped it. The implementation was corrected to skip committed robots in both loops; collision scan and the safety shield still execute before that skip.
6. Physical arrival clears the commitment before normal business successor planning. Priority-yield cleanup only clears a matching priority maneuver and cannot erase a newer joint-stall commitment.

Verdict: PASS

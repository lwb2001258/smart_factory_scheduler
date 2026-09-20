# Step P scheme review R1 — geometric continuity at joint handoff

## Recording evidence

- Fine-grained Webots animation poses (1 s windows, displacement >= 0.04 m, direction cosine < -0.5) show 25 physical direction reversals in the Step L run.
- Step O reduced those reversals to 12, proving that route handoff timing affects the visible back-and-forth, but it also reduced completed tasks from 10 to 9 and increased rapid route rewrites from 77 to 98. A fixed writer delay is therefore rejected.
- Step L contains 65 rapid `joint_grid_transaction -> joint_grid_transaction` rewrites, so protecting only explicit reverse/stall routes cannot solve the dominant handoff path.

## Proposed rule

1. For a rolling joint refresh, derive a preferred departure vector from the controller's current target (falling back to the remaining supervisor route).
2. Do not carry continuity across a business-goal change; pickup-to-delivery, charging, home, and task replacement remain free to turn around.
3. For only the first two space-time slots of the successor plan, prefer a moving cardinal step whose dot product with that vector is non-negative.
4. The filter is conditional: it applies only when at least one such step is immediately obstacle-, reservation-, and edge-conflict-free. If none exists, the normal wait/reverse moves remain available.
5. If the locally continuous search later proves infeasible, retry that robot once without the continuity direction inside the same bounded planner call.

## Review findings

- This acts on the geometric cause of visible reversal and applies to ordinary joint-to-joint refreshes as well as recovery-to-joint handoff.
- It does not add a time hold, route-writer lease, speed reduction, or whole-fleet freeze.
- Safety validation, separation envelopes, priority ordering, business goals, and the final independent joint-plan validator remain unchanged.
- Two slots are enough to carry the robot out of the previous conflict cell while remaining bounded to the existing rolling horizon.

Verdict: PASS

# Step G avoidance scheme review R1

Verdict: PASS

- The proposal addresses the recorded cause, not only the symptom: symmetric caution and a 0.5 s release window created repeated clear/caution transitions.
- Deterministic winner/yielder selection reuses the existing business-priority and terminal-egress rules; it does not introduce a competing priority system.
- Scope is bounded to CAUTION speed shaping. BRAKE and EMERGENCY remain symmetric zero-speed fail-closed actions.
- No route writer, reservation, task state, or controller protocol identity is changed.
- Acceptance requires real Webots recording plus a separate no-recording performance run.


# Step G avoidance scheme review R2

Verdict: PASS

- A 0.35/0.75 profile creates relative progress while retaining braking margin; the winner is not allowed to run unrestricted through a predicted conflict.
- A 1.5 s hold is long enough to suppress the recorded ~0.64 s oscillation and remains bounded, so stale restrictions cannot persist indefinitely.
- Multi-peer composition remains conservative because every pair can only lower each robot's desired scale.
- Exempt/low-battery/emergency robots are already protected by `_priority_yield_select`; hard risk continues to override any held caution immediately.
- The change must be rejected or rolled back if recording shows lower clearance, more unplanned stops, worse group-stop duration, or simulation ratio below 1.0.


# Step N implementation code review R1 — safety calculation

The helper scales only the selected yielder's measured velocity, leaves the
winner unchanged, and calls the same `assess_motion_risk` function with the
same 10 s horizon, 0.50 m collision distance, 0.85 m prediction clearance,
reaction latency, and braking deceleration as the primary shield. It returns
one-sided scales only for `RiskLevel.CLEAR`; missing roles or every other
result returns symmetric 0.6. The caller invokes this helper only for raw
CAUTION, so BRAKE and EMERGENCY remain symmetric zero-scale decisions.

Tests prove a crossing pair accepts a validated one-sided action, an
unproven candidate stays symmetric, and a finite-TTC BRAKE stops both robots.

Verdict: PASS

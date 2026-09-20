# Step H joint-arrival adjustment review R1

Verdict: PASS

- G3 is rejected on three hard gates; H restores the better G2 avoidance profile before making the arrival fix.
- G2's two stops occurred with no nearby peer or shield action, at about 0.24 m from the first joint waypoint, identifying a local convergence issue rather than an avoidance wait.
- Raising the joint-only threshold from 0.22 m to 0.24 m remains strictly below the 0.25 m grid edge and therefore cannot consume an untouched adjacent cell at dispatch.
- Business-goal threshold, hard distance safety, grants, reservations, and planner clearance are unchanged.


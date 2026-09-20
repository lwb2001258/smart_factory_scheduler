# Step L scheme review R1 — fallback time-window correctness

Evidence: the 600-second Webots recording completed 11 tasks but dispatched
972 routes, issued 229 replan requests, and repeatedly reported
`emergency:reservation_deadline_expired`. Earlier activation telemetry proves
fallback epochs install the intended waypoint-zero and offset count, while
the fallback builder assigns zero to every offset. Consequently every point
shares the same `activate_at + 4.75 s` latest-exit deadline, independent of
path length, producing synchronized fleet stops.

Proposed change: preserve the existing current-position anchor at offset zero,
give the first future grid cell offset zero to match the regular joint planner,
and increase every later point by one `joint_time_slot_s`. Require finite,
nondecreasing offsets with exactly one offset per point. Do not change speed,
clearance, collision prediction, grants, or emergency braking.

Boundary review: one-point paths receive `[0.0]`; anchored paths receive
`[0.0, 0.0, 4.75, ...]`; a near-start first point is conservatively treated as
an anchor because the controller consumes it within its 0.22 m threshold.

Verdict: PASS

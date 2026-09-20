# Step C route-less recovery review R1

Recording correlation found Robot 2 in an active delivery task with no
Supervisor/controller target and controller stop reason
`business_goal_or_inactive`. The existing watchdog waits 3 s after observing
that exact route-less state. This change reduces only that confirmation to
0.5 s. The robot must still be in an active navigation state, have no remaining
waypoint, have no pending transaction plan, and not be in priority-yield. The
same joint planner and atomic transaction produce the replacement route.

Verdict: PASS

# Step L implementation code review R1 — control semantics and safety

Reviewed files: fallback construction in `factory_supervisor.py`, the new
motion-stability measurements in the long-horizon workflow, and focused unit
tests.

The fallback offsets have the same execution convention as normal joint
paths: current/start anchor offset 0, first traversable cell offset 0, and one
additional `joint_time_slot_s` for each later point. Offset count always
matches waypoint count and values are finite/nondecreasing. The controller's
deadline check, supervisor segment-grant validation, swept occupancy checks,
dynamic shield, and minimum-distance gate are untouched. The change cannot
authorize motion into a segment that the existing validators reject.

The focused tests exercise exact offsets and workflow rejection thresholds;
142 tests passed.

Verdict: PASS

# Step N scheme review R2 — determinism, continuity, and rollback

This is not a replay of the rejected fixed asymmetric tuning experiments.
Those profiles were applied without a per-pair post-scaling safety proof. The
new branch is conditional: an independently recomputed CPA must satisfy the
existing CLEAR threshold, and every failed/ambiguous role selection falls
back to the current symmetric command.

Winner/yielder selection reuses terminal-egress precedence, task priority,
starvation aging, and robot-ID tie breaking, so a pair has deterministic
right-of-way. The change adds one constant-time CPA calculation only for raw
CAUTION pairs; it adds no planning, route write, or controller state. Tests
must cover validated one-sided acceptance, unsafe-candidate symmetric
fallback, and unchanged BRAKE behavior.

Acceptance uses one 600-second recording. Keep only if tasks are no worse than
the 11-task long baseline, motion/zero transitions and active-zero ratio pass,
shield changes fall below 30 per robot-minute, minimum separation stays at
least 0.50 m with zero violations, and recorded sim/wall ratio stays at least
1.0. Otherwise roll back this branch.

Verdict: PASS

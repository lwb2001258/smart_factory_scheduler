# Step L scheme review R2 — concurrency, liveness, and performance

The proposal extends only the latest-exit deadline of later fallback cells.
It does not grant entry into an occupied swept segment: the supervisor still
validates each current/next segment against fresh peer poses, reverse edges,
terminal ownership, and resource reservations; the controller retains its
reservation-deadline emergency stop; and the dynamic shield remains active.

The monotonic schedule removes the fleet-wide common expiry edge without
adding planner work or control-loop messages. Longer route-writer leases do
not block a same-owner joint successor, and the existing rolling refresh is
still capped at six seconds. Unit tests must cover anchored and one-point
paths, offset length, monotonicity, and exact slot progression. Acceptance
requires the full test suite plus a new 600-second recording and separate
600-second no-recording run, with zero distance violations and no worse
Webots sim/wall ratio. Roll back if throughput, stop/start continuity, or
conflict recovery regresses.

Verdict: PASS

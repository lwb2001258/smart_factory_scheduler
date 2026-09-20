# Step K workflow review R2 — liveness, safety, and simulator performance

Scope: independent second review of whether the gate can still accept the
reported fleet-wide stalls or trade robot efficiency for unsafe motion or poor
Webots performance.

Findings: the review added an absolute longest completion-plateau bound so a
large middle-of-run freeze cannot be hidden by one late completion. The gate
now requires at least 1.0 completed task/minute, bounded longest and final
plateaus, participation from at least half the fleet, no more than two
reservation-deadline stop episodes, zero pair-distance violations, at least
0.50 m minimum separation, and sim/wall ratio at least 1.0. Recording and
no-recording runs remain separate, preventing export cost from contaminating
the simulator performance result. Tests cover every new rejection path.

Verdict: PASS

# Step N scheme review R1 — collision-safety proof obligation

Recording evidence after the fallback fix shows 2,301 CAUTION entries and
2,202 CLEAR restorations in 600 seconds, while hard BRAKE/EMERGENCY entries
are only 124. The current pair loop applies every CAUTION to both robots,
slowing relative motion until CPA becomes clear, then accelerates both and
recreates the same risk. This is the observed speed-limit cycle.

Proposal: for CAUTION only, choose the existing deterministic winner/yielder,
scale only the yielder's measured velocity to 0.6, and recompute CPA/TTC with
the unchanged 0.85 m prediction clearance. Use the one-sided action only when
that candidate is CLEAR; otherwise retain the existing symmetric 0.6 action.
BRAKE, EMERGENCY, stale-pose stops, hard collision distance, and all route and
grant validators are unchanged.

The raw CAUTION trajectory already has no finite collision TTC within the
horizon. Candidate validation must still return CLEAR, so the optimization
cannot accept a lower predicted clearance than the current contract.

Verdict: PASS

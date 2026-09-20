# Step O scheme review R1 — anti-oscillation semantics

## Evidence reviewed

- The 600 s Step L recording contains 77 rapid route overrides. The conflict-recovery chains include `_priority_yield_direct -> _command_reverse` in 1.0–1.5 s, `_command_reverse -> joint_grid_transaction` in about 3.6–4.6 s, and a new `_command_reverse` less than 0.5 s after a joint route. These are route-authority changes, not task-goal changes.
- The recovery execution lease scaffolding is currently inactive: no successful recovery dispatch acquires it and `_simulate_movement` clears it on every loop.
- The historical 12 s lease is not reusable: its 600 s validation completed only 8 tasks and recorded ten 12 s unplanned stops.

## Proposed bounded rule

1. A successfully dispatched conflict maneuver (`_priority_yield_direct`, `_priority_yield_resume`, `_command_reverse`, or `_joint_stall_recovery`) owns a **5.0 s commitment window** tied to the same business goal and active plan source.
2. During that window, repeated conflict-recovery entry calls are idempotent and no different route writer, including a new joint transaction, may replace the maneuver.
3. Dynamic safety speed/brake commands remain active because the commitment gates only navigation-plan writes.
4. The commitment is cleared on physical maneuver/business-goal arrival, on a changed business goal, or by an already validated joint activation after expiry. The hard upper bound is 5.0 s, below the 8.0 s hard-stall escape deadline.
5. Other robots are not frozen. Existing time-indexed recovery cells keep the committed robot out of a new joint write set while allowing peers to be planned around its moving occupancy.

## Review findings

- The rule addresses the observed source-transition chains directly instead of changing scheduler priority, task selection, collision distance, or robot speed.
- Five seconds covers every observed 3.6–4.6 s premature recovery-to-joint handoff, while being materially shorter than the rejected 12 s lease.
- An idempotent entry gate is required in addition to writer priority; otherwise the same high-priority recovery source could still select a different escape target.
- Safety intervention remains independent and therefore cannot be suppressed by the commitment.

Verdict: PASS

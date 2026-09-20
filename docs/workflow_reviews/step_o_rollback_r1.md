# Step O rollback review R1 — recorded behavior gate

## Recorded result

- 600 s recording completed, but only 9 tasks finished versus 10 in Step L.
- Throughput fell from 1.00/min to 0.90/min and the longest completion plateau rose from 108.0 s to 186.8 s.
- Rapid route overrides increased from 77 to 98. Conflict-source routes fell from 29 to 19, but joint-to-joint rapid rewrites increased from 65 to 90.
- Reservation-deadline stop episodes increased from 6 to 10. Safety remained valid (0 violations, 0.550 m minimum distance), so the rejection is specifically an efficiency/liveness decision.

## Rollback decision

The fixed 5 s commitment delays a conflict overwrite but changes the timing of the fleet-wide joint replanner. The recording shows that the delayed handoff becomes additional joint route churn rather than stable forward progress. Retaining the change would violate the user's requirement that only a substantial robot-efficiency improvement may remain.

Rollback restores the exact recovery dispatch/watchdog/writer behavior validated by the prior 600 s Step L recording. Scheme and failed-run artifacts remain as audit evidence; runtime code and tests are reverted.

Verdict: PASS

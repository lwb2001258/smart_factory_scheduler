# Webots robot-efficiency iteration report

## Scope and fixed validation protocol

- Base branch: `experiment/speed24` at `75c3fc3`.
- Working branch: `experiment/speed24-efficiency-webots-70`.
- Scenario C, FCFS, seed 42, eight robots, 1800 simulated seconds.
- Webots batch/no-rendering/fast mode with full HTML5 animation export.
- Every code candidate was reviewed twice, exercised by the full automated
  suite twice, and then judged from a complete 30-minute recording.
- Acceptance floor: at least 70 tasks/30 min and Webots average speed factor
  above 1.2x. A later candidate also had to improve on the retained recording,
  including motion quality, rather than merely clear the absolute floor.
- Failed candidates were reverted; their recordings were retained as evidence.

## Recording results

All distance and motion-quality values below come from the HTML5 recording
frames. `min pair` is the minimum distance reconstructed from those frames.

| Run | Tasks | Webots x | Distance m | Stationary s | Unplanned s | Spin s | Reversals | Escapes | Replans | Longest plateau s | Min pair m | Decision |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| Base `75c3fc3` | 73 | 3.3644 | 1688.6 | 1002 | 573 | 46 | 423 | 243 | 290 | 131.536 | 0.524 | Baseline |
| Step 1: calm refresh 3.0 s | 80 | 3.2027 | 1719.7 | 871 | 581 | 45 | 364 | 212 | 222 | 108.256 | 0.527 | Retained |
| Step 2: calm refresh 3.5 s | 69 | 3.1289 | - | 1142 | - | - | - | - | - | - | - | Reverted |
| Step 3: reset hard-stall clock after escape | 72 | 3.2139 | - | 1108 | - | - | - | - | - | - | - | Reverted |
| Step 4: apply speed scale once | **84** | **3.2575** | **1838.8** | **624** | **373** | **15** | **369** | **115** | **127** | **94.112** | **0.500** | **Retained / final** |
| Step 5: calm refresh 3.25 s | 77 | 3.2541 | 1806.3 | 795 | 364 | 9 | 371 | 127 | 148 | 95.968 | 0.534 | Reverted |
| Step 6: initialize hard-stall anchor | 77 | 3.5618 | 1797.3 | 678 | 346 | 34 | 411 | 151 | 178 | 118.976 | 0.499 | Reverted |
| Step 7: direct-yield lease dedup | 80 | 2.7590 | 1749.2 | 907 | 487 | 24 | 399 | 209 | 230 | 94.112 | 0.529 | Reverted |
| Step 8: motor headroom 1.12 | 81 | 2.6536 | 1825.9 | 766 | 431 | 40 | 444 | 197 | 209 | 99.984 | 0.502 | Reverted |
| Step 9: bounded side-wait dedup | 87 | 2.7750 | 1768.9 | 696 | 471 | 36 | 379 | 258 | 275 | 94.112 | 0.479 | Reverted |
| Step 10: hard-escape window 5 s | 75 | 2.8650 | 1789.4 | 881 | 484 | 27 | 411 | 165 | 179 | 118.000 | 0.483 | Reverted |

Step 9 is the important counterexample to accepting throughput alone: it
completed three more tasks than Step 4, but simulator speed fell 14.8%,
unplanned stationary time rose 26.3%, physical escapes more than doubled, and
spin/reversal counts also increased. Step 10 reduced same-robot escape retries
within five seconds from 18 to 4, but converted route churn into longer stalls;
total escapes rose from 115 to 165 and throughput fell to 75.

## Final retained changes

1. `6bf5ad4 Throttle calm-period joint replanning`
   - Calm rolling-plan refresh changed from 2.0 s to 3.0 s.
   - Predicted conflicts still pull planning forward immediately.
2. `3746c47 Apply coordinated speed scaling once`
   - Removed the duplicate `speed_scale` multiplication inside coordinated
     control; scaling remains at the final motor-output stage.
   - Added a protocol regression test proving scaling is applied exactly once.

Relative to the starting recording, the retained result improves throughput
from 73 to 84 tasks (+15.1%), cuts total stationary time from 1002 to 624 seconds
(-37.7%), cuts unplanned stationary time from 573 to 373 seconds (-34.9%), cuts
spin time from 46 to 15 seconds (-67.4%), cuts reversals from 423 to 369
(-12.8%), cuts physical escapes from 243 to 115 (-52.7%), and shortens the
longest completion plateau from 131.536 to 94.112 seconds (-28.4%). Webots
average speed factor is 3.2575x, safely above the required 1.2x.

The retained result JSON also reports zero safety events, zero deadlocks, zero
pair-distance violations, and a metrics minimum pair distance of 0.534 m.

## Verification and artifacts

- Final post-rollback suite, run twice: `129 passed, 4 subtests passed` both
  times.
- Tracked working tree is clean at `3746c47`; only this artifact directory is
  intentionally untracked because the HTML5 recording frame JSON files are
  large (about 65-68 MiB each).
- Each run directory contains the `.html`, `.css`, `.x3d`, frame `.json`,
  `metrics.json`, and Webots performance log required to replay or audit it.
- Final retained recording: `step4_single_speed_scale/step4_C_FCFS_seed42_1800.html`.
- Rejected Step 9 recording: `step9_bounded_side_wait_dedup/step9_C_FCFS_seed42_1800.html`.
- Rejected Step 10 recording: `step10_escape_progress_window_5s/step10_C_FCFS_seed42_1800.html`.

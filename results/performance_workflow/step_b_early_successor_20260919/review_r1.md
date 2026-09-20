# Step B code review R1

Scope: correctness, causal fit and transaction semantics. Step B0 recording
showed 20 accepted stale-wait replans. The old endpoint handler deliberately
ignored the first member until all transaction members completed, creating the
wait that the watchdog later treated as stale. The change uses the first
endpoint as a preparation edge but requests every member of the active epoch;
it never creates a single-robot successor. The old prefix remains active until
the existing planner, validator and atomic prepare/arm/commit sequence succeeds.

Verdict: PASS

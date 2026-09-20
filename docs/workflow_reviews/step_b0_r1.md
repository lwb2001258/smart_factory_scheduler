# Step B0 revised-scheme and code review R1

Step A recording disproved the assumption that ordinary rolling refresh is the
dominant accepted request: 48/58 accepted requests were aggregated watchdog
events. Before changing eligibility, this step splits that aggregate into
emergency, hard-motion, route-less and stale-wait causes. The priority order is
intentional: emergency is safety-critical, then measured hard no-progress,
then missing route, then stale legal wait. Robot sets and the single coalesced
planner edge remain identical.

Verdict: PASS

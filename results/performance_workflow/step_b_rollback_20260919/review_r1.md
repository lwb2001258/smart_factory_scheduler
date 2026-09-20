# Step B rollback review R1

The recorded candidate was behavior-identical to its baseline and failed the
mandatory material-improvement gate. Reverting only the early-endpoint branch
and its changed assertion restores the last accepted B0 behavior while keeping
the independently accepted request provenance. No unrelated user changes are
touched.

Verdict: PASS

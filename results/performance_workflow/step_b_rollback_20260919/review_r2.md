# Step B rollback review R2

The rollback restores the previously tested rule that an active transaction
is not replaced from a single member endpoint. Safety, transaction atomicity,
request metrics and watchdog-cause provenance remain intact. A fresh Webots
recording and no-recording performance run are required to confirm restoration.

Verdict: PASS

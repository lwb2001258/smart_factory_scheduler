# Step K workflow review R1 — correctness and evidence integrity

Scope: the 5–10 minute Webots performance workflow change, including duration
validation, telemetry-derived reservation deadline episodes, throughput and
completion plateau gates, and their tests.

Findings: the first pass found that the requested duration alone could not
prove Webots actually completed the requested recording. The implementation
was corrected to read `experiment_info.sim_duration`, require the observed
duration to cover the request, and allow only the expected final basic-step
overshoot. Deadline stops are counted on false-to-true transitions per robot,
so telemetry sample rate cannot inflate the result. Baseline comparisons also
require matching requested durations. Targeted workflow tests pass.

Verdict: PASS

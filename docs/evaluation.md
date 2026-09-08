# Reproducible evaluation

The 12 scenarios cover nominal motion, curved motion, short and repeated frame loss, lane loss/recovery, stale/corrupt timestamps, backend failure, speed injection, steering injection, and clean replay termination.

Each run writes:

- `telemetry.v1.jsonl`: one record per deterministic cycle;
- `summary.json`: frame count, degraded frames, safe-stop frames and clamped frames;
- `index.html`: a human-readable report generated from telemetry only.

The benchmark reports p50, p95 and maximum mock-perception latency. It is a regression signal, not a claim about a particular board. ARM64 CI proves compilation portability; only a separately recorded board run may claim board performance.

## Adding a scenario

Keep the file deterministic and include an explicit `seed`, `frames`, `profile`, and fault timeline. Add a test assertion for the safety state or command envelope. Do not commit private recordings or data without a provenance and redistribution record.

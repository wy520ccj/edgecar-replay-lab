# Week 1 status: trustworthy replay foundation

This document records the first hardening increment after the v0.1.0 prototype. It is a status record for the repository and does not claim real-vehicle validation.

## Completed in this increment

- Safety limits are validated at construction time.
- NaN and infinite command/perception values enter `LATCHED_STOP` and emit zero speed with braking.
- Explicit brake requests take priority over motion and remain visible in telemetry.
- Repeated, backward, stale, and impossible future timestamps are rejected as hard faults.
- Stop decisions retain the last valid steering value, so a stop does not create an artificial steering jump.
- Every replay appends a `terminal: true` zero-speed/brake record with reason `replay_complete`.
- Scenario seeds now affect synthetic traces while preserving deterministic replay for the same seed.
- Telemetry reading supports scientific notation, validates required values and booleans, rejects malformed lines and unknown safety states, and keeps optional brake/terminal fields backward-compatible.
- `NullActuator` is now a concrete no-op sink and the replay path calls the recording sink for every approved command.
- The test executable contains 15 named checks covering safety boundaries, parsing, determinism, terminal output, and report round trips.

## Verification run

```text
cmake --preset release
cmake --build --preset release --parallel
ctest --test-dir build/release --output-on-failure
scripts/run_demo.ps1
```

The release build and tests pass locally. The demo script completes all 12 scenarios and regenerates `out/demo/index.html`.

## Deliberately deferred

The following belong to the next increments: dependency-backed schema validation, injected runtime components, `edgecar validate/run/compare` commands, Python bindings, measured-versus-simulated timing separation, discrete-event scheduling, failure minimization, and the OpenCV pixel-based backend. Until those are delivered, the built-in perception remains a deterministic mock and its latency is not a model or board measurement.

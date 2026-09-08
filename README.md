# EdgeCar Replay Lab

Deterministic replay and safety validation for resource-constrained autonomous vehicles.

![C++17](https://img.shields.io/badge/C%2B%2B-17-blue) ![License](https://img.shields.io/badge/license-Apache--2.0-green)

[Live replay report](https://wy520ccj.github.io/edgecar-replay-lab/)

EdgeCar Replay Lab is a small, hardware-independent C++17 reference stack for testing the part of an autonomous-car pipeline that is easiest to get wrong: what happens when perception is late, missing, stale, or unreasonable. It runs the same deterministic scenario repeatedly, injects faults, clamps actuator commands, and produces machine-readable telemetry plus an HTML report.

## Why this project

This is a software-in-the-loop lab, not a claim of road-ready autonomy. The design borrows practical ideas from autonomous-racing and automotive projects: modular interfaces, latest-frame processing, explicit safety supervision, replayable scenarios, and observable decisions. It deliberately keeps the core dependency-free so it can be built on a laptop and cross-compiled for ARM64.

## Quick start

```bash
cmake --preset release
cmake --build --preset release
ctest --test-dir build/release --output-on-failure
./build/release/bin/edgecar-replay --scenario scenarios/07_stale_timestamp.yaml --output out/stale
./build/release/bin/edgecar-report --telemetry out/stale/telemetry.v1.jsonl --output out/stale/index.html
```

On Windows PowerShell, run `./scripts/run_demo.ps1`. It configures, builds, tests all scenarios and creates `out/demo/index.html`.

## Architecture

```text
Scenario / video → FrameSource → Perception → Planner
                                         ↓
NullActuator ← SafetySupervisor ← requested command
                     ↓
             telemetry.v1.jsonl → HTML report
```

The core contracts live in `include/edgecar/`. Optional image/model adapters can implement the same interfaces without changing the safety or replay code. No binary model, course material, vehicle image, or vendor source is included.

## Safety behaviour

- `NORMAL`: healthy perception and commands inside the configured envelope.
- `DEGRADED`: a short perception gap; speed is reduced while the trace remains recoverable.
- `SAFE_STOP`: repeated missing observations; command is zero speed with braking requested.
- `LATCHED_STOP`: stale timestamps or backend failure; motion stays disabled until an explicit reset.

The default actuator is `NullActuator`; replay never opens a serial port or drives a motor.

## Evidence and limits

The repository's metrics are generated from synthetic, deterministic scenarios and are reproducible in CI. Historical EdgeBoard/course measurements are kept outside this public repository and are not presented as results of this code. This project is not certified for use on public roads and is not an ISO 26262 implementation.

## Documentation

- [中文说明](README.zh-CN.md)
- [Architecture and data contracts](docs/architecture.md)
- [Reproducible evaluation](docs/evaluation.md)
- [Contributing](CONTRIBUTING.md)
- [Security policy](SECURITY.md)

## License

Original source in this repository is licensed under Apache-2.0. See [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

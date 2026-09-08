# Resume evidence template

Use only numbers produced by a tagged release's `out/` report. Replace the bracketed values after running the release checklist.

**Embedded perception / vehicle software**

- Designed and implemented an original C++17 autonomous-vehicle replay stack with dependency-light interfaces for frame input, perception, planning, safety supervision, and actuator output.
- Built deterministic fault-injection and safety regression tests covering 12 scenarios; verified `[N/N]` scenario passes, `[X]%` core/safety coverage, and bounded speed/steering commands under stale-frame and perception-failure conditions.
- Added machine-readable vehicle telemetry and an HTML evaluation report with p50/p95/max latency, lateral error, degraded-mode duration, and safe-stop response; cross-compiled the core for ARM64 in CI.

Do not write that this repository has completed real-vehicle autonomy, road deployment, ISO 26262 compliance, or T710 validation. Historical course-board results belong to a separate private project and must be described separately.

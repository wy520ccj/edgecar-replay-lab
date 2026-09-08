#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root"
cmake --preset release
cmake --build --preset release --parallel
ctest --preset debug --output-on-failure
rm -rf out/demo
mkdir -p out/demo
for scenario in scenarios/*.yaml; do
  name="$(basename "$scenario" .yaml)"
  build/release/bin/edgecar-replay --scenario "$scenario" --output "out/demo/$name"
done
build/release/bin/edgecar-report --telemetry out/demo/03_s_curve_nominal/telemetry.v1.jsonl --output out/demo/index.html
echo "Demo report: $root/out/demo/index.html"

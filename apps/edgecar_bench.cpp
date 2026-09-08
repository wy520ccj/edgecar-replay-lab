#include "edgecar/pipeline.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  std::size_t frames = 300;
  if (argc > 1 && std::string(argv[1]) == "--frames" && argc > 2)
    frames = static_cast<std::size_t>(std::stoull(argv[2]));
  edgecar::Scenario scenario;
  scenario.id = "benchmark";
  scenario.frames = std::max<std::size_t>(1, frames);
  scenario.profile = "s_curve";
  const auto result = edgecar::run_scenario(scenario);
  std::vector<double> latency;
  latency.reserve(result.telemetry.size());
  for (const auto& record : result.telemetry) latency.push_back(record.perception_latency_ms);
  std::sort(latency.begin(), latency.end());
  const auto percentile = [&latency](double ratio) {
    const auto index = static_cast<std::size_t>(ratio * static_cast<double>(latency.size() - 1));
    return latency[index];
  };
  std::cout << std::fixed << std::setprecision(3)
            << "{\"backend\":\"mock\",\"frames\":" << result.telemetry.size()
            << ",\"latency_p50_ms\":" << percentile(0.50)
            << ",\"latency_p95_ms\":" << percentile(0.95)
            << ",\"latency_max_ms\":" << latency.back()
            << ",\"degraded_frames\":" << result.degraded_frames
            << ",\"safe_stop_frames\":" << result.safe_stop_frames << "}\n";
  return 0;
}

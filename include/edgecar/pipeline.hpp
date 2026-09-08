#pragma once

#include "edgecar/scenario.hpp"
#include "edgecar/safety.hpp"
#include "edgecar/telemetry.hpp"

#include <vector>

namespace edgecar {

struct RunResult {
  Scenario scenario;
  std::vector<TelemetryRecord> telemetry;
  std::size_t safe_stop_frames{0};
  std::size_t degraded_frames{0};
  std::size_t clamped_frames{0};
};

RunResult run_scenario(const Scenario& scenario,
                       SafetyLimits limits = {});

}  // namespace edgecar

#pragma once

#include "edgecar/interfaces.hpp"

#include <string>

namespace edgecar {

Scenario load_scenario(const std::string& path);
bool fault_active(const Scenario& scenario, FaultType type, std::size_t frame);
double fault_value(const Scenario& scenario, FaultType type, std::size_t frame,
                   double fallback = 0.0);

class SyntheticFrameSource final : public IFrameSource {
 public:
  explicit SyntheticFrameSource(Scenario scenario);
  std::optional<Frame> next() override;

 private:
  Scenario scenario_;
  std::size_t next_frame_{0};
};

class ReplayPerception final : public IPerceptionBackend {
 public:
  PerceptionResult infer(const Frame& frame) override;
};

class LanePlanner final : public IPlanner {
 public:
  ControlCommand plan(const VehicleState& state,
                      const PerceptionResult& perception) override;
};

}  // namespace edgecar

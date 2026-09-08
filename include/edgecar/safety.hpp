#pragma once

#include "edgecar/interfaces.hpp"

namespace edgecar {

struct SafetyLimits {
  double max_speed_mps{0.8};
  double degraded_speed_mps{0.25};
  double max_abs_steering{1.0};
  double max_steering_delta{0.20};
  std::int64_t max_frame_age_ms{120};
  std::size_t miss_to_degraded{1};
  std::size_t miss_to_stop{3};
};

class SafetySupervisor final : public ISafetySupervisor {
 public:
  explicit SafetySupervisor(SafetyLimits limits = {});
  SafetyDecision filter(const Frame& frame,
                        const PerceptionResult& perception,
                        const ControlCommand& requested,
                        std::int64_t cycle_time_ms) override;
  void reset() override;
  SafetyState state() const { return state_; }

 private:
  SafetyLimits limits_;
  SafetyState state_{SafetyState::Normal};
  std::size_t consecutive_misses_{0};
  double last_steering_{0.0};
};

class RecordingActuator final : public IActuatorSink {
 public:
  void send(const ControlCommand& command) override { last_command_ = command; }
  const ControlCommand& last_command() const { return last_command_; }

 private:
  ControlCommand last_command_{};
};

}  // namespace edgecar

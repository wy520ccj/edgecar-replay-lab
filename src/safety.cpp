#include "edgecar/safety.hpp"

#include <algorithm>
#include <cmath>

namespace edgecar {

SafetySupervisor::SafetySupervisor(SafetyLimits limits) : limits_(limits) {}

void SafetySupervisor::reset() {
  state_ = SafetyState::Normal;
  consecutive_misses_ = 0;
  last_steering_ = 0.0;
}

SafetyDecision SafetySupervisor::filter(const Frame& frame,
                                        const PerceptionResult& perception,
                                        const ControlCommand& requested,
                                        std::int64_t cycle_time_ms) {
  SafetyDecision decision;
  decision.command = requested;
  if (state_ == SafetyState::LatchedStop) {
    decision.state = state_;
    decision.reason = "latched_stop_requires_reset";
    decision.command = ControlCommand{0.0, 0.0, true, decision.reason};
    return decision;
  }

  const auto age = cycle_time_ms - frame.timestamp_ms;
  if (age > limits_.max_frame_age_ms || age < -20) {
    state_ = SafetyState::LatchedStop;
    decision.state = state_;
    decision.reason = "frame_timestamp_invalid_or_stale";
    decision.command = ControlCommand{0.0, 0.0, true, decision.reason};
    return decision;
  }

  const bool miss = !frame.valid || !perception.lane.valid;
  if (miss) {
    ++consecutive_misses_;
  } else {
    consecutive_misses_ = 0;
  }
  if (perception.health == Health::Failed) {
    state_ = SafetyState::LatchedStop;
    decision.state = state_;
    decision.reason = "perception_backend_failed";
    decision.command = ControlCommand{0.0, 0.0, true, decision.reason};
    return decision;
  }
  if (consecutive_misses_ >= limits_.miss_to_stop) {
    state_ = SafetyState::SafeStop;
    decision.state = state_;
    decision.reason = "consecutive_perception_misses";
    decision.command = ControlCommand{0.0, 0.0, true, decision.reason};
    last_steering_ = 0.0;
    return decision;
  }

  const double limited_speed = std::clamp(requested.speed_mps, 0.0, limits_.max_speed_mps);
  const double limited_steering = std::clamp(requested.steering_norm,
                                             -limits_.max_abs_steering,
                                             limits_.max_abs_steering);
  const double slew_limited = std::clamp(limited_steering,
                                         last_steering_ - limits_.max_steering_delta,
                                         last_steering_ + limits_.max_steering_delta);
  const bool clamped = std::abs(limited_speed - requested.speed_mps) > 1e-9 ||
                       std::abs(slew_limited - requested.steering_norm) > 1e-9;
  decision.command.speed_mps = limited_speed;
  decision.command.steering_norm = slew_limited;
  decision.command.brake = false;
  decision.reason = clamped ? "command_clamped" : "normal";
  last_steering_ = slew_limited;

  if (consecutive_misses_ >= limits_.miss_to_degraded ||
      perception.health == Health::Degraded) {
    state_ = SafetyState::Degraded;
    decision.command.speed_mps = std::min(decision.command.speed_mps,
                                          limits_.degraded_speed_mps);
    decision.reason = "degraded_perception";
  } else {
    state_ = SafetyState::Normal;
  }
  decision.state = state_;
  return decision;
}

}  // namespace edgecar

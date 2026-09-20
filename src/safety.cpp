#include "edgecar/safety.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace edgecar {

namespace {

bool finite(double value) { return std::isfinite(value); }

bool finite(const ControlCommand& command) {
  return finite(command.speed_mps) && finite(command.steering_norm);
}

bool finite(const PerceptionResult& perception) {
  return finite(perception.latency_ms) && finite(perception.lane.lateral_error) &&
         finite(perception.lane.heading_error) && finite(perception.lane.confidence);
}

SafetyDecision stop_decision(SafetyState state, const std::string& reason,
                             double steering) {
  SafetyDecision decision;
  decision.state = state;
  decision.reason = reason;
  decision.command = ControlCommand{0.0, steering, true, reason};
  return decision;
}

}  // namespace

void SafetyLimits::validate() const {
  if (!finite(max_speed_mps) || max_speed_mps < 0.0) throw std::invalid_argument("max_speed_mps must be finite and non-negative");
  if (!finite(degraded_speed_mps) || degraded_speed_mps < 0.0 || degraded_speed_mps > max_speed_mps)
    throw std::invalid_argument("degraded_speed_mps must be within the speed envelope");
  if (!finite(max_abs_steering) || max_abs_steering < 0.0) throw std::invalid_argument("max_abs_steering must be finite and non-negative");
  if (!finite(max_steering_delta) || max_steering_delta < 0.0) throw std::invalid_argument("max_steering_delta must be finite and non-negative");
  if (max_frame_age_ms < 0 || max_future_frame_ms < 0) throw std::invalid_argument("frame age limits must be non-negative");
  if (miss_to_degraded == 0 || miss_to_stop == 0 || miss_to_degraded > miss_to_stop)
    throw std::invalid_argument("miss thresholds must be positive and ordered");
}

SafetySupervisor::SafetySupervisor(SafetyLimits limits) : limits_(limits) {
  limits_.validate();
}

void SafetySupervisor::reset() {
  state_ = SafetyState::Normal;
  consecutive_misses_ = 0;
  last_steering_ = 0.0;
  last_frame_timestamp_ms_.reset();
}

SafetyDecision SafetySupervisor::filter(const Frame& frame,
                                        const PerceptionResult& perception,
                                        const ControlCommand& requested,
                                        std::int64_t cycle_time_ms) {
  SafetyDecision decision;
  if (!finite(requested) || !finite(perception)) {
    state_ = SafetyState::LatchedStop;
    return stop_decision(state_, "non_finite_input", last_steering_);
  }
  if (state_ == SafetyState::LatchedStop) {
    return stop_decision(state_, "latched_stop_requires_reset", last_steering_);
  }

  const auto age = cycle_time_ms - frame.timestamp_ms;
  const bool repeated_or_backward = last_frame_timestamp_ms_.has_value() &&
                                    frame.timestamp_ms <= *last_frame_timestamp_ms_;
  if (age > limits_.max_frame_age_ms || age < -limits_.max_future_frame_ms ||
      repeated_or_backward) {
    state_ = SafetyState::LatchedStop;
    return stop_decision(state_, repeated_or_backward ? "frame_timestamp_repeated_or_backward"
                                                      : "frame_timestamp_invalid_or_stale",
                         last_steering_);
  }
  last_frame_timestamp_ms_ = frame.timestamp_ms;

  const bool miss = !frame.valid || !perception.lane.valid;
  if (miss) {
    ++consecutive_misses_;
  } else {
    consecutive_misses_ = 0;
  }
  if (perception.health == Health::Failed) {
    state_ = SafetyState::LatchedStop;
    return stop_decision(state_, "perception_backend_failed", last_steering_);
  }
  if (consecutive_misses_ >= limits_.miss_to_stop) {
    state_ = SafetyState::SafeStop;
    return stop_decision(state_, "consecutive_perception_misses", last_steering_);
  }

  if (requested.brake) {
    decision.state = perception.health == Health::Degraded || consecutive_misses_ >= limits_.miss_to_degraded
                        ? SafetyState::Degraded : SafetyState::Normal;
    state_ = decision.state;
    decision.reason = "brake_requested";
    decision.command = ControlCommand{0.0, last_steering_, true, decision.reason};
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

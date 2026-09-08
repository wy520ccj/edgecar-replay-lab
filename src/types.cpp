#include "edgecar/types.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace edgecar {

std::string to_string(Health value) {
  switch (value) {
    case Health::Healthy: return "HEALTHY";
    case Health::Degraded: return "DEGRADED";
    case Health::Failed: return "FAILED";
  }
  return "UNKNOWN";
}

std::string to_string(SafetyState value) {
  switch (value) {
    case SafetyState::Normal: return "NORMAL";
    case SafetyState::Degraded: return "DEGRADED";
    case SafetyState::SafeStop: return "SAFE_STOP";
    case SafetyState::LatchedStop: return "LATCHED_STOP";
  }
  return "UNKNOWN";
}

std::string to_string(FaultType value) {
  switch (value) {
    case FaultType::DropFrame: return "drop_frame";
    case FaultType::Delay: return "delay";
    case FaultType::LaneLoss: return "lane_loss";
    case FaultType::DetectorFailure: return "detector_failure";
    case FaultType::CorruptTimestamp: return "corrupt_timestamp";
    case FaultType::OverSpeed: return "over_speed";
    case FaultType::SteeringSpike: return "steering_spike";
  }
  return "unknown";
}

FaultType fault_type_from_string(const std::string& value) {
  if (value == "drop_frame") return FaultType::DropFrame;
  if (value == "delay") return FaultType::Delay;
  if (value == "lane_loss") return FaultType::LaneLoss;
  if (value == "detector_failure") return FaultType::DetectorFailure;
  if (value == "corrupt_timestamp") return FaultType::CorruptTimestamp;
  if (value == "over_speed") return FaultType::OverSpeed;
  if (value == "steering_spike") return FaultType::SteeringSpike;
  throw std::invalid_argument("unknown fault type: " + value);
}

}  // namespace edgecar

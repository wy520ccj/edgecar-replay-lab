#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace edgecar {

enum class Health { Healthy, Degraded, Failed };
enum class SafetyState { Normal, Degraded, SafeStop, LatchedStop };
enum class FaultType {
  DropFrame,
  Delay,
  LaneLoss,
  DetectorFailure,
  CorruptTimestamp,
  OverSpeed,
  SteeringSpike,
};

struct FaultEvent {
  FaultType type{FaultType::DropFrame};
  std::size_t start_frame{0};
  std::size_t duration_frames{1};
  double value{0.0};
};

struct Scenario {
  std::string id{"unnamed"};
  std::uint32_t seed{1};
  std::size_t frames{120};
  std::string profile{"straight"};
  std::vector<FaultEvent> faults;
};

struct Frame {
  std::uint64_t sequence{0};
  std::int64_t timestamp_ms{0};
  std::uint32_t width{320};
  std::uint32_t height{240};
  bool valid{true};
  bool lane_lost{false};
  bool detector_failed{false};
  double truth_lateral_error{0.0};
  double truth_heading_error{0.0};
};

struct LaneObservation {
  bool valid{false};
  double lateral_error{0.0};
  double heading_error{0.0};
  double confidence{0.0};
};

struct Detection {
  std::string label;
  double confidence{0.0};
};

struct PerceptionResult {
  LaneObservation lane;
  std::vector<Detection> detections;
  double latency_ms{0.0};
  Health health{Health::Healthy};
};

struct VehicleState {
  double speed_mps{0.0};
  double steering_norm{0.0};
};

struct ControlCommand {
  double speed_mps{0.0};
  double steering_norm{0.0};
  bool brake{false};
  std::string reason{"initial"};
};

struct SafetyDecision {
  SafetyState state{SafetyState::Normal};
  ControlCommand command;
  std::string reason{"normal"};
};

struct TelemetryRecord {
  std::string scenario_id;
  std::uint64_t frame{0};
  std::int64_t timestamp_ms{0};
  bool frame_valid{true};
  bool lane_valid{false};
  double lane_confidence{0.0};
  double lateral_error{0.0};
  double requested_speed_mps{0.0};
  double requested_steering_norm{0.0};
  double applied_speed_mps{0.0};
  double applied_steering_norm{0.0};
  double perception_latency_ms{0.0};
  SafetyState safety_state{SafetyState::Normal};
  std::string safety_reason{"normal"};
};

std::string to_string(Health value);
std::string to_string(SafetyState value);
std::string to_string(FaultType value);
FaultType fault_type_from_string(const std::string& value);

}  // namespace edgecar

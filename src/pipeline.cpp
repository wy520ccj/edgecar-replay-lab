#include "edgecar/pipeline.hpp"

#include <algorithm>

namespace edgecar {

RunResult run_scenario(const Scenario& scenario, SafetyLimits limits) {
  RunResult result;
  result.scenario = scenario;
  SyntheticFrameSource source(scenario);
  ReplayPerception perception_backend;
  LanePlanner planner;
  SafetySupervisor supervisor(limits);
  VehicleState vehicle;

  for (std::size_t cycle = 0; cycle < scenario.frames; ++cycle) {
    const auto maybe_frame = source.next();
    if (!maybe_frame) break;
    const Frame frame = *maybe_frame;
    const PerceptionResult perception = perception_backend.infer(frame);
    ControlCommand requested = planner.plan(vehicle, perception);
    if (fault_active(scenario, FaultType::OverSpeed, cycle)) {
      requested.speed_mps = fault_value(scenario, FaultType::OverSpeed, cycle, 2.0);
      requested.reason = "injected_overspeed";
    }
    if (fault_active(scenario, FaultType::SteeringSpike, cycle)) {
      requested.steering_norm = fault_value(scenario, FaultType::SteeringSpike, cycle, 2.0);
      requested.reason = "injected_steering_spike";
    }
    const SafetyDecision decision = supervisor.filter(
        frame, perception, requested, static_cast<std::int64_t>(cycle * 33U));
    vehicle.speed_mps = decision.command.speed_mps;
    vehicle.steering_norm = decision.command.steering_norm;

    TelemetryRecord record;
    record.scenario_id = scenario.id;
    record.frame = frame.sequence;
    record.timestamp_ms = frame.timestamp_ms;
    record.frame_valid = frame.valid;
    record.lane_valid = perception.lane.valid;
    record.lane_confidence = perception.lane.confidence;
    record.lateral_error = perception.lane.lateral_error;
    record.requested_speed_mps = requested.speed_mps;
    record.requested_steering_norm = requested.steering_norm;
    record.applied_speed_mps = decision.command.speed_mps;
    record.applied_steering_norm = decision.command.steering_norm;
    record.perception_latency_ms = perception.latency_ms;
    record.safety_state = decision.state;
    record.safety_reason = decision.reason;
    result.telemetry.push_back(record);

    if (decision.state == SafetyState::SafeStop || decision.state == SafetyState::LatchedStop)
      ++result.safe_stop_frames;
    if (decision.state == SafetyState::Degraded) ++result.degraded_frames;
    if (decision.reason == "command_clamped") ++result.clamped_frames;
  }
  return result;
}

}  // namespace edgecar

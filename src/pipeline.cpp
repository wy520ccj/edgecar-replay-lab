#include "edgecar/pipeline.hpp"

#include <algorithm>

namespace edgecar {

RunResult run_scenario(const Scenario& scenario, SafetyLimits limits) {
  validate_scenario(scenario);
  RunResult result;
  result.scenario = scenario;
  SyntheticFrameSource source(scenario);
  ReplayPerception perception_backend;
  LanePlanner planner;
  SafetySupervisor supervisor(limits);
  RecordingActuator actuator;
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
    actuator.send(decision.command);

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
    record.requested_brake = requested.brake;
    record.applied_speed_mps = decision.command.speed_mps;
    record.applied_steering_norm = decision.command.steering_norm;
    record.applied_brake = decision.command.brake;
    record.perception_latency_ms = perception.latency_ms;
    record.safety_state = decision.state;
    record.safety_reason = decision.reason;
    result.telemetry.push_back(record);

    if (decision.state == SafetyState::SafeStop || decision.state == SafetyState::LatchedStop)
      ++result.safe_stop_frames;
    if (decision.state == SafetyState::Degraded) ++result.degraded_frames;
    if (decision.reason == "command_clamped") ++result.clamped_frames;
  }
  if (!result.telemetry.empty()) {
    TelemetryRecord terminal = result.telemetry.back();
    terminal.frame = result.telemetry.back().frame + 1U;
    terminal.timestamp_ms = result.telemetry.back().timestamp_ms + 33;
    terminal.requested_speed_mps = 0.0;
    terminal.requested_steering_norm = result.telemetry.back().applied_steering_norm;
    terminal.requested_brake = true;
    terminal.applied_speed_mps = 0.0;
    terminal.applied_steering_norm = result.telemetry.back().applied_steering_norm;
    terminal.applied_brake = true;
    terminal.safety_reason = "replay_complete";
    terminal.terminal = true;
    actuator.send(ControlCommand{0.0, terminal.applied_steering_norm, true, terminal.safety_reason});
    result.telemetry.push_back(terminal);
  }
  return result;
}

}  // namespace edgecar

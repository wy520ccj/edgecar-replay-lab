#include "edgecar/pipeline.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

edgecar::Scenario base(const std::string& id) {
  edgecar::Scenario scenario;
  scenario.id = id;
  scenario.frames = 60;
  scenario.profile = "curve";
  return scenario;
}

void test_scenario_loader() {
  const auto path = std::filesystem::temp_directory_path() / "edgecar_test.yaml";
  std::ofstream file(path);
  file << "id: loader\nseed: 42\nframes: 9\nprofile: s_curve\nfaults:\n"
          "  - type: drop_frame\n    frame: 3\n    duration: 2\n"
          "  - type: delay\n    frame: 7\n    duration: 1\n    value: 80\n";
  file.close();
  const auto scenario = edgecar::load_scenario(path.string());
  require(scenario.id == "loader" && scenario.seed == 42 && scenario.frames == 9,
          "scenario scalar fields were not parsed");
  require(scenario.profile == "s_curve" && scenario.faults.size() == 2,
          "scenario faults were not parsed");
  require(edgecar::fault_active(scenario, edgecar::FaultType::DropFrame, 4),
          "drop fault range is wrong");
  std::filesystem::remove(path);
}

void test_invalid_scenario_rejected() {
  const auto path = std::filesystem::temp_directory_path() / "edgecar_invalid_scenario.yaml";
  {
    std::ofstream file(path);
    file << "id: invalid\nframes: 4\nprofile: unknown_profile\n";
  }
  bool rejected = false;
  try {
    (void)edgecar::load_scenario(path.string());
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  std::filesystem::remove(path);
  require(rejected, "invalid scenario profile was accepted");
}

void test_determinism() {
  const auto first = edgecar::run_scenario(base("determinism"));
  const auto second = edgecar::run_scenario(base("determinism"));
  require(first.telemetry.size() == second.telemetry.size(), "run length changed");
  for (std::size_t index = 0; index < first.telemetry.size(); ++index)
    require(edgecar::telemetry_json(first.telemetry[index]) ==
                edgecar::telemetry_json(second.telemetry[index]),
            "same scenario produced a different trace");
}

void test_short_drop_degrades_without_stop() {
  auto scenario = base("short_drop");
  scenario.faults.push_back({edgecar::FaultType::DropFrame, 10, 1, 0.0});
  const auto result = edgecar::run_scenario(scenario);
  require(result.degraded_frames >= 1, "one dropped frame did not degrade");
  require(result.safe_stop_frames == 0, "one dropped frame caused a stop");
}

void test_consecutive_drop_stops() {
  auto scenario = base("drop_stop");
  scenario.faults.push_back({edgecar::FaultType::DropFrame, 10, 3, 0.0});
  const auto result = edgecar::run_scenario(scenario);
  require(result.safe_stop_frames >= 1, "consecutive dropped frames did not stop");
}

void test_hard_fault_latches() {
  auto scenario = base("hard_fault");
  scenario.faults.push_back({edgecar::FaultType::DetectorFailure, 12, 1, 0.0});
  const auto result = edgecar::run_scenario(scenario);
  require(result.safe_stop_frames >= 1, "detector failure did not stop");
  require(result.telemetry.back().safety_state == edgecar::SafetyState::LatchedStop,
          "hard perception fault was not latched");
}

void test_stale_frame_latches() {
  auto scenario = base("stale");
  scenario.faults.push_back({edgecar::FaultType::Delay, 8, 1, 250.0});
  const auto result = edgecar::run_scenario(scenario);
  require(result.telemetry[8].safety_state == edgecar::SafetyState::LatchedStop,
          "stale frame did not latch stop");
  require(result.telemetry.back().applied_speed_mps == 0.0,
          "latched stop later emitted motion");
}

void test_command_envelope() {
  auto scenario = base("envelope");
  scenario.faults.push_back({edgecar::FaultType::OverSpeed, 5, 1, 3.0});
  scenario.faults.push_back({edgecar::FaultType::SteeringSpike, 20, 1, 3.0});
  const auto result = edgecar::run_scenario(scenario);
  require(result.clamped_frames >= 2, "command envelope did not clamp injections");
  for (const auto& record : result.telemetry) {
    require(record.applied_speed_mps >= 0.0 && record.applied_speed_mps <= 0.8 + 1e-9,
            "speed envelope violated");
    require(std::abs(record.applied_steering_norm) <= 1.0 + 1e-9,
            "steering envelope violated");
  }
}

void test_report_round_trip() {
  const auto scenario = base("report");
  const auto result = edgecar::run_scenario(scenario);
  const auto directory = std::filesystem::temp_directory_path() / "edgecar_report_test";
  const auto jsonl = directory / "telemetry.jsonl";
  const auto html = directory / "index.html";
  edgecar::write_telemetry(jsonl.string(), result.telemetry);
  const auto loaded = edgecar::read_telemetry(jsonl.string());
  require(loaded.size() == result.telemetry.size(), "telemetry round trip lost records");
  edgecar::write_html_report(html.string(), loaded);
  require(std::filesystem::exists(html) && std::filesystem::file_size(html) > 100,
          "HTML report was not generated");
  std::filesystem::remove_all(directory);
}

void test_non_finite_input_latches() {
  edgecar::SafetySupervisor supervisor;
  edgecar::Frame frame;
  edgecar::PerceptionResult perception;
  perception.lane.valid = true;
  const auto decision = supervisor.filter(
      frame, perception,
      {std::numeric_limits<double>::quiet_NaN(), 0.0, false, "invalid"}, 0);
  require(decision.state == edgecar::SafetyState::LatchedStop,
          "non-finite command did not latch stop");
  require(decision.command.speed_mps == 0.0 && decision.command.brake,
          "non-finite command did not fail safe");
}

void test_brake_request_is_preserved() {
  edgecar::SafetySupervisor supervisor;
  edgecar::Frame frame;
  edgecar::PerceptionResult perception;
  perception.lane.valid = true;
  const auto decision = supervisor.filter(frame, perception, {0.5, 0.0, true, "brake"}, 0);
  require(decision.command.brake && decision.command.speed_mps == 0.0,
          "brake request was not preserved");
}

void test_timestamp_regression_latches() {
  edgecar::SafetySupervisor supervisor;
  edgecar::Frame frame;
  edgecar::PerceptionResult perception;
  perception.lane.valid = true;
  require(supervisor.filter(frame, perception, {}, 0).state == edgecar::SafetyState::Normal,
          "initial timestamp was rejected");
  frame.timestamp_ms = 0;
  const auto decision = supervisor.filter(frame, perception, {}, 33);
  require(decision.state == edgecar::SafetyState::LatchedStop,
          "repeated timestamp did not latch stop");
}

void test_invalid_limits_rejected() {
  bool rejected = false;
  try {
    edgecar::SafetyLimits limits;
    limits.degraded_speed_mps = 2.0;
    edgecar::SafetySupervisor supervisor(limits);
    (void)supervisor;
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "invalid safety limits were accepted");
}

void test_seed_changes_synthetic_trace() {
  auto first = base("seed_first");
  auto second = first;
  second.id = "seed_second";
  second.seed = first.seed + 1;
  const auto first_result = edgecar::run_scenario(first);
  const auto second_result = edgecar::run_scenario(second);
  require(first_result.telemetry[0].lateral_error != second_result.telemetry[0].lateral_error,
          "scenario seed did not affect synthetic trace");
}

void test_terminal_record() {
  const auto result = edgecar::run_scenario(base("terminal"));
  require(result.telemetry.back().terminal, "replay did not emit terminal record");
  require(result.telemetry.back().applied_speed_mps == 0.0 &&
              result.telemetry.back().applied_brake &&
              result.telemetry.back().safety_reason == "replay_complete",
          "terminal record did not command a stop");
}

void test_strict_telemetry_reader() {
  const auto directory = std::filesystem::temp_directory_path() / "edgecar_strict_telemetry";
  std::filesystem::create_directories(directory);
  const auto valid = directory / "valid.jsonl";
  const auto invalid = directory / "invalid.jsonl";
  {
    std::ofstream file(valid);
    file << R"({"scenario_id":"external","frame":0,"timestamp_ms":0,"frame_valid":true,"lane_valid":true,"lane_confidence":9.8e-1,"lateral_error":1e-3,"requested_speed_mps":5e-1,"requested_steering_norm":0e0,"applied_speed_mps":5e-1,"applied_steering_norm":0e0,"perception_latency_ms":8e0,"safety_state":"NORMAL","safety_reason":"normal"})" << '\n';
  }
  const auto loaded = edgecar::read_telemetry(valid.string());
  require(loaded.size() == 1 && loaded[0].lane_valid &&
              std::abs(loaded[0].lateral_error - 1e-3) < 1e-12,
          "strict telemetry reader lost valid JSON values");
  {
    std::ofstream file(invalid);
    file << "THIS IS NOT JSON\n";
  }
  bool rejected = false;
  try {
    (void)edgecar::read_telemetry(invalid.string());
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected, "invalid telemetry line was accepted");
  std::filesystem::remove_all(directory);
}

void test_published_scenarios() {
  struct Expectation {
    const char* file;
    std::size_t degraded;
    std::size_t safe_stop;
    std::size_t clamped;
  };
  const std::vector<Expectation> cases = {
      {"01_straight_nominal.yaml", 0, 0, 0},
      {"02_curve_nominal.yaml", 0, 0, 0},
      {"03_s_curve_nominal.yaml", 0, 0, 0},
      {"04_short_drop.yaml", 1, 0, 0},
      {"05_consecutive_drop.yaml", 2, 1, 0},
      {"06_lane_loss_recovery.yaml", 2, 0, 1},
      {"07_stale_timestamp.yaml", 0, 55, 0},
      {"08_corrupt_timestamp.yaml", 0, 55, 0},
      {"09_detector_failure.yaml", 0, 55, 0},
      {"10_overspeed_injection.yaml", 0, 0, 1},
      {"11_steering_spike.yaml", 0, 0, 1},
      {"12_replay_end.yaml", 0, 0, 0},
  };
  for (const auto& expected : cases) {
    const auto result = edgecar::run_scenario(
        edgecar::load_scenario(std::string("scenarios/") + expected.file));
    require(result.degraded_frames == expected.degraded,
            std::string("unexpected degraded count for ") + expected.file);
    require(result.safe_stop_frames == expected.safe_stop,
            std::string("unexpected stop count for ") + expected.file);
    require(result.clamped_frames == expected.clamped,
            std::string("unexpected clamp count for ") + expected.file);
    require(result.telemetry.back().terminal && result.telemetry.back().applied_brake &&
                result.telemetry.back().applied_speed_mps == 0.0,
            std::string("missing terminal stop for ") + expected.file);
  }
}

}  // namespace

int main() {
  try {
    test_scenario_loader();
    test_invalid_scenario_rejected();
    test_determinism();
    test_short_drop_degrades_without_stop();
    test_consecutive_drop_stops();
    test_hard_fault_latches();
    test_stale_frame_latches();
    test_command_envelope();
    test_report_round_trip();
    test_non_finite_input_latches();
    test_brake_request_is_preserved();
    test_timestamp_regression_latches();
    test_invalid_limits_rejected();
    test_seed_changes_synthetic_trace();
    test_terminal_record();
    test_strict_telemetry_reader();
    test_published_scenarios();
    std::cout << "edgecar-tests: PASS (17 suites)\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "edgecar-tests: FAIL: " << error.what() << '\n';
    return 1;
  }
}

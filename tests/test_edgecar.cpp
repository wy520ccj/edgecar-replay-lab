#include "edgecar/pipeline.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

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

}  // namespace

int main() {
  try {
    test_scenario_loader();
    test_determinism();
    test_short_drop_degrades_without_stop();
    test_consecutive_drop_stops();
    test_hard_fault_latches();
    test_stale_frame_latches();
    test_command_envelope();
    test_report_round_trip();
    std::cout << "edgecar-tests: PASS (8 suites)\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "edgecar-tests: FAIL: " << error.what() << '\n';
    return 1;
  }
}

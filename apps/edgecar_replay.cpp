#include "edgecar/pipeline.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {

void usage() {
  std::cout << "Usage: edgecar-replay --scenario <file.yaml> --output <directory>\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string scenario_path;
  std::string output_dir = "out/replay";
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--scenario" && index + 1 < argc) scenario_path = argv[++index];
    else if (argument == "--output" && index + 1 < argc) output_dir = argv[++index];
    else if (argument == "--help" || argument == "-h") { usage(); return 0; }
    else { usage(); return 2; }
  }
  if (scenario_path.empty()) { usage(); return 2; }
  try {
    const auto scenario = edgecar::load_scenario(scenario_path);
    const auto result = edgecar::run_scenario(scenario);
    std::filesystem::create_directories(output_dir);
    edgecar::write_telemetry(output_dir + "/telemetry.v1.jsonl", result.telemetry);
    std::ofstream summary(output_dir + "/summary.json");
    summary << "{\"scenario_id\":\"" << scenario.id
            << "\",\"frames\":" << result.telemetry.size()
            << ",\"degraded_frames\":" << result.degraded_frames
            << ",\"safe_stop_frames\":" << result.safe_stop_frames
            << ",\"clamped_frames\":" << result.clamped_frames << "}\n";
    std::cout << "scenario=" << scenario.id << " frames=" << result.telemetry.size()
              << " degraded=" << result.degraded_frames
              << " safe_stop=" << result.safe_stop_frames
              << " clamped=" << result.clamped_frames << "\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "edgecar-replay: " << error.what() << '\n';
    return 1;
  }
}

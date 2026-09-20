#include "edgecar/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace edgecar {
namespace {

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::size_t as_size(const std::string& value, const std::string& key) {
  try {
    if (value.empty() || value.front() == '-') throw std::invalid_argument("negative");
    std::size_t consumed = 0;
    const auto parsed = std::stoull(value, &consumed);
    if (consumed != value.size()) throw std::invalid_argument("trailing characters");
    return static_cast<std::size_t>(parsed);
  } catch (...) {
    throw std::invalid_argument("invalid " + key + ": " + value);
  }
}

double as_double(const std::string& value, const std::string& key) {
  try {
    std::size_t consumed = 0;
    const auto parsed = std::stod(value, &consumed);
    if (consumed != value.size() || !std::isfinite(parsed)) throw std::invalid_argument("invalid number");
    return parsed;
  } catch (...) {
    throw std::invalid_argument("invalid " + key + ": " + value);
  }
}

bool known_scenario_key(const std::string& key) {
  return key == "id" || key == "seed" || key == "frames" || key == "profile" || key == "faults";
}

bool known_fault_key(const std::string& key) {
  return key == "type" || key == "frame" || key == "start_frame" ||
         key == "duration" || key == "duration_frames" || key == "value";
}

}  // namespace

Scenario load_scenario(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open scenario: " + path);

  Scenario scenario;
  FaultEvent current;
  bool in_fault = false;
  bool has_type = false;
  std::string line;
  while (std::getline(input, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) line.erase(comment);
    const auto raw = trim(line);
    if (raw.empty() || raw == "faults:") continue;
    if (raw.rfind("- ", 0) == 0) {
      if (in_fault && has_type) scenario.faults.push_back(current);
      current = FaultEvent{};
      in_fault = true;
      has_type = false;
      line = trim(raw.substr(2));
    } else {
      line = raw;
    }

    const auto colon = line.find(':');
    if (colon == std::string::npos) continue;
    const auto key = trim(line.substr(0, colon));
    const auto value = trim(line.substr(colon + 1));
    if (in_fault && (raw.rfind("- ", 0) == 0 || known_fault_key(key))) {
      if (key == "type") {
        current.type = fault_type_from_string(value);
        has_type = true;
      } else if (key == "frame" || key == "start_frame") {
        current.start_frame = as_size(value, key);
      } else if (key == "duration" || key == "duration_frames") {
        current.duration_frames = std::max<std::size_t>(1, as_size(value, key));
      } else if (key == "value") {
        current.value = as_double(value, key);
      } else {
        throw std::invalid_argument("invalid fault key: " + key);
      }
      continue;
    }

    if (key == "id") scenario.id = value;
    else if (key == "seed") scenario.seed = static_cast<std::uint32_t>(as_size(value, key));
    else if (key == "frames") scenario.frames = as_size(value, key);
    else if (key == "profile") scenario.profile = value;
    else if (key == "faults") continue;
    else if (!known_scenario_key(key)) throw std::invalid_argument("invalid scenario key: " + key);
  }
  if (in_fault && has_type) scenario.faults.push_back(current);
  validate_scenario(scenario);
  return scenario;
}

void validate_scenario(const Scenario& scenario) {
  if (scenario.id.empty()) throw std::invalid_argument("scenario id must not be empty");
  if (scenario.frames == 0) throw std::invalid_argument("scenario frames must be positive");
  if (scenario.profile != "straight" && scenario.profile != "curve" && scenario.profile != "s_curve")
    throw std::invalid_argument("unsupported scenario profile: " + scenario.profile);
  for (const auto& fault : scenario.faults) {
    if (fault.duration_frames == 0) throw std::invalid_argument("fault duration must be positive");
    if (fault.start_frame >= scenario.frames) throw std::invalid_argument("fault starts after scenario end");
    if (!std::isfinite(fault.value)) throw std::invalid_argument("fault value must be finite");
  }
}

bool fault_active(const Scenario& scenario, FaultType type, std::size_t frame) {
  return std::any_of(scenario.faults.begin(), scenario.faults.end(),
                     [type, frame](const FaultEvent& fault) {
                       return fault.type == type && frame >= fault.start_frame &&
                              frame < fault.start_frame + fault.duration_frames;
                     });
}

double fault_value(const Scenario& scenario, FaultType type, std::size_t frame,
                   double fallback) {
  for (const auto& fault : scenario.faults) {
    if (fault.type == type && frame >= fault.start_frame &&
        frame < fault.start_frame + fault.duration_frames) {
      return fault.value;
    }
  }
  return fallback;
}

SyntheticFrameSource::SyntheticFrameSource(Scenario scenario)
    : scenario_(std::move(scenario)) {}

std::optional<Frame> SyntheticFrameSource::next() {
  if (next_frame_ >= scenario_.frames) return std::nullopt;
  const auto index = next_frame_++;
  constexpr double pi = 3.14159265358979323846;
  const double seed_offset = static_cast<double>(scenario_.seed % 1000U) / 1000.0;
  const double phase = (static_cast<double>(index) + seed_offset) / 30.0;
  Frame frame;
  frame.sequence = index;
  frame.timestamp_ms = static_cast<std::int64_t>(index * 33U);
  frame.truth_lateral_error = 0.04 * std::sin(phase);
  frame.truth_heading_error = 0.02 * std::cos(phase);
  if (scenario_.profile == "curve") {
    frame.truth_lateral_error = 0.12 * std::sin(phase * 0.65);
    frame.truth_heading_error = 0.08 * std::cos(phase * 0.65);
  } else if (scenario_.profile == "s_curve") {
    frame.truth_lateral_error = 0.16 * std::sin(phase * 0.9);
    frame.truth_heading_error = 0.10 * std::cos(phase * 0.9);
  }
  frame.valid = !fault_active(scenario_, FaultType::DropFrame, index);
  frame.lane_lost = fault_active(scenario_, FaultType::LaneLoss, index);
  frame.detector_failed = fault_active(scenario_, FaultType::DetectorFailure, index);
  if (fault_active(scenario_, FaultType::Delay, index)) {
    frame.timestamp_ms -= static_cast<std::int64_t>(fault_value(scenario_, FaultType::Delay, index, 150.0));
  }
  if (fault_active(scenario_, FaultType::CorruptTimestamp, index)) frame.timestamp_ms = -1000;
  (void)pi;
  return frame;
}

PerceptionResult ReplayPerception::infer(const Frame& frame) {
  PerceptionResult result;
  result.latency_ms = 8.0 + static_cast<double>((frame.sequence * 17U) % 5U) * 0.25;
  if (!frame.valid) {
    result.health = Health::Degraded;
    return result;
  }
  if (frame.detector_failed) {
    result.health = Health::Failed;
    return result;
  }
  if (frame.lane_lost) {
    result.health = Health::Degraded;
    return result;
  }
  result.health = Health::Healthy;
  result.lane.valid = true;
  result.lane.lateral_error = frame.truth_lateral_error;
  result.lane.heading_error = frame.truth_heading_error;
  result.lane.confidence = 0.98;
  return result;
}

ControlCommand LanePlanner::plan(const VehicleState&, const PerceptionResult& perception) {
  ControlCommand command;
  if (!perception.lane.valid) {
    command.reason = "lane_unavailable";
    command.speed_mps = 0.35;
    return command;
  }
  command.steering_norm = 1.45 * perception.lane.lateral_error +
                          0.90 * perception.lane.heading_error;
  command.speed_mps = std::max(0.15, 0.80 - 0.30 * std::abs(command.steering_norm));
  command.reason = "lane_tracking";
  return command;
}

}  // namespace edgecar

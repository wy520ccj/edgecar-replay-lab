#include "edgecar/telemetry.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace edgecar {
namespace {

std::string escape_json(const std::string& value) {
  std::string out;
  for (const char character : value) {
    if (character == '\\') out += "\\\\";
    else if (character == '"') out += "\\\"";
    else if (character == '\n') out += "\\n";
    else out += character;
  }
  return out;
}

std::string escape_html(const std::string& value) {
  std::string out;
  for (const char character : value) {
    if (character == '&') out += "&amp;";
    else if (character == '<') out += "&lt;";
    else if (character == '>') out += "&gt;";
    else if (character == '"') out += "&quot;";
    else out += character;
  }
  return out;
}

template <typename T>
T field_number(const std::string& line, const std::string& key) {
  const std::string number = std::is_integral_v<T>
                                 ? "(-?[0-9]+)"
                                 : "(-?[0-9]+(\\.[0-9]*)?([eE][+-]?[0-9]+)?|\\.[0-9]+([eE][+-]?[0-9]+)?)";
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*" + number + "\\s*[,}]");
  std::smatch match;
  if (!std::regex_search(line, match, pattern)) throw std::runtime_error("missing or invalid numeric field: " + key);
  try {
    if constexpr (std::is_integral_v<T>) return static_cast<T>(std::stoll(match[1].str()));
    const auto value = static_cast<T>(std::stod(match[1].str()));
    if (!std::isfinite(value)) throw std::runtime_error("non-finite numeric field: " + key);
    return value;
  } catch (...) {
    throw std::runtime_error("invalid numeric field: " + key);
  }
}

bool field_bool(const std::string& line, const std::string& key, bool fallback,
                bool required) {
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(true|false)\\s*[,}]");
  std::smatch match;
  if (!std::regex_search(line, match, pattern)) {
    const std::regex present("\\\"" + key + "\\\"\\s*:");
    if (!required && !std::regex_search(line, present)) return fallback;
    throw std::runtime_error("missing or invalid boolean field: " + key);
  }
  return match[1].str() == "true";
}

std::string field_string(const std::string& line, const std::string& key) {
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch match;
  return std::regex_search(line, match, pattern) ? match[1].str() : std::string{};
}

}  // namespace

std::string telemetry_json(const TelemetryRecord& record) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(5)
         << "{\"scenario_id\":\"" << escape_json(record.scenario_id)
         << "\",\"frame\":" << record.frame
         << ",\"timestamp_ms\":" << record.timestamp_ms
         << ",\"frame_valid\":" << (record.frame_valid ? "true" : "false")
         << ",\"lane_valid\":" << (record.lane_valid ? "true" : "false")
         << ",\"lane_confidence\":" << record.lane_confidence
         << ",\"lateral_error\":" << record.lateral_error
         << ",\"requested_speed_mps\":" << record.requested_speed_mps
         << ",\"requested_steering_norm\":" << record.requested_steering_norm
         << ",\"requested_brake\":" << (record.requested_brake ? "true" : "false")
         << ",\"applied_speed_mps\":" << record.applied_speed_mps
         << ",\"applied_steering_norm\":" << record.applied_steering_norm
         << ",\"applied_brake\":" << (record.applied_brake ? "true" : "false")
         << ",\"perception_latency_ms\":" << record.perception_latency_ms
         << ",\"safety_state\":\"" << to_string(record.safety_state)
         << "\",\"safety_reason\":\"" << escape_json(record.safety_reason)
         << "\",\"terminal\":" << (record.terminal ? "true" : "false") << "}";
  return output.str();
}

void write_telemetry(const std::string& path, const std::vector<TelemetryRecord>& records) {
  const std::filesystem::path output_path(path);
  if (!output_path.parent_path().empty()) std::filesystem::create_directories(output_path.parent_path());
  std::ofstream output(path);
  if (!output) throw std::runtime_error("cannot write telemetry: " + path);
  for (const auto& record : records) output << telemetry_json(record) << '\n';
}

std::vector<TelemetryRecord> read_telemetry(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot read telemetry: " + path);
  std::vector<TelemetryRecord> records;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    if (line.front() != '{' || line.back() != '}') throw std::runtime_error("invalid JSONL object");
    TelemetryRecord record;
    record.scenario_id = field_string(line, "scenario_id");
    if (record.scenario_id.empty()) throw std::runtime_error("missing scenario_id");
    record.frame = field_number<std::uint64_t>(line, "frame");
    record.timestamp_ms = field_number<std::int64_t>(line, "timestamp_ms");
    record.frame_valid = field_bool(line, "frame_valid", false, true);
    record.lane_valid = field_bool(line, "lane_valid", false, true);
    record.lane_confidence = field_number<double>(line, "lane_confidence");
    record.lateral_error = field_number<double>(line, "lateral_error");
    record.requested_speed_mps = field_number<double>(line, "requested_speed_mps");
    record.requested_steering_norm = field_number<double>(line, "requested_steering_norm");
    record.requested_brake = field_bool(line, "requested_brake", false, false);
    record.applied_speed_mps = field_number<double>(line, "applied_speed_mps");
    record.applied_steering_norm = field_number<double>(line, "applied_steering_norm");
    record.applied_brake = field_bool(line, "applied_brake", false, false);
    record.perception_latency_ms = field_number<double>(line, "perception_latency_ms");
    const auto state = field_string(line, "safety_state");
    if (state == "DEGRADED") record.safety_state = SafetyState::Degraded;
    else if (state == "SAFE_STOP") record.safety_state = SafetyState::SafeStop;
    else if (state == "LATCHED_STOP") record.safety_state = SafetyState::LatchedStop;
    else if (state != "NORMAL") throw std::runtime_error("invalid safety_state");
    record.safety_reason = field_string(line, "safety_reason");
    if (record.safety_reason.empty()) throw std::runtime_error("missing safety_reason");
    record.terminal = field_bool(line, "terminal", false, false);
    records.push_back(record);
  }
  return records;
}

void write_html_report(const std::string& output_path,
                       const std::vector<TelemetryRecord>& records) {
  const std::filesystem::path path(output_path);
  if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
  std::ofstream output(output_path);
  if (!output) throw std::runtime_error("cannot write report: " + output_path);
  std::size_t degraded = 0;
  std::size_t stops = 0;
  double max_latency = 0.0;
  double sum_error = 0.0;
  for (const auto& record : records) {
    if (record.safety_state == SafetyState::Degraded) ++degraded;
    if (record.safety_state == SafetyState::SafeStop || record.safety_state == SafetyState::LatchedStop) ++stops;
    max_latency = std::max(max_latency, record.perception_latency_ms);
    sum_error += std::abs(record.lateral_error);
  }
  const double mean_error = records.empty() ? 0.0 : sum_error / static_cast<double>(records.size());
  output << "<!doctype html><html lang=\"en\"><meta charset=\"utf-8\"><title>EdgeCar Replay Report</title>"
         << "<style>body{font:16px system-ui;margin:2rem;max-width:1100px;color:#18202a}"
            ".card{display:inline-block;background:#eef3f8;padding:1rem;margin:.3rem;border-radius:8px}"
            "table{border-collapse:collapse;width:100%}td,th{padding:.4rem;border-bottom:1px solid #ddd;text-align:left}"
            ".stop{color:#b42318;font-weight:700}.degraded{color:#b54708;font-weight:700}</style>"
         << "<h1>EdgeCar Replay Lab</h1><p>Deterministic safety and edge-pipeline report.</p>"
         << "<div class=card>Frames <b>" << records.size() << "</b></div>"
         << "<div class=card>Mean |lateral error| <b>" << std::setprecision(4) << mean_error << "</b></div>"
         << "<div class=card>Max perception latency <b>" << max_latency << " ms</b></div>"
         << "<div class=card>Degraded frames <b>" << degraded << "</b></div>"
         << "<div class=card>Stop frames <b>" << stops << "</b></div>"
         << "<h2>Trace</h2><table><tr><th>Frame</th><th>Lane</th><th>Requested speed</th><th>Applied speed</th><th>Brake</th><th>Safety</th><th>Reason</th></tr>";
  for (const auto& record : records) {
    const auto state = to_string(record.safety_state);
    const auto css = record.safety_state == SafetyState::Normal ? "" :
                     (record.safety_state == SafetyState::Degraded ? " class=\"degraded\"" : " class=\"stop\"");
    output << "<tr><td>" << record.frame << "</td><td>" << (record.lane_valid ? "valid" : "lost")
           << "</td><td>" << record.requested_speed_mps << "</td><td>" << record.applied_speed_mps
           << "</td><td>" << (record.applied_brake ? "yes" : "no")
           << "</td><td" << css << ">" << state << "</td><td>" << escape_html(record.safety_reason) << "</td></tr>";
  }
  output << "</table><p><small>Simulation/replay evidence only; no vehicle actuation is performed.</small></p></html>\n";
}

}  // namespace edgecar

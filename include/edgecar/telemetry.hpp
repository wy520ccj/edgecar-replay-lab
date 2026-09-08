#pragma once

#include "edgecar/types.hpp"

#include <iosfwd>
#include <string>
#include <vector>

namespace edgecar {

std::string telemetry_json(const TelemetryRecord& record);
void write_telemetry(const std::string& path,
                     const std::vector<TelemetryRecord>& records);
std::vector<TelemetryRecord> read_telemetry(const std::string& path);
void write_html_report(const std::string& output_path,
                       const std::vector<TelemetryRecord>& records);

}  // namespace edgecar

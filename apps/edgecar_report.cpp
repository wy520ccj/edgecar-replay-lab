#include "edgecar/telemetry.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
  std::string telemetry;
  std::string output = "out/report/index.html";
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--telemetry" && index + 1 < argc) telemetry = argv[++index];
    else if (argument == "--output" && index + 1 < argc) output = argv[++index];
    else if (argument == "--help" || argument == "-h") {
      std::cout << "Usage: edgecar-report --telemetry <jsonl> --output <index.html>\n";
      return 0;
    } else {
      std::cerr << "unknown argument: " << argument << '\n';
      return 2;
    }
  }
  if (telemetry.empty()) {
    std::cerr << "--telemetry is required\n";
    return 2;
  }
  try {
    edgecar::write_html_report(output, edgecar::read_telemetry(telemetry));
    std::cout << "report=" << output << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "edgecar-report: " << error.what() << '\n';
    return 1;
  }
}

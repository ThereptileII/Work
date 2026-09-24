#include "application/MarineMapping.h"
#include "application/Settings.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>
namespace opennav::application {
namespace {
void Require(bool ok, const char *message) {
  if (!ok)
    throw std::invalid_argument(message);
}
bool Supported(vessel::Quantity q) {
  return q == vessel::Quantity::MotorPower ||
         q == vessel::Quantity::ShaftPower ||
         q == vessel::Quantity::MotorTemperature;
}
} // namespace
void ValidateSignalKMappings(const std::vector<SignalKMapping> &mappings) {
  Require(mappings.size() <= 16,
          "At most sixteen explicit propulsion mappings");
  std::set<std::string> paths;
  for (const auto &m : mappings) {
    Require(Supported(m.quantity),
            "Custom mapping only supports motor temperature, motor electrical "
            "power and shaft power");
    Require(m.path.size() > 12 && m.path.size() <= 256 &&
                m.path.rfind("propulsion.", 0) == 0 &&
                std::count(m.path.begin(), m.path.end(), '.') >= 2 &&
                m.path.back() != '.' &&
                m.path.find("..") == std::string::npos &&
                std::all_of(m.path.begin(), m.path.end(),
                            [](unsigned char c) {
                              return (c >= 'a' && c <= 'z') ||
                                     (c >= 'A' && c <= 'Z') ||
                                     (c >= '0' && c <= '9') || c == '.' ||
                                     c == '_' || c == '-';
                            }),
            "Mapping requires a bounded explicit propulsion.instance.path");
    Require(paths.insert(m.path).second, "Duplicate Signal K mapping path");
    Require(std::isfinite(m.scale) && m.scale != 0 &&
                std::abs(m.scale) <= 1e9 && std::isfinite(m.offset) &&
                std::abs(m.offset) <= 1e9,
            "Invalid mapping scale/offset");
    const auto last = m.path.substr(m.path.rfind('.') + 1);
    Require(last != "revolutions" && last != "coolantTemperature",
            "Do not override standard RPM/coolant paths");
  }
}
std::vector<SignalKMapping> ImportSignalKMappings(const std::string &csv) {
  Require(csv.size() <= 16384 && csv.find('\0') == std::string::npos,
          "Mapping CSV exceeds 16 KiB or contains NUL");
  std::istringstream in(csv);
  std::string line;
  auto read = [&] {
    if (!std::getline(in, line))
      return false;
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    return true;
  };
  Require(read() && line == "OpenNavXSignalK,1",
          "Missing OpenNavXSignalK,1 header");
  Require(read() && line == "path,quantity,scale,offset",
          "Expected path,quantity,scale,offset columns");
  std::vector<SignalKMapping> result;
  while (read()) {
    Require(!line.empty(), "Blank mapping row");
    std::vector<std::string> fields;
    std::size_t start = 0;
    for (;;) {
      auto end = line.find(',', start);
      fields.push_back(
          line.substr(start, end == std::string::npos ? end : end - start));
      if (end == std::string::npos)
        break;
      start = end + 1;
    }
    Require(fields.size() == 4, "Mapping row must have four columns");
    auto q =
        std::find_if(vessel::Quantities().begin(), vessel::Quantities().end(),
                     [&](const auto &q) { return fields[1] == q.key; });
    Require(q != vessel::Quantities().end() && Supported(q->quantity),
            "Unsupported custom propulsion quantity");
    result.push_back({fields[0], q->quantity, ParseSettingNumber(fields[2]),
                      ParseSettingNumber(fields[3])});
    Require(result.size() <= 16, "Too many mappings");
  }
  ValidateSignalKMappings(result);
  return result;
}
std::string ExportSignalKMappings(const std::vector<SignalKMapping> &mappings) {
  ValidateSignalKMappings(mappings);
  std::string csv = "OpenNavXSignalK,1\npath,quantity,scale,offset\n";
  for (const auto &m : mappings)
    csv += m.path + "," + vessel::Describe(m.quantity).key + "," +
           SettingNumber(m.scale) + "," + SettingNumber(m.offset) + "\n";
  return csv;
}
} // namespace opennav::application

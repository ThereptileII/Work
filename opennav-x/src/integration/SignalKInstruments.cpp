#include "integration/MarineDecoder.h"
#include "integration/ExternalJson.h"
#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <map>
#include <sstream>
#include "rapidjson/document.h"

namespace opennav::integration {
using vessel::Quantity;
namespace {
const rapidjson::Value &Member(const rapidjson::Value &object, const char *name) {
  static const rapidjson::Value missing;
  if (!object.IsObject()) return missing;
  const auto it = object.FindMember(name);
  return it == object.MemberEnd() ? missing : it->value;
}
std::string Text(const rapidjson::Value &value) {
  return value.IsString() ? std::string(value.GetString(), value.GetStringLength()) : std::string{};
}
struct Mapping {
  Quantity quantity;
  double scale = 1, offset = 0;
  bool estimated = false;
  std::string device;
};
std::optional<std::chrono::system_clock::time_point>
Utc(const std::string &text) {
  if (text.size() < 20 || text.size() > 30 || text.back() != 'Z' ||
      text[4] != '-' || text[7] != '-' || text[10] != 'T' || text[13] != ':' ||
      text[16] != ':')
    return {};
  std::tm fields{};
  std::istringstream stream(text.substr(0, 19));
  stream.imbue(std::locale::classic());
  stream >> std::get_time(&fields, "%Y-%m-%dT%H:%M:%S");
  if (stream.fail() || fields.tm_year < 70 || fields.tm_year > 200 ||
      fields.tm_sec > 59)
    return {};
  long nanoseconds = 0;
  if (text.size() > 20) {
    if (text[19] != '.' || text.size() < 22)
      return {};
    long multiplier = 100000000;
    for (std::size_t i = 20; i + 1 < text.size(); ++i) {
      if (text[i] < '0' || text[i] > '9')
        return {};
      nanoseconds += (text[i] - '0') * multiplier;
      multiplier /= 10;
    }
  }
  const auto original = fields;
#ifdef _WIN32
  const auto seconds = _mkgmtime(&fields);
#else
  const auto seconds = timegm(&fields);
#endif
  if (seconds < 0 || fields.tm_year != original.tm_year ||
      fields.tm_mon != original.tm_mon || fields.tm_mday != original.tm_mday ||
      fields.tm_hour != original.tm_hour || fields.tm_min != original.tm_min ||
      fields.tm_sec != original.tm_sec)
    return {};
  return std::chrono::system_clock::from_time_t(seconds) +
         std::chrono::duration_cast<std::chrono::system_clock::duration>(
             std::chrono::nanoseconds(nanoseconds));
}
std::vector<std::string> Parts(const std::string &path) {
  std::vector<std::string> result;
  std::istringstream s(path);
  std::string part;
  while (std::getline(s, part, '.')) {
    if (part.empty() || part.size() > 80)
      return {};
    result.push_back(part);
  }
  return result;
}
std::optional<Mapping> Map(const std::string &path,
                           const std::vector<SignalKBinding> &bindings) {
  constexpr double degrees = 180.0 / 3.14159265358979323846,
                   knots = 3600.0 / 1852.0;
  static const std::map<std::string, Mapping> fixed = {
      {"navigation.headingTrue", {Quantity::Heading, degrees, 0, true, {}}},
      {"navigation.speedThroughWater", {Quantity::WaterSpeed, knots}},
      {"environment.depth.belowTransducer", {Quantity::Depth}},
      {"environment.water.temperature",
       {Quantity::WaterTemperature, 1, -273.15}},
      {"environment.outside.pressure", {Quantity::Pressure, .01}},
      {"environment.wind.speedApparent", {Quantity::ApparentWindSpeed, knots}},
      {"environment.wind.angleApparent",
       {Quantity::ApparentWindAngle, degrees}},
      {"environment.wind.speedTrue",
       {Quantity::TrueWindSpeed, knots, 0, true, {}}},
      {"environment.wind.angleTrueWater",
       {Quantity::TrueWindAngle, degrees, 0, true, {}}},
      {"steering.rudderAngle", {Quantity::Rudder, degrees}}};
  auto it = fixed.find(path);
  if (it != fixed.end())
    return it->second;
  const auto p = Parts(path);
  if (p.size() >= 4 && p[0] == "electrical" && p[1] == "batteries") {
    Mapping m;
    m.device = "electrical.batteries." + p[2];
    if (p.size() == 4 && p[3] == "voltage")
      m.quantity = Quantity::BatteryVoltage;
    else if (p.size() == 4 && p[3] == "current")
      m.quantity = Quantity::BatteryNativeCurrent;
    else if (p.size() == 5 && p[3] == "capacity" &&
             (p[4] == "stateOfCharge" || p[4] == "stateOfHealth")) {
      m.quantity =
          p[4] == "stateOfCharge" ? Quantity::BatterySoc : Quantity::BatterySoh;
      m.scale = 100;
    } else
      return {};
    return m;
  }
  if (p.size() == 3 && p[0] == "propulsion") {
    Mapping m;
    m.device = "propulsion." + p[1];
    if (p[2] == "revolutions") {
      m.quantity = Quantity::MotorRpm;
      m.scale = 60;
      return m;
    }
    if (p[2] == "coolantTemperature") {
      m.quantity = Quantity::CoolantTemperature;
      m.offset = -273.15;
      return m;
    }
  }
  if (p.size() == 4 && p[0] == "tanks" && p[3] == "currentLevel") {
    Mapping m;
    m.scale = 100;
    m.device = "tanks." + p[1] + "." + p[2];
    if (p[1] == "freshWater")
      m.quantity = Quantity::FreshWater;
    else if (p[1] == "fuel")
      m.quantity = Quantity::Fuel;
    else if (p[1] == "wasteWater" || p[1] == "blackWater")
      m.quantity = Quantity::Waste;
    else
      return {};
    return m;
  }
  for (const auto &b : bindings)
    if (b.path == path && b.path.size() <= 256 && !p.empty() &&
        std::isfinite(b.scale) && b.scale != 0 && std::abs(b.scale) <= 1e9 &&
        std::isfinite(b.offset) && std::abs(b.offset) <= 1e9 &&
        static_cast<unsigned>(b.quantity) <
            static_cast<unsigned>(Quantity::Count)) {
      return Mapping{b.quantity, b.scale, b.offset, false,
                     "configured/" + path};
    }
  return {};
}
} // namespace
std::vector<vessel::SensorObservation>
DecodeSignalKInstruments(const std::string &json, const std::string &self,
                         const std::string &iface, vessel::Time received,
                         std::chrono::system_clock::time_point wall,
                         const std::vector<SignalKBinding> &bindings) {
  std::vector<vessel::SensorObservation> result;
  if (!BoundedJsonText(json) || self.empty() || self == "vessels.self" ||
      iface.empty() || iface.size() > 200 || bindings.size() > 64)
    return result;
  rapidjson::Document root;
  root.Parse<rapidjson::kParseValidateEncodingFlag>(json.data(), json.size());
  if (root.HasParseError() || !root.IsObject() ||
      Text(Member(root, "context")) != self || !Member(root, "updates").IsArray())
    return result;
  const auto &updates = Member(root, "updates");
  if (updates.Size() > 512)
    return result;
  for (const auto &update : updates.GetArray()) {
    if (!update.IsObject() || !Member(update, "timestamp").IsString() ||
        !Member(update, "values").IsArray())
      continue;
    const auto time =
        Utc(Text(Member(update, "timestamp")));
    if (!time || *time > wall || wall - *time > std::chrono::hours(24))
      continue;
    const auto at =
        received -
        std::chrono::duration_cast<vessel::Clock::duration>(wall - *time);
    std::string source = Text(Member(update, "$source"));
    const auto &origin = Member(update, "source");
    if (source.empty() && origin.IsObject()) {
      source = Text(Member(origin, "label"));
      const auto &instance = Member(origin, "src");
      if (!source.empty() && instance.IsString()) source += "/" + Text(instance);
      else if (!source.empty() && instance.IsInt()) source += "/" + std::to_string(instance.GetInt());
    }
    if (source.empty() || source.size() > 120)
      continue;
    const auto &values = Member(update, "values");
    if (values.Size() > 256)
      continue;
    for (rapidjson::SizeType j = 0; j < values.Size() && result.size() < 1024; ++j) {
      auto &item = values[j];
      if (!item.IsObject() || !Member(item, "path").IsString())
        continue;
      const auto path = Text(Member(item, "path"));
      auto map = Map(path, bindings);
      const bool attitude = path == "navigation.attitude";
      if (attitude)
        map = Mapping{Quantity::Heel, 180.0 / 3.14159265358979323846};
      if (!map)
        continue;
      const auto identity = "SignalK/" + iface + "/" + source;
      const auto provenance = identity + "/" + path;
      vessel::Sample sample{{}, provenance, at, vessel::Validity::Invalid};
      sample.device_id =
          identity + "/" + (map->device.empty() ? "vessel" : map->device);
      const auto &value = attitude ? Member(Member(item, "value"), "roll") : Member(item, "value");
      // JSON null, booleans and numeric-looking strings never become zero.
      if (value.IsNumber()) {
        const double raw = value.GetDouble();
        double number = raw * map->scale + map->offset;
        if (std::isfinite(number)) {
          sample.value = number;
          sample.validity = map->estimated ? vessel::Validity::Estimated
                                           : vessel::Validity::Measured;
        }
      }
      result.push_back({map->quantity, provenance, std::move(sample), 30});
    }
  }
  return result;
}
} // namespace opennav::integration

#include "application/Settings.h"
#include <cmath>
#include <iomanip>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>
namespace opennav::application {
namespace {
void Require(bool ok, const std::string &message) {
  if (!ok)
    throw std::invalid_argument(message);
}
void Range(double n, double lo, double hi, const char *name) {
  Require(std::isnan(n) || (std::isfinite(n) && n >= lo && n <= hi),
          std::string("Invalid ") + name);
}
using Record = std::map<std::string, std::string>;
void Size(const std::string &s, std::size_t max, const char *name) {
  Require(s.size() <= max && s.find('\0') == std::string::npos,
          std::string("Invalid ") + name);
}
} // namespace
double ParseSettingNumber(const std::string &s) {
  if (s.empty())
    return std::numeric_limits<double>::quiet_NaN();
  Size(s, 64, "number");
  std::istringstream in(s);
  in.imbue(std::locale::classic());
  double v;
  Require(bool(in >> v) && (in >> std::ws).eof() && std::isfinite(v),
          "Use a finite decimal number with a dot, or leave unconfigured");
  return v;
}
std::string SettingNumber(double n) {
  if (std::isnan(n))
    return {};
  Require(std::isfinite(n), "Infinite configuration value");
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17) << n;
  return out.str();
}
void ValidateSettings(const Settings &s) {
  ValidateSignalKMappings(s.signal_k_mappings);
  const auto &e = s.energy;
  Range(e.battery.capacity_kwh, .001, 100000, "usable capacity (kWh)");
  Range(e.battery.reserve_soc_percent, 0, 100, "reserve SOC (%)");
  Require(std::isfinite(e.battery.minimum_speed_kn) &&
              e.battery.minimum_speed_kn >= .1 &&
              e.battery.minimum_speed_kn <= 20,
          "Minimum passage speed must be 0.1..20 kn");
  Range(e.hotel_kw, 0, 10000, "hotel load (kW)");
  Range(e.shaft_efficiency, .001, 1, "shaft efficiency (0..1)");
  Size(e.battery_device_id, 1024, "battery identity");
  Size(e.battery.source, 1024, "model provenance");
  Require(e.consumption == smartnav::ConsumptionModel::MeasuredPack ||
              e.consumption == smartnav::ConsumptionModel::CalibratedCurve,
          "Unknown consumption model");
  if (!e.curve.points.empty() ||
      e.consumption == smartnav::ConsumptionModel::CalibratedCurve)
    Require(smartnav::ValidPowerCurve(e.curve),
            "A valid imported calibration is required");
  Require(s.current == vessel::CurrentConvention::Unconfigured ||
              s.current == vessel::CurrentConvention::PositiveDischarge ||
              s.current == vessel::CurrentConvention::PositiveCharge,
          "Unknown current convention");
  Range(s.hazard.draft_m, 0, 100, "draft (m)");
  Range(s.hazard.safety_margin_m, 0, 100, "safety margin (m)");
  Range(s.hazard.corridor_half_width_m, 1, 10000, "corridor half width (m)");
  for (const auto &p : s.sources) {
    Require(p.first >= vessel::Quantity::Heading &&
                p.first < vessel::Quantity::Count,
            "Unknown source quantity");
    Size(p.second.pinned_source, 512, "source identity");
    const auto &f = p.second.freshness;
    Require(f.aging_after.count() > 0 && f.stale_after > f.aging_after &&
                f.stale_after <= vessel::Duration{300000},
            "Source ages must satisfy 0 < aging < stale <= 300 s");
    vessel::SensorRegistry verifier;
    verifier.Configure(p.first, p.second); // Share the live reducer's limits.
  }
}
std::string EncodeSettings(const Settings &s) {
  ValidateSettings(s);
  const auto &e = s.energy;
  Record r{
      {"capacity", SettingNumber(e.battery.capacity_kwh)},
      {"reserve", SettingNumber(e.battery.reserve_soc_percent)},
      {"minimum_speed", SettingNumber(e.battery.minimum_speed_kn)},
      {"model_source", e.battery.source},
      {"battery", e.battery_device_id},
      {"consumption", e.consumption == smartnav::ConsumptionModel::MeasuredPack
                          ? "measured"
                          : "curve"},
      {"current", s.current == vessel::CurrentConvention::PositiveDischarge
                      ? "discharge"
                  : s.current == vessel::CurrentConvention::PositiveCharge
                      ? "charge"
                      : "unconfigured"},
      {"hotel", SettingNumber(e.hotel_kw)},
      {"efficiency", SettingNumber(e.shaft_efficiency)},
      {"draft", SettingNumber(s.hazard.draft_m)},
      {"margin", SettingNumber(s.hazard.safety_margin_m)},
      {"corridor", SettingNumber(s.hazard.corridor_half_width_m)}};
  if (!s.signal_k_mappings.empty())
    r["signal_k_mappings"] = ExportSignalKMappings(s.signal_k_mappings);
  if (!e.curve.points.empty()) {
    r["curve"] = smartnav::ExportPowerCurve(e.curve);
    r["curve_source"] = e.curve.source;
  }
  for (const auto &p : s.sources) {
    const std::string key =
        std::string("source.") + vessel::Describe(p.first).key;
    r[key] = p.second.pinned_source;
    r[key + ".aging"] = std::to_string(p.second.freshness.aging_after.count());
    r[key + ".stale"] = std::to_string(p.second.freshness.stale_after.count());
  }
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << "OpenNavXSettings 1\n";
  for (const auto &p : r)
    out << std::quoted(p.first) << ' ' << std::quoted(p.second) << '\n';
  Require(out.str().size() <= 65536, "Settings record exceeds 64 KiB");
  return out.str();
}
Settings DecodeSettings(const std::string &record) {
  Require(record.size() <= 65536 && record.find('\0') == std::string::npos,
          "Invalid settings record size/content");
  std::istringstream in(record);
  in.imbue(std::locale::classic());
  std::string header;
  std::getline(in, header);
  Require(header == "OpenNavXSettings 1", "Unsupported settings version");
  Record r;
  while (!(in >> std::ws).eof()) {
    std::string k, v;
    Require(in.peek() == '"' && bool(in >> std::quoted(k)),
            "Invalid settings key");
    in >> std::ws;
    Require(in.peek() == '"' && bool(in >> std::quoted(v)),
            "Invalid settings value");
    Size(k, 128, "settings key");
    Size(v, 32768, "settings value");
    Require(r.emplace(k, v).second && r.size() <= 128,
            "Duplicate/excess settings fields");
  }
  auto take = [&](const std::string &k) {
    const auto it = r.find(k);
    Require(it != r.end(), "Missing settings field: " + k);
    auto v = it->second;
    r.erase(it);
    return v;
  };
  Settings s;
  auto &e = s.energy;
  e.battery.capacity_kwh = ParseSettingNumber(take("capacity"));
  e.battery.reserve_soc_percent = ParseSettingNumber(take("reserve"));
  e.battery.minimum_speed_kn = ParseSettingNumber(take("minimum_speed"));
  e.battery.source = take("model_source");
  e.battery_device_id = take("battery");
  auto model = take("consumption");
  Require(model == "measured" || model == "curve", "Unknown consumption model");
  e.consumption = model == "measured"
                      ? smartnav::ConsumptionModel::MeasuredPack
                      : smartnav::ConsumptionModel::CalibratedCurve;
  const auto current = take("current");
  Require(current == "unconfigured" || current == "charge" ||
              current == "discharge",
          "Unknown current sign convention");
  s.current = current == "charge" ? vessel::CurrentConvention::PositiveCharge
              : current == "discharge"
                  ? vessel::CurrentConvention::PositiveDischarge
                  : vessel::CurrentConvention::Unconfigured;
  e.hotel_kw = ParseSettingNumber(take("hotel"));
  e.shaft_efficiency = ParseSettingNumber(take("efficiency"));
  s.hazard.draft_m = ParseSettingNumber(take("draft"));
  s.hazard.safety_margin_m = ParseSettingNumber(take("margin"));
  s.hazard.corridor_half_width_m = ParseSettingNumber(take("corridor"));
  if (r.count("curve")) {
    const auto csv = take("curve"), source = take("curve_source");
    e.curve = smartnav::ImportPowerCurve(csv, source);
  }
  for (const auto &q : vessel::Quantities()) {
    const auto key = std::string("source.") + q.key;
    if (!r.count(key))
      continue;
    vessel::SourcePolicy p;
    p.pinned_source = take(key);
    auto duration = [&](const std::string &k) {
      const auto d = ParseSettingNumber(take(k));
      Require(std::isfinite(d) && d >= 0 && d <= 300000 && std::floor(d) == d,
              "Invalid source duration");
      return vessel::Duration{static_cast<long long>(d)};
    };
    p.freshness = {duration(key + ".aging"), duration(key + ".stale")};
    s.sources[q.quantity] = p;
  }
  if (r.count("signal_k_mappings"))
    s.signal_k_mappings = ImportSignalKMappings(take("signal_k_mappings"));
  Require(r.empty(), "Unknown settings fields");
  ValidateSettings(s);
  return s;
}
} // namespace opennav::application

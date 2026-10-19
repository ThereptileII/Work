#include "cloud/CloudProtocol.h"
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <cmath>
#include <functional>

namespace opennav::cloud {
namespace {
using vessel::VesselState;
using Writer = rapidjson::Writer<rapidjson::StringBuffer>;

const char *WireQuality(vessel::Quality q) {
  switch (q) {
  case vessel::Quality::Live: return "live";
  case vessel::Quality::Aging: return "aging";
  case vessel::Quality::Stale: return "stale";
  case vessel::Quality::Estimated: return "estimated";
  case vessel::Quality::Uncertain: return "uncertain";
  case vessel::Quality::Unavailable: break;
  }
  return nullptr;
}

std::string CleanSource(const std::string &source) {
  std::string out;
  for (const char c : source) {
    if (out.size() >= 64) break;
    const auto u = static_cast<unsigned char>(c);
    // ASCII printable only: the server refuses control characters, and a
    // truncated multibyte sequence would be invalid UTF-8.
    if (u >= 0x20 && u < 0x7f) out += c;
  }
  return out;
}

bool Parse(const std::string &body, rapidjson::Document &doc) {
  if (body.empty() || body.size() > 16384) return false;
  doc.Parse(body.c_str(), body.size());
  return !doc.HasParseError() && doc.IsObject();
}
std::string String(const rapidjson::Value &object, const char *name, std::size_t max = 256) {
  const auto it = object.FindMember(name);
  if (it == object.MemberEnd() || !it->value.IsString() || it->value.GetStringLength() > max) return {};
  return {it->value.GetString(), it->value.GetStringLength()};
}
std::string Json(const std::function<void(Writer &)> &fill) {
  rapidjson::StringBuffer buffer;
  Writer writer(buffer);
  writer.StartObject();
  fill(writer);
  writer.EndObject();
  return {buffer.GetString(), buffer.GetSize()};
}
}  // namespace

bool ValidToken(std::string_view token) {
  if (token.size() != 69 || token.substr(0, 5) != "skg1_") return false;
  for (const char c : token.substr(5))
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  return true;
}

std::string FormatUserCode(const std::string &code) {
  return code.size() == 8 ? code.substr(0, 4) + "-" + code.substr(4) : code;
}

const std::vector<TelemetryKey> &TelemetryKeys() {
#define SKAGER_KEY(name, lo, hi, member) \
  TelemetryKey{name, lo, hi, [](const VesselState &s) -> const vessel::Sample & { return s.member; }}
  static const std::vector<TelemetryKey> keys{
      SKAGER_KEY("navigation.latitude", -90, 90, navigation.latitude_deg),
      SKAGER_KEY("navigation.longitude", -180, 180, navigation.longitude_deg),
      SKAGER_KEY("navigation.sog", 0, 100, navigation.sog_kn),
      SKAGER_KEY("navigation.cog", 0, 360, navigation.cog_deg),
      SKAGER_KEY("navigation.heading", 0, 360, navigation.heading_true_deg),
      SKAGER_KEY("navigation.stw", 0, 100, navigation.stw_kn),
      SKAGER_KEY("wind.apparent_speed", 0, 200, wind.apparent_speed_kn),
      SKAGER_KEY("wind.apparent_angle", -180, 360, wind.apparent_angle_deg),
      SKAGER_KEY("wind.true_speed", 0, 200, wind.true_speed_kn),
      SKAGER_KEY("wind.true_angle", -180, 360, wind.true_angle_deg),
      SKAGER_KEY("environment.depth", 0, 12000, environment.depth_below_transducer_m),
      SKAGER_KEY("environment.water_temperature", -5, 50, environment.water_temperature_c),
      SKAGER_KEY("environment.pressure", 800, 1100, environment.pressure_hpa),
      SKAGER_KEY("steering.rudder", -90, 90, rudder.angle_deg),
      SKAGER_KEY("steering.heel", -90, 90, rudder.heel_deg),
      SKAGER_KEY("propulsion.electrical_power", -1000, 1000, propulsion.electrical_power_kw),
      SKAGER_KEY("propulsion.shaft_power", -1000, 1000, propulsion.shaft_power_kw),
      SKAGER_KEY("propulsion.rpm", -20000, 20000, propulsion.motor_rpm),
      SKAGER_KEY("propulsion.gear", 0, 2, propulsion.gear_code),
      SKAGER_KEY("propulsion.regeneration", 0, 2, propulsion.regeneration_code),
      SKAGER_KEY("propulsion.motor_temperature", -50, 250, propulsion.motor_temperature_c),
      SKAGER_KEY("propulsion.coolant_temperature", -50, 250, propulsion.coolant_temperature_c),
      SKAGER_KEY("battery.voltage", 0, 1500, battery.voltage_v),
      // Only the explicitly mapped "+ discharge" current; never the raw source sign.
      SKAGER_KEY("battery.current", -5000, 5000, battery.current_a),
      SKAGER_KEY("battery.soc", 0, 100, battery.soc_percent),
      SKAGER_KEY("battery.soh", 0, 100, battery.soh_percent),
      SKAGER_KEY("battery.capacity", 0, 10000, battery.usable_capacity_kwh),
      SKAGER_KEY("battery.net_discharge", -1000, 1000, battery.net_discharge_kw),
      SKAGER_KEY("tanks.fresh_water", 0, 100, tanks.fresh_water_percent),
      SKAGER_KEY("tanks.fuel", 0, 100, tanks.fuel_percent),
      SKAGER_KEY("tanks.waste", 0, 100, tanks.waste_percent),
      SKAGER_KEY("tanks.other", 0, 100, tanks.other_percent),
      SKAGER_KEY("connectivity.input_rate", 0, 100000, connectivity.received_messages_per_second),
  };
#undef SKAGER_KEY
  return keys;
}

std::optional<std::string> EncodeTelemetry(const VesselState &state, vessel::Time now) {
  if (state.simulated || state.replayed) return {};
  rapidjson::StringBuffer buffer;
  Writer writer(buffer);
  writer.StartObject();
  writer.Key("items");
  writer.StartObject();
  int count = 0;
  for (const auto &key : TelemetryKeys()) {
    const auto &sample = key.sample(state);
    const auto assessed = vessel::Assess(sample, now);
    const char *quality = WireQuality(assessed.quality);
    if (!quality || !assessed.value || !assessed.age) continue;
    const double value = *assessed.value;
    const double age = std::chrono::duration<double>(*assessed.age).count();
    if (!std::isfinite(value) || value < key.min || value > key.max || age < 0 || age > 86400) continue;
    writer.Key(key.key);
    writer.StartObject();
    writer.Key("v");
    writer.Double(std::round(value * 1e6) / 1e6);
    writer.Key("age");
    writer.Double(std::round(age * 10) / 10);
    writer.Key("q");
    writer.String(quality);
    const auto source = CleanSource(sample.source);
    if (!source.empty()) {
      writer.Key("src");
      writer.String(source.c_str(), static_cast<rapidjson::SizeType>(source.size()));
    }
    writer.EndObject();
    ++count;
  }
  writer.EndObject();
  writer.EndObject();
  if (!count) return {};
  return std::string(buffer.GetString(), buffer.GetSize());
}

std::string EncodeLinkRequest(const std::string &device_name, const std::string &platform,
                              const std::string &version) {
  std::string name = CleanSource(device_name).substr(0, 60);
  if (name.empty() || name.find_first_not_of(' ') == std::string::npos) name = "SKAGER";
  // The server accepts [0-9A-Za-z][0-9A-Za-z.+-]{0,39}; anything else is dropped.
  std::string safe_version;
  for (const char c : version) {
    const bool alnum = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
    if (safe_version.size() < 40 && (alnum || (!safe_version.empty() && (c == '.' || c == '+' || c == '-'))))
      safe_version += c;
  }
  if (safe_version.empty()) safe_version = "0";
  return Json([&](Writer &w) {
    w.Key("name"); w.String(name.c_str());
    w.Key("platform"); w.String(platform.c_str());
    w.Key("version"); w.String(safe_version.c_str());
  });
}

std::string EncodePoll(const std::string &device_code) {
  return Json([&](Writer &w) { w.Key("deviceCode"); w.String(device_code.c_str()); });
}

std::optional<LinkOffer> ParseLinkOffer(const std::string &body) {
  rapidjson::Document doc;
  if (!Parse(body, doc)) return {};
  LinkOffer offer;
  offer.device_code = String(doc, "deviceCode", 64);
  offer.user_code = String(doc, "userCode", 8);
  offer.verification_url = String(doc, "verificationUrl", 200);
  if (offer.device_code.size() != 64 || offer.user_code.size() != 8 ||
      offer.verification_url.rfind(std::string(kOrigin) + "/", 0) != 0)
    return {};
  const auto number = [&doc](const char *name, int lo, int hi, int fallback) {
    const auto it = doc.FindMember(name);
    if (it == doc.MemberEnd() || !it->value.IsInt()) return fallback;
    const int v = it->value.GetInt();
    return v < lo ? lo : v > hi ? hi : v;
  };
  offer.expires_in = std::chrono::seconds(number("expiresIn", 60, 1800, 600));
  offer.interval = std::chrono::seconds(number("interval", 2, 60, 5));
  return offer;
}

PollResult ParsePoll(const std::string &body) {
  rapidjson::Document doc;
  if (!Parse(body, doc)) return {};
  const auto state = String(doc, "state", 20);
  if (state == "pending") return {PollState::Pending, {}};
  if (state == "denied") return {PollState::Denied, {}};
  if (state == "expired") return {PollState::Expired, {}};
  if (state == "slow_down") return {PollState::SlowDown, {}};
  if (state == "approved") {
    auto token = String(doc, "token", 69);
    if (ValidToken(token)) return {PollState::Approved, std::move(token)};
  }
  return {};
}

std::optional<AccountStatus> ParseStatus(const std::string &body) {
  rapidjson::Document doc;
  if (!Parse(body, doc)) return {};
  const auto account = doc.FindMember("account");
  const auto device = doc.FindMember("device");
  const auto cloud = doc.FindMember("cloud");
  if (account == doc.MemberEnd() || !account->value.IsObject() || cloud == doc.MemberEnd() ||
      !cloud->value.IsObject())
    return {};
  AccountStatus status;
  status.email = String(account->value, "email", 254);
  if (status.email.empty()) return {};
  if (device != doc.MemberEnd() && device->value.IsObject())
    status.device_name = String(device->value, "name", 60);
  const auto active = cloud->value.FindMember("active");
  status.cloud_active = active != cloud->value.MemberEnd() && active->value.IsBool() && active->value.GetBool();
  status.cloud_status = String(cloud->value, "status", 40);
  status.renews_at = String(cloud->value, "renewsAt", 40);
  const auto cancel = cloud->value.FindMember("cancelAtPeriodEnd");
  status.cancel_at_period_end = cancel != cloud->value.MemberEnd() && cancel->value.IsBool() && cancel->value.GetBool();
  return status;
}
}  // namespace opennav::cloud

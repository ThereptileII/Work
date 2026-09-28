#include "ais/AisStreamCodec.h"
#include "integration/ExternalJson.h"
#include <cmath>
#include <ctime>
#include <iomanip>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <set>
#include <sstream>

namespace opennav::ais {
namespace {
using Value = rapidjson::Value;
using Wall = std::chrono::system_clock;
using namespace std::chrono_literals;
const Value &Member(const Value &v, const char *key) {
  static const Value empty;
  if (!v.IsObject())
    return empty;
  const auto it = v.FindMember(key);
  return it == v.MemberEnd() ? empty : it->value;
}
bool Structure(const Value &v) {
  if (v.IsObject()) {
    if (v.MemberCount() > 64)
      return false;
    std::set<std::string> names;
    for (auto it = v.MemberBegin(); it != v.MemberEnd(); ++it)
      if (it->name.GetStringLength() > 128 ||
          !names.emplace(it->name.GetString(), it->name.GetStringLength())
               .second ||
          !Structure(it->value))
        return false;
  } else if (v.IsArray()) {
    if (v.Size() > 256)
      return false;
    for (const auto &item : v.GetArray())
      if (!Structure(item))
        return false;
  }
  return true;
}
bool Id(const Value &v) {
  return v.IsInt() && v.GetInt() >= 100000000 && v.GetInt() <= 999999999;
}
bool Range(const Value &v, double low, double high) {
  return v.IsNumber() && std::isfinite(v.GetDouble()) && v.GetDouble() >= low &&
         v.GetDouble() <= high;
}
bool OptionalNumber(const Value &object, const char *key, double low,
                    double high, double unavailable,
                    std::optional<double> &out) {
  const auto &v = Member(object, key);
  if (v.IsNull())
    return true;
  if (!v.IsNumber() || !std::isfinite(v.GetDouble()))
    return false;
  if (v.GetDouble() == unavailable)
    return true;
  if (!Range(v, low, high))
    return false;
  out = v.GetDouble();
  return true;
}
bool Text(const Value &o, const char *key, std::size_t limit,
          std::optional<std::string> &out) {
  const auto &v = Member(o, key);
  if (v.IsNull())
    return true;
  if (!v.IsString() || v.GetStringLength() > limit)
    return false;
  std::string s(v.GetString(), v.GetStringLength());
  for (unsigned char c : s)
    if (c < 32 || c == 127)
      return false;
  // AIS @/space padding is not a vessel name or callsign.
  while (!s.empty() && (s.back() == ' ' || s.back() == '@'))
    s.pop_back();
  while (!s.empty() && s.front() == ' ')
    s.erase(s.begin());
  if (!s.empty())
    out = std::move(s);
  return true;
}
std::optional<Wall::time_point> Utc(const Value &v) {
  if (!v.IsString() || v.GetStringLength() < 20 || v.GetStringLength() > 44)
    return {};
  std::string text(v.GetString(), v.GetStringLength());
  if (text[4] != '-' || text[7] != '-' ||
      (text[10] != 'T' && text[10] != ' ') || text[13] != ':' ||
      text[16] != ':')
    return {};
  for (unsigned i : {0, 1, 2, 3, 5, 6, 8, 9, 11, 12, 14, 15, 17, 18})
    if (text[i] < '0' || text[i] > '9')
      return {};
  text[10] = 'T';
  std::tm tm{};
  std::istringstream in(text.substr(0, 19));
  in.imbue(std::locale::classic());
  in >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
  if (in.fail() || tm.tm_year < 70 || tm.tm_year > 199 || tm.tm_sec > 59)
    return {};
  std::size_t end = 19;
  long ns = 0, multiplier = 100000000;
  if (text[end] == '.') {
    const auto first = ++end;
    while (end < text.size() && text[end] >= '0' && text[end] <= '9') {
      if (end - first == 9)
        return {};
      ns += (text[end++] - '0') * multiplier;
      multiplier /= 10;
    }
    if (end == first)
      return {};
  }
  if (text.substr(end) != "Z" && text.substr(end) != " +0000 UTC")
    return {};
  const auto original = tm;
#ifdef _WIN32
  const auto seconds = _mkgmtime(&tm);
#else
  const auto seconds = timegm(&tm);
#endif
  if (seconds < 0 || tm.tm_year != original.tm_year ||
      tm.tm_mon != original.tm_mon || tm.tm_mday != original.tm_mday ||
      tm.tm_hour != original.tm_hour || tm.tm_min != original.tm_min ||
      tm.tm_sec != original.tm_sec)
    return {};
  return Wall::from_time_t(seconds) +
         std::chrono::duration_cast<Wall::duration>(
             std::chrono::nanoseconds(ns));
}
bool StaticFields(const Value &v, StaticReport &r, const char *type = "Type") {
  if (!Text(v, "Name", 128, r.name) || !Text(v, "CallSign", 32, r.callsign) ||
      !Text(v, "Destination", 128, r.destination) ||
      !OptionalNumber(v, type, 1, 99, 0, r.ship_type))
    return false;
  const auto &d = Member(v, "Dimension");
  if (d.IsNull())
    return true;
  if (!d.IsObject())
    return false;
  const auto &a = Member(d, "A"), &b = Member(d, "B"), &c = Member(d, "C"),
             &e = Member(d, "D");
  if (!Range(a, 0, 511) || !Range(b, 0, 511) || !Range(c, 0, 63) ||
      !Range(e, 0, 63) || !a.IsInt() || !b.IsInt() || !c.IsInt() || !e.IsInt())
    return false;
  // A zero component means unspecified, not a valid zero-length hull.
  if (a.GetInt() && b.GetInt())
    r.length_m = a.GetInt() + b.GetInt();
  if (c.GetInt() && e.GetInt())
    r.beam_m = c.GetInt() + e.GetInt();
  return true;
}
} // namespace
DecodedMessage DecodeAisStream(const std::string &json, vessel::Time received,
                               Wall::time_point wall_now) {
  DecodedMessage result;
  if (json.size() > 65536 || !integration::BoundedJsonText(json) ||
      received == vessel::Time{})
    return result;
  rapidjson::Document root;
  root.Parse<rapidjson::kParseValidateEncodingFlag>(json.data(), json.size());
  if (root.HasParseError() || !root.IsObject() || !Structure(root))
    return result;
  // Never retain server error text: it can echo credentials/subscription data.
  if (root.HasMember("error") || root.HasMember("Error")) {
    result.kind = DecodeKind::ServiceError;
    return result;
  }
  const auto &type = Member(root, "MessageType"),
             &message = Member(root, "Message");
  if (!type.IsString() || type.GetStringLength() > 64 || !message.IsObject())
    return result;
  const std::string name(type.GetString(), type.GetStringLength());
  if (name == "SubscriptionConfirmation") {
    const auto &compression = Member(message, "CompressionEnabled");
    if (!compression.IsBool())
      return result;
    result.kind = DecodeKind::Confirmation;
    result.compression = compression.GetBool();
    return result;
  }
  const bool position = name == "PositionReport" ||
                        name == "StandardClassBPositionReport" ||
                        name == "ExtendedClassBPositionReport";
  if (!position && name != "ShipStaticData" && name != "StaticDataReport") {
    result.kind = DecodeKind::Ignored;
    return result;
  }
  const auto &meta = Member(root, "MetaData"),
             &v = Member(message, name.c_str());
  const auto &id = Member(v, "UserID"), &meta_id = Member(meta, "MMSI"),
             &valid = Member(v, "Valid");
  if (!Id(id) || !Id(meta_id) || id.GetInt() != meta_id.GetInt() ||
      !valid.IsBool() || !valid.GetBool())
    return result;
  auto observed = received;
  const auto &time = Member(meta, "time_utc");
  const bool receipt_only = time.IsNull();
  if (!receipt_only) {
    const auto stamp = Utc(time);
    // Validate wall clock range before any potentially overflowing subtraction.
    if (!stamp || wall_now < Wall::time_point{} ||
        wall_now > Wall::from_time_t(4102444800LL) || *stamp > wall_now ||
        *stamp < wall_now - 10min || received < vessel::Time::min() + 10min)
      return result;
    observed -=
        std::chrono::duration_cast<vessel::Clock::duration>(wall_now - *stamp);
  }
  PositionReport p;
  StaticReport s;
  p.mmsi = s.mmsi = id.GetInt();
  p.observed_at = s.observed_at = observed;
  // This is service/receipt time, never a claimed transponder generation time.
  p.receipt_time_only = receipt_only;
  if (position) {
    const auto &lat = Member(v, "Latitude"), &lon = Member(v, "Longitude");
    // Metadata position may be cached. Only typed position reports own motion.
    if (!Range(lat, -90, 90) || !Range(lon, -180, 180) ||
        !OptionalNumber(v, "Sog", 0, 102.2, 102.3, p.sog) ||
        !OptionalNumber(v, "Cog", 0, 359.999999, 360, p.cog) ||
        !OptionalNumber(v, "TrueHeading", 0, 359, 511, p.heading) ||
        !OptionalNumber(v, "NavigationalStatus", 0, 14, 15,
                        p.navigation_status))
      return result;
    p.latitude = lat.GetDouble();
    p.longitude = lon.GetDouble();
    result.position = p;
  }
  if (name == "ExtendedClassBPositionReport" || name == "ShipStaticData") {
    if (!StaticFields(v, s))
      return {};
    result.static_data = s;
  } else if (name == "StaticDataReport") {
    const auto &part = Member(v, "PartNumber");
    if (!part.IsBool())
      return {};
    const auto &report = Member(v, part.GetBool() ? "ReportB" : "ReportA");
    const auto &good = Member(report, "Valid");
    if (!good.IsBool() || !good.GetBool())
      return {};
    if (!StaticFields(report, s, "ShipType"))
      return {};
    result.static_data = s;
  }
  result.kind = DecodeKind::Reports;
  return result;
}
std::optional<std::string>
AisStreamSubscription(const std::string &key,
                      const std::vector<BoundingBox> &boxes,
                      const std::vector<int> &mmsis) {
  if (key.empty() || key.size() > 512 || boxes.empty() || boxes.size() > 2 ||
      mmsis.size() > 200)
    return {};
  for (unsigned char c : key)
    if (c < 33 || c > 126)
      return {};
  for (const auto &b : boxes)
    if (!std::isfinite(b.south) || !std::isfinite(b.north) ||
        !std::isfinite(b.west) || !std::isfinite(b.east) || b.south < -90 ||
        b.north > 90 || b.south >= b.north || b.west < -180 || b.east > 180 ||
        b.west >= b.east)
      return {};
  std::set<int> ids;
  for (int id : mmsis)
    if (id < 100000000 || id > 999999999 || !ids.insert(id).second)
      return {};
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> w(buffer);
  w.StartObject();
  w.Key("APIKey");
  w.String(key.data(), static_cast<rapidjson::SizeType>(key.size()));
  w.Key("BoundingBoxes");
  w.StartArray();
  for (const auto &b : boxes) {
    w.StartArray();
    w.StartArray();
    w.Double(b.south);
    w.Double(b.west);
    w.EndArray();
    w.StartArray();
    w.Double(b.north);
    w.Double(b.east);
    w.EndArray();
    w.EndArray();
  }
  w.EndArray();
  if (!mmsis.empty()) {
    w.Key("FiltersShipMMSI");
    w.StartArray();
    for (int id : mmsis)
      w.String(std::to_string(id).c_str());
    w.EndArray();
  }
  w.Key("FilterMessageTypes");
  w.StartArray();
  for (const char *type :
       {"PositionReport", "StandardClassBPositionReport",
        "ExtendedClassBPositionReport", "ShipStaticData", "StaticDataReport"})
    w.String(type);
  w.EndArray();
  w.EndObject();
  return std::string(buffer.GetString(), buffer.GetSize());
}
} // namespace opennav::ais

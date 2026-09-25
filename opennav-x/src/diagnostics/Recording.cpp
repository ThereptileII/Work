#include "diagnostics/Recording.h"
#include "vessel/DataItems.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>

namespace opennav::diagnostics {
using namespace vessel;
namespace {
void Require(bool ok, const char *message) {
  if (!ok)
    throw std::invalid_argument(message);
}
constexpr long long Span = 24 * 60 * 60 * 1000;
long long Stamp(Time t) {
  return std::chrono::duration_cast<Duration>(t.time_since_epoch()).count();
}
Time At(long long ms) {
  return Time{std::chrono::duration_cast<Clock::duration>(Duration(ms))};
}
void Utf8(const std::string &s) {
  for (std::size_t i = 0; i < s.size();) {
    const auto lead = static_cast<unsigned char>(s[i++]);
    if (lead < 128) {
      Require(lead != 0, "NUL in recording text");
      continue;
    }
    const unsigned n = lead >= 0xc2 && lead <= 0xdf   ? 1
                       : lead >= 0xe0 && lead <= 0xef ? 2
                       : lead >= 0xf0 && lead <= 0xf4 ? 3
                                                      : 0;
    Require(n && i + n <= s.size(), "Invalid UTF-8 recording text");
    unsigned code = lead & ((1u << (6 - n)) - 1);
    for (unsigned j = 0; j < n; ++j) {
      const auto c = static_cast<unsigned char>(s[i++]);
      Require((c & 0xc0) == 0x80, "Invalid UTF-8 continuation");
      code = (code << 6) | (c & 0x3f);
    }
    Require(code >= (n == 1   ? 0x80u
                     : n == 2 ? 0x800u
                              : 0x10000u) &&
                code <= 0x10ffff && !(code >= 0xd800 && code <= 0xdfff),
            "Invalid UTF-8 codepoint");
  }
}
std::string Hex(const std::string &s, std::size_t limit = 4096) {
  Require(s.size() <= limit, "Recording text exceeds limit");
  Utf8(s);
  static const char digits[] = "0123456789abcdef";
  std::string out;
  out.reserve(s.size() * 2);
  for (unsigned char c : s) {
    out += digits[c >> 4];
    out += digits[c & 15];
  }
  return out.empty() ? "-" : out;
}
std::string Unhex(const std::string &s, std::size_t limit = 4096) {
  if (s == "-")
    return {};
  Require(s.size() <= 2 * limit && s.size() % 2 == 0,
          "Invalid recording text length");
  auto digit = [](char c) -> unsigned {
    if (c >= '0' && c <= '9')
      return c - '0';
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    throw std::invalid_argument("Invalid recording text encoding");
  };
  std::string out;
  for (std::size_t i = 0; i < s.size(); i += 2) {
    const char c = static_cast<char>(digit(s[i]) * 16 + digit(s[i + 1]));
    Require(c != '\0', "NUL in recording text");
    out += c;
  }
  Utf8(out);
  return out;
}
long long Integer(const std::string &s, long long lo, long long hi) {
  Require(!s.empty() && s.size() <= 20, "Invalid integer length");
  Require(std::all_of(s.begin() + (s[0] == '-' ? 1 : 0), s.end(),
                      [](char c) { return c >= '0' && c <= '9'; }),
          "Invalid recording integer");
  std::size_t end = 0;
  long long n = 0;
  try {
    n = std::stoll(s, &end);
  } catch (const std::out_of_range &) {
    throw std::invalid_argument("Recording integer overflow");
  }
  Require(end == s.size() && n >= lo && n <= hi,
          "Recording integer out of range");
  return n;
}
std::optional<double> Number(const std::string &s) {
  if (s == "-")
    return {};
  const double value = application::ParseSettingNumber(s);
  Require(std::isfinite(value), "Nonfinite recording value");
  return value;
}
std::string Number(std::optional<double> n) {
  if (!n)
    return "-";
  Require(std::isfinite(*n), "Nonfinite recording value");
  return application::SettingNumber(*n);
}
std::vector<std::string> Split(const std::string &line) {
  Require(line.size() <= FrameByteLimit, "Recording line too long");
  std::vector<std::string> out;
  std::size_t at = 0;
  for (;;) {
    auto end = line.find('\t', at);
    out.push_back(line.substr(at, end - at));
    Require(out.size() <= 20, "Too many fields");
    if (end == std::string::npos)
      break;
    at = end + 1;
  }
  return out;
}
void Row(std::ostream &out, std::initializer_list<std::string> fields) {
  bool first = true;
  for (const auto &s : fields) {
    if (!first)
      out << '\t';
    out << s;
    first = false;
  }
  out << '\n';
}
std::string Valid(Validity v) {
  return std::to_string(static_cast<unsigned>(v));
}
Validity Valid(const std::string &s) {
  return static_cast<Validity>(Integer(s, 0, 3));
}
bool Coordinate(const std::string &name) {
  return name == "Latitude" || name == "Longitude";
}
void CheckSample(const Sample &s, Time at) {
  Require(s.observed_at <= at && Stamp(s.observed_at) >= -Span,
          "Invalid sample timestamp");
  Require(s.freshness.aging_after.count() >= 0 &&
              s.freshness.stale_after > s.freshness.aging_after &&
              s.freshness.stale_after <= Duration(300000),
          "Invalid recording freshness");
  Require(static_cast<unsigned>(s.validity) <= 3, "Invalid recording validity");
  Require(!s.value || (std::isfinite(*s.value) && !s.source.empty()),
          "Invalid recording sample");
}
void Domain(const std::string &name, const Sample &s) {
  if (!s.value)
    return;
  const double n = *s.value;
  if (name == "Latitude")
    Require(std::abs(n) <= 90, "Latitude out of range");
  else if (name == "Longitude")
    Require(std::abs(n) <= 180, "Longitude out of range");
  else if (name == "Speed over ground")
    Require(n >= 0 && n <= 200, "SOG out of range");
  else if (name == "Course over ground")
    Require(n >= 0 && n < 360, "COG out of range");
  else if (name == "Input rate")
    Require(n >= 0 && n <= 1000000, "Input rate out of range");
  else {
    // Build the name-to-domain mapping once. Checkpointing an hour-long
    // session must not recreate schema objects for every sample.
    static const auto domains = [] {
      std::map<std::string, QuantityInfo> result;
      const VesselState schema;
      for (const auto &item : DataItems(schema))
        for (const auto &q : Quantities())
          if (item.sample == &Field(schema, q.quantity))
            result.emplace(item.name, q);
      return result;
    }();
    const auto found = domains.find(name);
    Require(found != domains.end(), "Unspecified recorded domain");
    const auto &q = found->second;
    Require(n >= q.minimum && n <= q.maximum, "Sensor outside recorded domain");
    if (q.quantity == Quantity::Gear)
      Require(std::floor(n) == n, "Fractional gear");
  }
}
void CheckRoute(const RouteProgressSnapshot &r, Time at) {
  Require(r.remaining_steps.size() <= 128 && r.waypoint_count <= 100000,
          "Recorded route too large");
  Require(r.observed_at <= at && Stamp(r.observed_at) >= -Span,
          "Invalid route observation time");
  Require(!r.position_observed_at ||
              (*r.position_observed_at <= r.observed_at &&
               Stamp(*r.position_observed_at) >= -Span),
          "Invalid route position time");
  Require(static_cast<unsigned>(r.state) <=
              static_cast<unsigned>(RouteState::AwaitingProgress),
          "Unknown route state");
  Require(!r.remaining_distance_nm ||
              (std::isfinite(*r.remaining_distance_nm) &&
               *r.remaining_distance_nm >= 0 &&
               *r.remaining_distance_nm <= 1e9),
          "Invalid route distance");
  Require(!r.active_waypoint_index ||
              *r.active_waypoint_index < r.waypoint_count,
          "Invalid active index");
  if (r.state == RouteState::Valid) {
    Require(!r.route_id.empty() && !r.revision_scope.empty() &&
                r.route_revision && !r.active_waypoint_id.empty() &&
                r.active_waypoint_index && r.position_observed_at &&
                r.remaining_distance_nm && !r.source.empty() &&
                !r.position_source.empty(),
            "Incomplete valid route");
  }
  std::set<std::string> identities;
  for (const auto &step : r.remaining_steps) {
    Require(!step.waypoint_id.empty() &&
                identities.insert(step.waypoint_id).second,
            "Repeated recorded waypoint");
    Require(std::isfinite(step.latitude_deg) &&
                std::abs(step.latitude_deg) <= 90 &&
                std::isfinite(step.longitude_deg) &&
                std::abs(step.longitude_deg) <= 180,
            "Invalid recorded waypoint");
    Require(std::isfinite(step.distance_from_previous_nm) &&
                step.distance_from_previous_nm >= 0 &&
                step.distance_from_previous_nm <= 1e9,
            "Invalid recorded leg");
    Require(!step.course_true_deg ||
                (std::isfinite(*step.course_true_deg) &&
                 *step.course_true_deg >= 0 && *step.course_true_deg < 360),
            "Invalid recorded course");
  }
  if (!r.remaining_steps.empty() && r.state == RouteState::Valid)
    Require(r.remaining_steps.front().waypoint_id == r.active_waypoint_id &&
                r.remaining_steps.size() <=
                    r.waypoint_count - *r.active_waypoint_index,
            "Mismatched recorded route steps");
}
void WriteFrame(std::ostream &out, const RecordedFrame &f, bool nav) {
  const auto at = At(f.elapsed.count());
  Row(out,
      {"F", std::to_string(f.elapsed.count()), f.state.simulated ? "1" : "0"});
  for (const auto &item : DataItems(f.state)) {
    if (!nav && Coordinate(item.name))
      continue;
    const auto &s = *item.sample;
    if (s.source.empty() && !s.value)
      continue;
    CheckSample(s, at);
    Domain(item.name, s);
    Row(out, {"S", Hex(item.name), Hex(item.unit), Number(s.value),
              Valid(s.validity), std::to_string(Stamp(s.observed_at)),
              std::to_string(s.freshness.aging_after.count()),
              std::to_string(s.freshness.stale_after.count()), Hex(s.source),
              Hex(s.device_id)});
  }
  for (const auto &item : TextDataItems(f.state)) {
    const auto &s = *item.sample;
    if (s.source.empty() && !s.value)
      continue;
    CheckSample({s.value ? std::optional<double>{1} : std::nullopt, s.source,
                 s.observed_at, s.validity, s.freshness, s.device_id},
                at);
    Row(out, {"T", Hex(item.name), s.value ? Hex(*s.value) : "~",
              Valid(s.validity), std::to_string(Stamp(s.observed_at)),
              std::to_string(s.freshness.aging_after.count()),
              std::to_string(s.freshness.stale_after.count()), Hex(s.source),
              Hex(s.device_id)});
  }
  if (nav && f.state.navigation.route) {
    const auto &r = *f.state.navigation.route;
    CheckRoute(r, at);
    Row(out,
        {"R", Hex(r.route_id), Hex(r.revision_scope),
         std::to_string(r.route_revision), Hex(r.active_waypoint_id),
         r.active_waypoint_index ? std::to_string(*r.active_waypoint_index)
                                 : "-",
         std::to_string(r.waypoint_count), Number(r.remaining_distance_nm),
         std::to_string(Stamp(r.observed_at)),
         r.position_observed_at ? std::to_string(Stamp(*r.position_observed_at))
                                : "-",
         std::to_string(static_cast<unsigned>(r.state)), Hex(r.source),
         Hex(r.position_source), Hex(r.route_name)});
    for (const auto &step : r.remaining_steps)
      Row(out, {"L", Hex(step.waypoint_id), Hex(step.name),
                Number(step.latitude_deg), Number(step.longitude_deg),
                Number(step.distance_from_previous_nm),
                Number(step.course_true_deg)});
  }
  Row(out, {"E"});
}
} // namespace

application::Settings RecordingAssumptions(const application::Settings &s) {
  application::Settings result;
  result.energy = s.energy;
  result.current = s.current;
  result.hazard = s.hazard;
  result.energy.battery.source = "Recorded user energy assumptions";
  result.energy.curve.source = "Recorded calibration";
  // No arbitrary profile paths, connection passwords, source pin list or custom
  // mappings.
  return result;
}
RecordedFrame CaptureFrame(const VesselState &state, Time now, Time start,
                           bool nav) {
  Require(!state.replayed, "Cannot record replay as new observations");
  Require(now >= start && now - start <= Duration(Span),
          "Recording duration outside one day");
  RecordedFrame f{std::chrono::duration_cast<Duration>(now - start), state};
  for (auto &item : MutableDataItems(f.state)) {
    auto &s = *item.sample;
    if ((!nav && Coordinate(item.name)) || s.observed_at > now ||
        now - s.observed_at > Duration(Span)) {
      s = {};
      continue;
    }
    s.observed_at =
        At(std::chrono::duration_cast<Duration>(s.observed_at - start).count());
    if (s.value && !std::isfinite(*s.value)) {
      s.value.reset();
      s.validity = Validity::Invalid;
    }
  }
  for (auto &item : MutableTextDataItems(f.state)) {
    auto &s = *item.sample;
    if (s.observed_at > now || now - s.observed_at > Duration(Span)) {
      s = {};
      continue;
    }
    s.observed_at =
        At(std::chrono::duration_cast<Duration>(s.observed_at - start).count());
  }
  if (!nav)
    f.state.navigation.route.reset();
  else if (state.navigation.route) {
    auto r = std::make_shared<RouteProgressSnapshot>(*state.navigation.route);
    r->observed_at = At(
        std::chrono::duration_cast<Duration>(r->observed_at - start).count());
    if (r->position_observed_at)
      r->position_observed_at = At(
          std::chrono::duration_cast<Duration>(*r->position_observed_at - start)
              .count());
    f.state.navigation.route = std::move(r);
  }
  return f;
}
std::string EncodeRecording(const Recording &recording) {
  Require(!recording.frames.empty() &&
              recording.frames.size() <= RecordingFrameLimit,
          "Too many recorded frames");
  std::ostringstream out;
  out.imbue(std::locale::classic());
  Row(out,
      {"OpenNavXRecording", "1", recording.navigation_included ? "1" : "0"});
  // Configuration has a larger independently validated bound than item text.
  const auto config =
      application::EncodeSettings(RecordingAssumptions(recording.assumptions));
  Require(config.size() <= 60000, "Recording assumptions exceed limit");
  Row(out, {"C", Hex(config, 60000)});
  long long previous = -1;
  for (const auto &f : recording.frames) {
    Require(f.state.simulated == recording.frames.front().state.simulated &&
                !f.state.replayed,
            "Mixed live/Demo/replay recording");
    Require(f.elapsed.count() > previous && f.elapsed.count() <= Span,
            "Unordered recording frames");
    previous = f.elapsed.count();
    const auto before = out.tellp();
    WriteFrame(out, f, recording.navigation_included);
    Require(out.tellp() - before <= static_cast<std::streamoff>(FrameByteLimit),
            "Frame exceeds limit");
    Require(out.tellp() <= static_cast<std::streamoff>(RecordingByteLimit),
            "Recording exceeds size limit");
  }
  Row(out, {"END", std::to_string(recording.frames.size())});
  Require(out.tellp() <= static_cast<std::streamoff>(RecordingByteLimit),
          "Recording exceeds size limit");
  return out.str();
}

Recording DecodeRecording(const std::string &bytes) {
  Require(!bytes.empty() && bytes.size() <= RecordingByteLimit &&
              bytes.back() == '\n',
          "Truncated or oversized recording");
  std::istringstream in(bytes);
  std::string line;
  auto next = [&]() {
    Require(bool(std::getline(in, line)), "Truncated recording");
    return Split(line);
  };
  auto fields = next();
  Require(fields.size() == 3 && fields[0] == "OpenNavXRecording" &&
              fields[1] == "1" && (fields[2] == "0" || fields[2] == "1"),
          "Unsupported recording header");
  Recording result;
  result.navigation_included = fields[2] == "1";
  fields = next();
  Require(fields.size() == 2 && fields[0] == "C",
          "Missing recording assumptions");
  result.assumptions = application::DecodeSettings(Unhex(fields[1], 60000));
  Require(
      application::EncodeSettings(result.assumptions) ==
          application::EncodeSettings(RecordingAssumptions(result.assumptions)),
      "Recording contains non-recording configuration");
  bool finished = false;
  while (!finished) {
    fields = next();
    if (fields[0] == "END") {
      Require(fields.size() == 2 &&
                  Integer(fields[1], 1, RecordingFrameLimit) ==
                      static_cast<long long>(result.frames.size()),
              "Recording frame count mismatch");
      Require(in.peek() == std::char_traits<char>::eof(),
              "Trailing recording contents");
      finished = true;
      continue;
    }
    Require(fields.size() == 3 && fields[0] == "F" &&
                (fields[2] == "0" || fields[2] == "1") &&
                result.frames.size() < RecordingFrameLimit,
            "Invalid frame start");
    const auto elapsed = Integer(fields[1], 0, Span);
    Require(result.frames.empty() ||
                elapsed > result.frames.back().elapsed.count(),
            "Unordered recording");
    RecordedFrame frame;
    frame.elapsed = Duration(elapsed);
    frame.state.simulated = fields[2] == "1";
    Require(result.frames.empty() ||
                frame.state.simulated == result.frames.front().state.simulated,
            "Mixed live/Demo recording");
    std::set<std::string> seen;
    std::shared_ptr<RouteProgressSnapshot> route;
    const auto frameStart = in.tellg();
    for (;;) {
      fields = next();
      Require(in.tellg() - frameStart <=
                  static_cast<std::streamoff>(FrameByteLimit),
              "Oversized frame");
      if (fields[0] == "E") {
        Require(fields.size() == 1, "Invalid frame end");
        break;
      }
      if (fields[0] == "S") {
        Require(fields.size() == 10 && !route, "Invalid sample record");
        const auto name = Unhex(fields[1]);
        Require(seen.insert("S" + name).second, "Duplicate sample");
        Require(result.navigation_included || !Coordinate(name),
                "Unconsented navigation data");
        bool found = false;
        for (auto &item : MutableDataItems(frame.state))
          if (name == item.name) {
            Require(Unhex(fields[2]) == item.unit, "Wrong recorded unit");
            *item.sample = {Number(fields[3]),
                            Unhex(fields[8]),
                            At(Integer(fields[5], -Span, elapsed)),
                            Valid(fields[4]),
                            {Duration(Integer(fields[6], 0, 300000)),
                             Duration(Integer(fields[7], 1, 300000))},
                            Unhex(fields[9])};
            CheckSample(*item.sample, At(elapsed));
            Domain(name, *item.sample);
            found = true;
            break;
          }
        Require(found, "Unknown sample");
      } else if (fields[0] == "T") {
        Require(fields.size() == 9 && !route, "Invalid text sample record");
        const auto name = Unhex(fields[1]);
        Require(seen.insert("T" + name).second, "Duplicate text sample");
        bool found = false;
        for (auto &item : MutableTextDataItems(frame.state))
          if (name == item.name) {
            *item.sample = {fields[2] == "~" ? std::optional<std::string>{}
                                             : Unhex(fields[2]),
                            Unhex(fields[7]),
                            At(Integer(fields[4], -Span, elapsed)),
                            Valid(fields[3]),
                            {Duration(Integer(fields[5], 0, 300000)),
                             Duration(Integer(fields[6], 1, 300000))},
                            Unhex(fields[8])};
            const auto &s = *item.sample;
            CheckSample({s.value ? std::optional<double>{1} : std::nullopt,
                         s.source, s.observed_at, s.validity, s.freshness,
                         s.device_id},
                        At(elapsed));
            found = true;
            break;
          }
        Require(found, "Unknown text sample");
      } else if (fields[0] == "R") {
        Require(fields.size() == 14 && result.navigation_included && !route,
                "Invalid recorded route");
        route = std::make_shared<RouteProgressSnapshot>();
        auto &r = *route;
        r.route_id = Unhex(fields[1]);
        r.revision_scope = Unhex(fields[2]);
        // Revision uses the entire uint64 domain, with no signed conversion.
        Require(!fields[3].empty() && fields[3].size() <= 20 &&
                    std::all_of(fields[3].begin(), fields[3].end(),
                                [](char c) { return c >= '0' && c <= '9'; }),
                "Invalid route revision");
        try {
          r.route_revision = std::stoull(fields[3]);
        } catch (const std::out_of_range &) {
          throw std::invalid_argument("Route revision overflow");
        }
        r.active_waypoint_id = Unhex(fields[4]);
        if (fields[5] != "-")
          r.active_waypoint_index =
              static_cast<std::size_t>(Integer(fields[5], 0, 100000));
        r.waypoint_count =
            static_cast<std::size_t>(Integer(fields[6], 0, 100000));
        r.remaining_distance_nm = Number(fields[7]);
        r.observed_at = At(Integer(fields[8], -Span, elapsed));
        if (fields[9] != "-")
          r.position_observed_at = At(Integer(fields[9], -Span, elapsed));
        r.state = static_cast<RouteState>(
            Integer(fields[10], 0,
                    static_cast<long long>(RouteState::AwaitingProgress)));
        r.source = Unhex(fields[11]);
        r.position_source = Unhex(fields[12]);
        r.route_name = Unhex(fields[13]);
      } else if (fields[0] == "L") {
        Require(fields.size() == 7 && route &&
                    route->remaining_steps.size() < 128,
                "Invalid route leg");
        const auto lat = Number(fields[3]), lon = Number(fields[4]),
                   distance = Number(fields[5]);
        Require(lat && lon && distance, "Missing route geometry");
        route->remaining_steps.push_back({Unhex(fields[1]), Unhex(fields[2]),
                                          *lat, *lon, *distance,
                                          Number(fields[6])});
      } else
        throw std::invalid_argument("Unknown recording record");
    }
    if (route) {
      CheckRoute(*route, At(elapsed));
      frame.state.navigation.route = route;
    }
    result.frames.push_back(std::move(frame));
  }
  return result;
}
VesselState ReplayFrame(const RecordedFrame &frame, Time origin) {
  // Origin is supplied by a replay session, never recomputed by a UI read.
  Require(origin.time_since_epoch() >= Clock::duration::zero() &&
              origin <= Time::max() - std::chrono::hours(48),
          "Invalid replay clock origin");
  auto state = frame.state;
  state.replayed = true;
  auto rebase = [&](auto &s) {
    s.observed_at = origin + s.observed_at.time_since_epoch();
    if (!s.source.empty())
      s.source = "REPLAY / " + s.source;
  };
  for (auto &item : MutableDataItems(state))
    rebase(*item.sample);
  for (auto &item : MutableTextDataItems(state))
    rebase(*item.sample);
  if (state.navigation.route) {
    auto r = std::make_shared<RouteProgressSnapshot>(*state.navigation.route);
    r->observed_at = origin + r->observed_at.time_since_epoch();
    if (r->position_observed_at)
      r->position_observed_at =
          origin + r->position_observed_at->time_since_epoch();
    r->source = "REPLAY / " + r->source;
    r->position_source = "REPLAY / " + r->position_source;
    state.navigation.route = r;
  }
  return state;
}
} // namespace opennav::diagnostics

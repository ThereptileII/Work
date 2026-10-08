#include "weather/GribStream.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <map>
#include <set>
#include <tuple>

namespace opennav::weather::gribstream {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxComponent = 100.0;  // m/s; GFS 10 m wind never nears it.
constexpr long long kScale = 10000;      // 1e-4 degree ≈ 11 m.

std::string Fixed4(long long scaled) {
  std::string out;
  if (scaled < 0) {
    out.push_back('-');
    scaled = -scaled;
  }
  out += std::to_string(scaled / kScale);
  std::string frac = std::to_string(scaled % kScale);
  out.push_back('.');
  out.append(4 - frac.size(), '0');
  out += frac;
  return out;
}
long long Scaled(double degrees) {
  return static_cast<long long>(std::llround(degrees * kScale));
}
std::optional<double> Number(std::string_view text) {
  if (text.empty() || text.size() > 32) return std::nullopt;
  if (text.front() == '+') text.remove_prefix(1);
  double value = 0;
  const auto *end = text.data() + text.size();
  const auto parsed = std::from_chars(text.data(), end, value);
  if (parsed.ec != std::errc{} || parsed.ptr != end || !std::isfinite(value))
    return std::nullopt;
  return value;
}
bool Digits(std::string_view text, std::size_t at, std::size_t count, int &value) {
  if (at + count > text.size()) return false;
  value = 0;
  for (std::size_t i = at; i < at + count; ++i) {
    if (text[i] < '0' || text[i] > '9') return false;
    value = value * 10 + (text[i] - '0');
  }
  return true;
}
// Howard Hinnant's days_from_civil; proleptic Gregorian, UTC.
long long DaysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const long long era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + static_cast<long long>(doe) - 719468;
}
void CivilFromDays(long long z, int &y, unsigned &m, unsigned &d) {
  z += 719468;
  const long long era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned doe = static_cast<unsigned>(z - era * 146097);
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const long long yy = static_cast<long long>(yoe) + era * 400;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp = (5 * doy + 2) / 153;
  d = doy - (153 * mp + 2) / 5 + 1;
  m = mp < 10 ? mp + 3 : mp - 9;
  y = static_cast<int>(yy + (m <= 2));
}
int DaysInMonth(int y, int m) {
  static const int days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  const bool leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
  return m == 2 && leap ? 29 : days[m - 1];
}
std::string_view Trim(std::string_view s) {
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.remove_prefix(1);
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r'))
    s.remove_suffix(1);
  return s;
}
// Simple CSV fields: no embedded commas/quotes are expected from this
// service. A field wrapped in quotes is unwrapped; any other quote rejects it.
bool SplitFields(std::string_view line, std::vector<std::string_view> &fields,
                 std::size_t limit) {
  fields.clear();
  std::size_t start = 0;
  while (true) {
    const auto comma = line.find(',', start);
    auto field = Trim(line.substr(start, comma == std::string_view::npos
                                             ? std::string_view::npos
                                             : comma - start));
    if (field.size() >= 2 && field.front() == '"' && field.back() == '"')
      field = field.substr(1, field.size() - 2);
    if (field.find('"') != std::string_view::npos) return false;
    fields.push_back(field);
    if (fields.size() > limit) return false;
    if (comma == std::string_view::npos) return true;
    start = comma + 1;
  }
}
double LonDelta(double a, double b) { return std::abs(std::remainder(a - b, 360.0)); }
} // namespace

bool ValidModel(std::string_view model) {
  if (model.empty() || model.size() > 32) return false;
  return std::all_of(model.begin(), model.end(), [](char c) {
    return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
  });
}
std::string TimeseriesUrl(std::string_view model) {
  if (!ValidModel(model)) model = kDefaultModel;
  return std::string(kHost) + "/api/v2/" + std::string(model) + "/timeseries";
}

std::optional<ForecastQuery> Clamp(const ForecastQuery &query, WallTime now) {
  ForecastQuery out;
  std::set<std::pair<long long, long long>> seen;
  for (const auto &p : query.points) {
    if (out.points.size() >= kMaxForecastPoints) break;
    if (!std::isfinite(p.latitude_deg) || !std::isfinite(p.longitude_deg) ||
        std::abs(p.latitude_deg) > 90 || std::abs(p.longitude_deg) > 1e6)
      continue;
    double lon = std::remainder(p.longitude_deg, 360.0);
    long long lat_s = Scaled(p.latitude_deg), lon_s = Scaled(lon);
    if (lon_s >= 180 * kScale) lon_s -= 360 * kScale;
    if (!seen.insert({lat_s, lon_s}).second) continue;
    out.points.push_back({static_cast<double>(lat_s) / kScale,
                          static_cast<double>(lon_s) / kScale});
  }
  if (out.points.empty() || query.until <= query.from) return std::nullopt;
  using std::chrono::hours;
  const auto earliest = std::chrono::floor<hours>(now) - hours(1);
  WallTime from = std::chrono::floor<hours>(query.from);
  if (from < earliest) from = earliest;
  if (from > now + hours(16 * 24)) return std::nullopt;
  WallTime until = std::min<WallTime>(query.until, from + hours(kMaxForecastSteps - 1));
  if (until <= from) return std::nullopt;
  out.from = from;
  out.until = until;
  return out;
}

std::string RequestBody(const ForecastQuery &q) {
  std::string body = "{\"fromTime\":\"" + FormatUtc(q.from) + "\",\"untilTime\":\"" +
                     FormatUtc(q.until) + "\",\"coordinates\":[";
  for (std::size_t i = 0; i < q.points.size(); ++i) {
    if (i) body += ',';
    body += "{\"lat\":" + Fixed4(Scaled(q.points[i].latitude_deg)) +
            ",\"lon\":" + Fixed4(Scaled(q.points[i].longitude_deg)) +
            ",\"name\":\"p" + std::to_string(i) + "\"}";
  }
  body += "],\"variables\":["
          "{\"name\":\"UGRD\",\"level\":\"10 m above ground\",\"alias\":\"u\"},"
          "{\"name\":\"VGRD\",\"level\":\"10 m above ground\",\"alias\":\"v\"}],"
          "\"includeMetadata\":[\"native_coordinate\"]}";
  return body;
}

std::optional<WallTime> ParseUtc(std::string_view t) {
  int y, mo, d, h, mi, s;
  if (t.size() < 20 || t.size() > 40 || !Digits(t, 0, 4, y) || t[4] != '-' ||
      !Digits(t, 5, 2, mo) || t[7] != '-' || !Digits(t, 8, 2, d) || t[10] != 'T' ||
      !Digits(t, 11, 2, h) || t[13] != ':' || !Digits(t, 14, 2, mi) ||
      t[16] != ':' || !Digits(t, 17, 2, s))
    return std::nullopt;
  if (y < 1970 || y > 9999 || mo < 1 || mo > 12 || d < 1 || d > DaysInMonth(y, mo) ||
      h > 23 || mi > 59 || s > 59)
    return std::nullopt;
  std::size_t at = 19;
  long long nanos = 0;
  if (t[at] == '.') {
    ++at;
    int digits = 0;
    while (at < t.size() && t[at] >= '0' && t[at] <= '9') {
      if (++digits > 9) return std::nullopt;
      nanos = nanos * 10 + (t[at] - '0');
      ++at;
    }
    if (!digits) return std::nullopt;
    for (int i = digits; i < 9; ++i) nanos *= 10;
  }
  const auto zone = t.substr(at);
  if (zone != "Z" && zone != "+00:00") return std::nullopt;
  const long long seconds =
      DaysFromCivil(y, static_cast<unsigned>(mo), static_cast<unsigned>(d)) * 86400LL +
      h * 3600LL + mi * 60LL + s;
  return WallTime(std::chrono::duration_cast<WallTime::duration>(
      std::chrono::seconds(seconds) + std::chrono::nanoseconds(nanos)));
}

std::string FormatUtc(WallTime time) {
  const auto total = std::chrono::floor<std::chrono::seconds>(time).time_since_epoch().count();
  long long days = total / 86400, rem = total % 86400;
  if (rem < 0) {
    rem += 86400;
    --days;
  }
  int y;
  unsigned m, d;
  CivilFromDays(days, y, m, d);
  auto two = [](long long v) { return (v < 10 ? "0" : "") + std::to_string(v); };
  return std::to_string(y) + "-" + two(m) + "-" + two(d) + "T" + two(rem / 3600) +
         ":" + two(rem / 60 % 60) + ":" + two(rem % 60) + "Z";
}

std::optional<WindVector> FromUV(double u, double v) {
  if (!std::isfinite(u) || !std::isfinite(v) || std::abs(u) > kMaxComponent ||
      std::abs(v) > kMaxComponent)
    return std::nullopt;
  const double speed = std::hypot(u, v);
  if (!(speed <= kMaxComponent)) return std::nullopt;
  WindVector w;
  w.speed_mps = speed;
  if (speed < 1e-9) return w;  // Calm: direction is undefined; report 0.
  double dir = std::fmod(270.0 - std::atan2(v, u) * 180.0 / kPi + 360.0, 360.0);
  if (dir < 0) dir += 360.0;
  if (dir >= 360.0) dir -= 360.0;
  w.direction_from_true_deg = dir;
  return w;
}

ParseResult ParseCsv(std::string_view csv, const ForecastQuery &q) {
  ParseResult r;
  if (csv.size() > kMaxResponseBytes) {
    r.error = "Forecast response too large";
    return r;
  }
  if (csv.size() >= 3 && static_cast<unsigned char>(csv[0]) == 0xEF &&
      static_cast<unsigned char>(csv[1]) == 0xBB && static_cast<unsigned char>(csv[2]) == 0xBF)
    csv.remove_prefix(3);
  std::vector<std::string_view> fields;
  std::size_t pos = 0;
  auto next_line = [&](std::string_view &line) {
    while (pos < csv.size()) {
      const auto end = csv.find('\n', pos);
      line = csv.substr(pos, end == std::string_view::npos ? std::string_view::npos : end - pos);
      pos = end == std::string_view::npos ? csv.size() : end + 1;
      if (!Trim(line).empty()) return true;
    }
    return false;
  };
  std::string_view header;
  if (!next_line(header) || !SplitFields(header, fields, 32)) {
    r.error = "Forecast response has no readable header";
    return r;
  }
  std::map<std::string_view, std::size_t> column;
  for (std::size_t i = 0; i < fields.size(); ++i) column.emplace(fields[i], i);
  auto index = [&](std::string_view name) -> std::optional<std::size_t> {
    auto it = column.find(name);
    if (it == column.end()) return std::nullopt;
    return it->second;
  };
  const auto c_run = index("forecasted_at"), c_valid = index("forecasted_time"),
             c_name = index("name"), c_u = index("u"), c_v = index("v");
  const auto c_lat = index("lat"), c_lon = index("lon");
  const auto c_nlat = index("native_lat"), c_nlon = index("native_lon");
  if (!c_run || !c_valid || !c_name || !c_u || !c_v) {
    r.error = "Forecast response is missing required columns";
    return r;
  }
  const std::size_t width = fields.size();
  const auto earliest = q.from - std::chrono::hours(1);
  const auto latest = q.until + std::chrono::hours(1);
  std::map<std::pair<std::size_t, WallTime>, ForecastWind> best;
  std::size_t rows = 0;
  std::string_view line;
  while (next_line(line)) {
    if (++rows > kMaxResponseRows) {
      r.truncated = true;
      break;
    }
    if (!SplitFields(line, fields, width) || fields.size() != width) {
      ++r.rejected_rows;
      continue;
    }
    const auto name = fields[*c_name];
    std::size_t point = 0;
    bool name_ok = name.size() >= 2 && name.size() <= 4 && name[0] == 'p';
    for (std::size_t i = 1; name_ok && i < name.size(); ++i) {
      if (name[i] < '0' || name[i] > '9' || (i == 1 && name[i] == '0' && name.size() > 2))
        name_ok = false;
      else
        point = point * 10 + static_cast<std::size_t>(name[i] - '0');
    }
    const auto run = ParseUtc(fields[*c_run]);
    const auto valid = ParseUtc(fields[*c_valid]);
    const auto u = Number(fields[*c_u]), v = Number(fields[*c_v]);
    if (!name_ok || point >= q.points.size() || !run || !valid || !u || !v ||
        *valid < earliest || *valid > latest || *run > *valid + std::chrono::hours(24) ||
        *run < *valid - std::chrono::hours(24 * 16)) {
      ++r.rejected_rows;
      continue;
    }
    const auto &requested = q.points[point];
    if (c_lat && c_lon) {
      const auto lat = Number(fields[*c_lat]), lon = Number(fields[*c_lon]);
      if (!lat || !lon || std::abs(*lat - requested.latitude_deg) > 1e-3 ||
          LonDelta(*lon, requested.longitude_deg) > 1e-3) {
        ++r.rejected_rows;
        continue;
      }
    }
    const auto wind = FromUV(*u, *v);
    if (!wind) {
      ++r.rejected_rows;
      continue;
    }
    ForecastWind value;
    value.requested = requested;
    if (c_nlat && c_nlon) {
      const auto lat = Number(fields[*c_nlat]), lon = Number(fields[*c_nlon]);
      if (lat && lon && std::abs(*lat) <= 90 && std::abs(*lon) <= 360 &&
          std::abs(*lat - requested.latitude_deg) <= 2.0 &&
          LonDelta(*lon, requested.longitude_deg) <= 2.0)
        value.grid = Coordinate{*lat, std::remainder(*lon, 360.0)};
    }
    value.speed_mps = wind->speed_mps;
    value.direction_from_true_deg = wind->direction_from_true_deg;
    value.model_run = *run;
    value.valid_time = *valid;
    auto [it, inserted] = best.emplace(std::make_pair(point, *valid), value);
    if (!inserted && it->second.model_run < value.model_run) it->second = value;
  }
  std::set<WallTime> steps;
  for (const auto &entry : best) steps.insert(entry.first.second);
  std::set<WallTime> kept;
  for (auto t : steps) {
    if (kept.size() >= kMaxForecastSteps) break;
    kept.insert(t);
  }
  for (auto &entry : best)
    if (kept.count(entry.first.second)) r.winds.push_back(entry.second);
  std::sort(r.winds.begin(), r.winds.end(), [](const auto &a, const auto &b) {
    return std::tie(a.valid_time, a.requested.latitude_deg, a.requested.longitude_deg) <
           std::tie(b.valid_time, b.requested.latitude_deg, b.requested.longitude_deg);
  });
  if (r.winds.empty()) {
    r.error = rows ? "Forecast response contained no valid wind values"
                   : "Forecast service returned no data for this area";
    return r;
  }
  r.ok = true;
  return r;
}
} // namespace opennav::weather::gribstream

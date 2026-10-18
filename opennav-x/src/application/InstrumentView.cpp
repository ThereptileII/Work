#include "application/InstrumentView.h"
#include "vessel/DisplayItems.h"
#include <algorithm>
#include <cmath>

namespace opennav::application {
namespace {
bool Chosen(const std::vector<std::string> &selection, const std::string &key) {
  return std::find(selection.begin(), selection.end(), key) != selection.end();
}
InstrumentReading Read(const vessel::DisplayItem &item, vessel::Time now,
                       bool selected = true) {
  InstrumentReading r;
  r.key = item.key;
  r.title = item.title;
  r.unit = item.unit;
  r.source = item.sample->source;
  r.selected = selected;
  if (r.key == "sog")
    r.title = "SPEED OVER GROUND";
  if (r.key == "cog")
    r.title = "COURSE OVER GROUND";
  if (r.key == "stw")
    r.title = "SPEED THROUGH WATER";
  if (r.key == "tws")
    r.title = "TRUE WIND SPEED";
  if (r.key == "aws")
    r.title = "APPARENT WIND SPEED";
  if (r.key == "twa")
    r.title = "TRUE WIND ANGLE";
  if (r.key == "awa")
    r.title = "APPARENT WIND ANGLE";
  if (r.key == "depth") {
    r.title = "DEPTH / TRANSDUCER";
    r.unit = "m";
  }
  if (r.key == "water_temp")
    r.title = "WATER TEMPERATURE";
  if (r.key == "heading")
    r.title = "HEADING / TRUE";
  const bool absolute = r.key == "heading" || r.key == "cog";
  const bool relative = r.key == "twa" || r.key == "awa";
  if (absolute) {
    r.decimals = 0;
    r.unit = "°T";
  }
  if (relative) {
    r.decimals = 0;
    r.unit = "° relative";
  }
  if (r.key == "rpm")
    r.decimals = 0;
  if (!selected)
    return r;
  const auto &s = *item.sample;
  // Invalid external freshness configuration cannot throw through the painter.
  if (s.freshness.aging_after < vessel::Duration::zero() ||
      s.freshness.stale_after <= s.freshness.aging_after)
    return r;
  const auto a = vessel::Assess(s, now);
  r.quality = a.quality;
  r.age = a.age;
  if (a.quality != vessel::Quality::Live &&
      a.quality != vessel::Quality::Aging &&
      a.quality != vessel::Quality::Estimated)
    return r;
  if (a.value && ((absolute && (*a.value < 0 || *a.value >= 360)) ||
                  (relative && std::abs(*a.value) > 180))) {
    r.quality = vessel::Quality::Unavailable;
    return r;
  }
  r.value = a.value;
  return r;
}
} // namespace
InstrumentView PresentInstruments(const vessel::VesselState &s,
                                  const std::vector<std::string> &selection,
                                  vessel::Time now) {
  InstrumentView v;
  v.replayed = s.replayed;
  v.simulated = s.simulated;
  const auto items = vessel::DisplayItems(s);
  auto read = [&](const char *key) {
    for (const auto &item : items)
      if (std::string(item.key) == key)
        return Read(item, now, Chosen(selection, key));
    return InstrumentReading{};
  };
  v.heading = read("heading");
  v.true_speed = read("tws");
  v.true_angle = read("twa");
  v.wind_speed = v.true_speed;
  v.wind_angle = v.true_angle;
  const auto heading_at = s.navigation.heading_true_deg.observed_at;
  const auto coherent = [](vessel::Time a, vessel::Time b) {
    return (a > b ? a - b : b - a) <= std::chrono::seconds(2);
  };
  const bool true_direction = v.true_angle.value &&
      coherent(heading_at, s.wind.true_angle_deg.observed_at);
  const auto apparent_speed = read("aws"), apparent_angle = read("awa");
  const auto measured = [](const InstrumentReading &reading) {
    return reading.value && (reading.quality == vessel::Quality::Live ||
                             reading.quality == vessel::Quality::Aging);
  };
  const bool apparent_pair = measured(apparent_speed) && measured(apparent_angle) &&
      apparent_speed.source == apparent_angle.source &&
      coherent(s.wind.apparent_speed_kn.observed_at, s.wind.apparent_angle_deg.observed_at);
  if (!true_direction && apparent_pair) {
    v.wind_speed = apparent_speed;
    v.wind_angle = apparent_angle;
    v.wind_is_apparent = true;
  }
  const auto wind_at = v.wind_is_apparent ? s.wind.apparent_angle_deg.observed_at
                                         : s.wind.true_angle_deg.observed_at;
  if (v.heading.value && v.wind_angle.value && coherent(heading_at, wind_at))
    v.wind_bearing_true_deg =
        std::fmod(*v.heading.value + *v.wind_angle.value + 360., 360.);
  else if (!v.heading.value)
    v.wind_direction_reason = "True heading unavailable";
  else if (!v.wind_angle.value)
    v.wind_direction_reason = "No coherent wind direction available";
  else
    v.wind_direction_reason = "Heading and wind observations are not aligned";
  // Preserve configured selection. Put the prototype's eight primary tiles
  // first, with other configured quantities below; no setting is discarded.
  std::vector<std::string> keys{"sog", "cog", "stw",  "depth",
                                "aws", "awa", "heel", "water_temp"};
  for (const auto &key : selection)
    if (!Chosen(keys, key))
      keys.push_back(key);
  for (const auto &key : keys) {
    if (!Chosen(selection, key) || key == "heading" || key == "tws" ||
        key == "twa")
      continue;
    for (const auto &item : items)
      if (key == item.key)
        v.tiles.push_back(Read(item, now));
  }
  return v;
}
} // namespace opennav::application

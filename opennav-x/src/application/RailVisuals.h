#pragma once
#include "smartnav/Energy.h"
#include "vessel/VesselState.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace opennav::application {
// SCRUM-361: what the prototype draws under a rail value. Plain data, built
// only from current measurements; anything unknown yields Kind::None or omits
// the part (no mark without a safety depth, no "At destination" without a
// prediction).
struct RailVisual {
  enum class Kind { None, Sparkline, Meter, Direction, Bar } kind = Kind::None;
  std::vector<double> series;
  double fraction = -1, marker = -1, angle_deg = 0;
  bool warning = false;
  std::string caption, value_text;
};

// SOG history for the sparkline: one point per interval, a bounded window.
class RailHistory {
 public:
  static constexpr std::size_t kPoints = 30;           // 5 min at 10 s.
  static constexpr auto kInterval = std::chrono::seconds(10);
  void Observe(const vessel::Sample &sample, vessel::Time now) {
    const auto a = vessel::Assess(sample, now);
    if (!a.value || a.quality != vessel::Quality::Live) {
      if (a.quality == vessel::Quality::Stale || a.quality == vessel::Quality::Unavailable)
        values_.clear();  // A gap must not be drawn as a continuous line.
      return;
    }
    if (last_ && now - *last_ < kInterval) return;
    last_ = now;
    values_.push_back(*a.value);
    while (values_.size() > kPoints) values_.pop_front();
  }
  std::vector<double> Values() const { return {values_.begin(), values_.end()}; }

 private:
  std::deque<double> values_;
  std::optional<vessel::Time> last_;
};

inline std::optional<double> LiveValue(const vessel::Sample &sample, vessel::Time now) {
  const auto a = vessel::Assess(sample, now);
  if (!a.value || (a.quality != vessel::Quality::Live && a.quality != vessel::Quality::Aging))
    return std::nullopt;
  return a.value;
}

// Fixed-point text without printf: MSVC/wx headers can macro-replace
// snprintf, and the classic locale keeps a '.' decimal separator.
inline std::string Fixed(double value, int decimals) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::fixed << std::setprecision(decimals) << value;
  return out.str();
}

inline RailVisual RailVisualFor(const std::string &key, const vessel::VesselState &s,
                                vessel::Time now, const RailHistory &sog,
                                std::optional<double> safety_depth_m,
                                const smartnav::EnergyPrediction *energy) {
  RailVisual v;
  if (key == "sog") {
    v.series = sog.Values();
    if (v.series.size() >= 3) v.kind = RailVisual::Kind::Sparkline;
  } else if (key == "depth") {
    const auto depth = LiveValue(s.environment.depth_below_transducer_m, now);
    if (!depth || *depth < 0) return v;
    const bool safety = safety_depth_m && std::isfinite(*safety_depth_m) && *safety_depth_m > 0;
    // Scale so the safety mark sits near the prototype's ~30 % position.
    const double full = safety ? std::max(*safety_depth_m * 3.5, 10.0) : 10.0;
    v.kind = RailVisual::Kind::Meter;
    v.fraction = std::clamp(*depth / std::max(full, *depth), 0.0, 1.0);
    if (safety) {
      v.marker = *safety_depth_m / std::max(full, *depth);
      v.warning = *depth < *safety_depth_m;
      v.caption = Fixed(*safety_depth_m, 1) + " m safety";
    }
  } else if (key == "aws" || key == "tws") {
    const auto angle = LiveValue(key == "aws" ? s.wind.apparent_angle_deg
                                              : s.wind.true_angle_deg, now);
    if (!angle || std::abs(*angle) > 180) return v;
    v.kind = RailVisual::Kind::Direction;
    // The arrow shows where the wind comes from, relative to the bow.
    v.angle_deg = *angle + 180;
    v.value_text = Fixed(std::abs(*angle), 0) + "\xC2\xB0";
    v.caption = std::abs(*angle) < 0.5 || std::abs(*angle) > 179.5 ? ""
                : *angle < 0 ? "PORT" : "STBD";
  } else if (key == "soc") {
    const auto soc = LiveValue(s.battery.soc_percent, now);
    if (!soc) return v;
    v.kind = RailVisual::Kind::Bar;
    v.fraction = std::clamp(*soc / 100.0, 0.0, 1.0);
    if (energy && energy->arrival.estimate && energy->arrival.estimate->soc_percent) {
      v.caption = "At destination";
      v.value_text = Fixed(*energy->arrival.estimate->soc_percent, 0) + "%";
    }
  }
  return v;
}
} // namespace opennav::application

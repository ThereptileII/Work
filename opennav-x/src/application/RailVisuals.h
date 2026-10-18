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
// the part (no safety mark across incompatible depth datums, no
// "At destination" without a prediction).
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
    if (source_ != sample.source || (last_ && now < *last_) ||
        (observed_ && sample.observed_at < *observed_)) {
      values_.clear(); last_.reset(); observed_.reset();
    }
    source_ = sample.source;
    const auto a = vessel::Assess(sample, now);
    if (!a.value || a.quality != vessel::Quality::Live) {
      if (a.quality != vessel::Quality::Aging) {
        values_.clear(); last_.reset(); observed_.reset();
      } // A gap or source switch must not be drawn as a continuous line.
      return;
    }
    if (observed_ && sample.observed_at == *observed_) return;
    if (last_ && now - *last_ < kInterval) return;
    last_ = now;
    observed_ = sample.observed_at;
    values_.push_back(*a.value);
    while (values_.size() > kPoints) values_.pop_front();
  }
  std::vector<double> Values() const { return {values_.begin(), values_.end()}; }

 private:
  std::deque<double> values_;
  std::optional<vessel::Time> last_;
  std::optional<vessel::Time> observed_;
  std::string source_;
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
    if (!LiveValue(s.navigation.sog_kn, now)) return v;
    v.series = sog.Values();
    if (v.series.size() >= 3) v.kind = RailVisual::Kind::Sparkline;
  } else if (key == "depth") {
    const auto depth = LiveValue(s.environment.depth_below_transducer_m, now);
    if (!depth || *depth < 0) return v;
    // This snapshot is explicitly below-transducer depth. Without a known
    // surface offset it cannot be compared to chart safety depth. Preserve the
    // measurement and meter, but do not invent a matching datum or warning.
    (void)safety_depth_m;
    const double full = std::max(10.0, *depth);
    v.kind = RailVisual::Kind::Meter;
    v.fraction = std::clamp(*depth / full, 0.0, 1.0);
  } else if (key == "aws" || key == "tws") {
    const auto &direction = key == "aws" ? s.wind.apparent_angle_deg : s.wind.true_angle_deg;
    const auto &speed = key == "aws" ? s.wind.apparent_speed_kn : s.wind.true_speed_kn;
    const auto a = vessel::Assess(direction, now), b = vessel::Assess(speed, now);
    const auto usable = [](vessel::Quality quality) {
      return quality == vessel::Quality::Live || quality == vessel::Quality::Aging ||
             quality == vessel::Quality::Estimated;
    };
    const auto skew = direction.observed_at > speed.observed_at
        ? direction.observed_at - speed.observed_at : speed.observed_at - direction.observed_at;
    if (!a.value || !b.value || !usable(a.quality) || !usable(b.quality) ||
        std::abs(*a.value) > 180 || direction.source != speed.source ||
        skew > std::chrono::seconds(2)) return v;
    const auto angle = a.value;
    v.kind = RailVisual::Kind::Direction;
    // The arrow shows where the wind comes from, relative to the bow.
    v.angle_deg = *angle + 180;
    v.value_text = Fixed(std::abs(*angle), 0) + "\xC2\xB0";
    v.caption = std::abs(*angle) < 0.5 || std::abs(*angle) > 179.5 ? ""
                : *angle < 0 ? "PORT" : "STBD";
    if (a.quality == vessel::Quality::Estimated && b.quality != vessel::Quality::Estimated)
      v.caption += v.caption.empty() ? "EST." : " / EST.";
  } else if (key == "soc") {
    const auto reading = vessel::Assess(s.battery.soc_percent, now);
    const auto soc = reading.value;
    if (!soc || *soc < 0 || *soc > 100 ||
        (reading.quality != vessel::Quality::Live && reading.quality != vessel::Quality::Aging &&
         reading.quality != vessel::Quality::Estimated)) return v;
    v.kind = RailVisual::Kind::Bar;
    v.fraction = std::clamp(*soc / 100.0, 0.0, 1.0);
    if (energy && energy->arrival.estimate && energy->arrival.estimate->soc_percent &&
        std::isfinite(*energy->arrival.estimate->soc_percent) &&
        *energy->arrival.estimate->soc_percent >= 0 && *energy->arrival.estimate->soc_percent <= 100) {
      v.caption = "At destination";
      v.value_text = Fixed(*energy->arrival.estimate->soc_percent, 0) + "%";
    }
  }
  return v;
}
} // namespace opennav::application

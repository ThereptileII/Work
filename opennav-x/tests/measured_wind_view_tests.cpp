#include "application/MeasuredWindView.h"
#include <iostream>
#include <stdexcept>

using namespace opennav;
using namespace std::chrono_literals;

namespace {
void Check(bool ok, const char *why) {
  if (!ok) throw std::runtime_error(why);
}
vessel::Sample Sample(double value, const char *source, vessel::Time observed) {
  vessel::Sample s;
  s.value = value;
  s.source = source;
  s.observed_at = observed;
  s.validity = vessel::Validity::Measured;
  return s;
}
} // namespace

int main() {
  try {
    const auto now = vessel::Time{} + 30s;
    vessel::Wind wind;
    wind.apparent_speed_kn = Sample(12.4, "NMEA2000/130306", now - 500ms);
    wind.apparent_angle_deg = Sample(-72, "NMEA0183/MWV", now - 1500ms);
    auto view = application::MeasureWind(wind, now);
    Check(application::WindSpeedText(view.true_speed) == "\xE2\x80\x94" &&
          application::WindStatusText(view.true_speed) == "UNAVAILABLE",
          "Missing true wind does not hide or replace apparent wind");
    Check(application::WindSpeedText(view.apparent_speed) == "12.4 kn" &&
          application::WindAngleText(view.apparent_angle) == "72\xC2\xB0 port",
          "Independent live AWS and AWA retain units and signed side");
    Check(application::WindStatusText(view.apparent_speed) == "LIVE \xC2\xB7 0.5 s" &&
          application::WindSourceText(view.apparent_speed) == "Source: NMEA2000/130306" &&
          application::WindStatusText(view.apparent_angle) == "LIVE \xC2\xB7 1.5 s" &&
          application::WindSourceText(view.apparent_angle) == "Source: NMEA0183/MWV",
          "Each apparent field retains its own source and age");

    view = application::MeasureWind(wind, now + 2s);
    Check(application::WindSpeedText(view.apparent_speed) == "12.4 kn" &&
          application::WindStatusText(view.apparent_speed).find("AGING") != std::string::npos,
          "Aging apparent wind remains visible with explicit quality");
    view = application::MeasureWind(wind, now + 6s);
    Check(application::WindSpeedText(view.apparent_speed) == "\xE2\x80\x94" &&
          application::WindAngleText(view.apparent_angle) == "\xE2\x80\x94" &&
          application::WindStatusText(view.apparent_speed).find("STALE") != std::string::npos,
          "Stale apparent wind has no current numeric value");

    wind.apparent_speed_kn = {};
    wind.apparent_angle_deg = Sample(35, "NMEA0183/MWV", now - 500ms);
    view = application::MeasureWind(wind, now);
    Check(application::WindSpeedText(view.apparent_speed) == "\xE2\x80\x94" &&
          application::WindAngleText(view.apparent_angle) == "35\xC2\xB0 starboard",
          "AWA is independent when AWS is unavailable");
    wind.apparent_angle_deg.validity = vessel::Validity::Estimated;
    view = application::MeasureWind(wind, now);
    Check(application::WindAngleText(view.apparent_angle) == "\xE2\x80\x94" &&
          application::WindStatusText(view.apparent_angle).find("ESTIMATED") != std::string::npos,
          "Estimated angle cannot masquerade as measured onboard");
    wind.apparent_speed_kn = Sample(-2, "NMEA2000/130306", now);
    view = application::MeasureWind(wind, now);
    Check(application::WindSpeedText(view.apparent_speed) == "\xE2\x80\x94" &&
          application::WindStatusText(view.apparent_speed).find("INVALID") != std::string::npos,
          "Out-of-domain speed is rejected");
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  std::cout << "measured wind presentation passed\n";
}

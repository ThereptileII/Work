#include "integration/ChartLightHover.h"
#include <iostream>
#include <stdexcept>

using namespace opennav::integration;
namespace {
void Check(bool value, const char *why) {
  if (!value) throw std::runtime_error(why);
}
}
int main() {
  try {
    const wxColour red(255, 0, 0, 100), green(0, 255, 0, 100), white(255, 255, 0, 130);
    for (const auto *scheme : {"DAY", "DUSK", "NIGHT"}) {
      const auto r = ExtendedLightSectorInk(true, red, scheme);
      const auto g = ExtendedLightSectorInk(true, green, scheme);
      const auto w = ExtendedLightSectorInk(true, white, scheme);
      Check(r.Red() > r.Green() && g.Green() > g.Red(),
            "Red and green sector meaning remains distinguishable in each theme");
      Check(r != g && r != w && g != w,
            "White/default, red and green sectors remain distinct");
      Check(r != red && g != green && w != white,
            "Each supported hover class uses the XNav palette");
      Check(ExtendedLightSectorInk(false, red, scheme) == red &&
            ExtendedLightSectorInk(false, green, scheme) == green &&
            ExtendedLightSectorInk(false, white, scheme) == white,
            "Legacy colour and alpha remain unchanged");
      const auto boundary = ExtendedLightBoundaryInk(true, scheme, 128);
      Check(boundary.Alpha() == 128 && boundary != wxColour(0, 0, 0, 128),
            "Theme-aware boundary ink retains original opacity");
    }
    const auto day = ExtendedLightSectorInk(true, red, "DAY");
    const auto dusk = ExtendedLightSectorInk(true, red, "DUSK");
    const auto night = ExtendedLightSectorInk(true, red, "NIGHT");
    Check(day.Alpha() == 100 && dusk.Alpha() == 50 && night.Alpha() == 20,
          "Retained hover state resolves current theme brightness at paint time");
    const wxColour unknown(13, 27, 39, 55);
    Check(ExtendedLightSectorInk(true, unknown, "DAY") == unknown &&
          ExtendedLightSectorInk(true, red, "UNKNOWN") == red,
          "Unknown colour classes and schemes preserve source appearance");
    Check(ExtendedLightBoundaryInk(false, "NIGHT", 50) == wxColour(0, 0, 0, 50),
          "Legacy boundary remains unchanged");
    std::cout << "Light hover palette tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}

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
    Check(RicherLightSectorInk(3,0xb66e6c)==0xbd6765 &&
          RicherLightSectorInk(3,0xd3948c)==0xda8f85 &&
          RicherLightSectorInk(4,0x508d78)==0x4a937a &&
          RicherLightSectorInk(4,0x8fbaa2)==0x8bbea2 &&
          RicherLightSectorInk(3,0x885e57,true)==0x8d5a52 &&
          RicherLightSectorInk(4,0x5e7a67,true)==0x597b64,
          "Sector richness matches the independent owned-SVG HLS fixtures");
    const auto luminance=[](std::uint32_t c) {
      return ((c>>16)&255)*2126+((c>>8)&255)*7152+(c&255)*722;
    };
    for(const auto rgb:{0x885e57u,0x5e7a67u})for(unsigned color:{3,4}) {
      Check(luminance(RicherLightSectorInk(color,rgb,true))<=luminance(rgb),
            "Night richness never increases weighted luminance");
      Check(RicherLightSectorInk(1,rgb,true)==rgb &&
            RicherLightSectorInk(6,rgb,true)==rgb,
            "White and unsupported lens classes remain unchanged");
    }
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

#pragma once
#include <cstdint>
namespace opennav::chart_style::generated {
struct Resource { const char *name; const char *sha256; std::uint64_t bytes; };
inline constexpr Resource resources[] = {
  {"chartsymbols.xml", "5aca020aae65f3bcb38ff548ee183868dca72811cdd33dd26cc89d2c726738e3", 2242894},
  {"S52RAZDS.RLE", "560e1a1df493d7ae4c8506da814ad936adb13ef93c6f1e2abc93f2819860ff26", 1305627},
  {"rastersymbols-day.png", "9569ec1e433b33b2d178787f9f9e86fbc29fa43b78a9317d8e97fe3f7dc30153", 196379},
  {"rastersymbols-dusk.png", "270ba6ae5f2e5492683441717bcd881ba5fd1d6dcb58c5b19dd72dfdcb0359e0", 173348},
  {"rastersymbols-dark.png", "c187b5ebcc54ee481c8a71880cfca8933da8e1ae6b5c2d5af9bfb54ec7a59eb7", 168069},
};
struct Background { std::uint32_t land, water; };
inline constexpr Background backgrounds[] = {
  {0xeeeee2, 0xd5e5e5},
  {0x4e615d, 0x344f59},
  {0x1d2925, 0x0e171c},
};
} // namespace opennav::chart_style::generated

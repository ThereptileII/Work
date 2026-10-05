#pragma once
#include <cstdint>
namespace opennav::chart_style::generated {
struct Resource { const char *name; const char *sha256; std::uint64_t bytes; };
inline constexpr Resource resources[] = {
  {"chartsymbols.xml", "5aca020aae65f3bcb38ff548ee183868dca72811cdd33dd26cc89d2c726738e3", 2242894},
  {"S52RAZDS.RLE", "560e1a1df493d7ae4c8506da814ad936adb13ef93c6f1e2abc93f2819860ff26", 1305627},
  {"rastersymbols-day.png", "865c7238985b2ea7e2d2c46d2612f70b0d2e3b4992a33f6bf3cd2fb5fee59de9", 199897},
  {"rastersymbols-dusk.png", "33fa1b8f539a5f90b2d220599ee9fe926cf11f86b60ce643b720037ba4e26446", 174289},
  {"rastersymbols-dark.png", "55aa9eb7bb9624ca6d730c47e481c2eea2c9dbdefef2f5ccebf54e67bc9eb116", 169663},
};
struct Background { std::uint32_t land, water; };
inline constexpr Background backgrounds[] = {
  {0xeeeee2, 0xd5e5e5},
  {0x4e615d, 0x344f59},
  {0x1d2925, 0x0e171c},
};
} // namespace opennav::chart_style::generated

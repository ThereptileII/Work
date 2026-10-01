#pragma once
#include <algorithm>

namespace opennav::ui {
// The immutable prototype changes only these XNav CSS selectors. OS DPI is
// applied separately by wx FromDIP; these are logical CSS-pixel targets.
constexpr bool ValidInterfaceScale(int percent) {
  return percent == 100 || percent == 125 || percent == 150;
}
constexpr int DisplayFieldHeight(int percent) {
  return percent == 150 ? 56 : percent == 125 ? 52 : 46;
}
constexpr int DisplayFieldFont(int percent) {
  return percent == 150 ? 18 : 16;
}
constexpr int DisplayActionHeight(int percent, int base_height) {
  return percent == 150 ? std::max(base_height, 56) : base_height;
}
constexpr int DisplayActionFont(int percent, int base_font) {
  return percent == 150 ? 14 : base_font;
}
} // namespace opennav::ui

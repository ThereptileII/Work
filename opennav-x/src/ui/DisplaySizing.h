#pragma once
#include <algorithm>

namespace opennav::ui {
// User display magnification is independent of Windows DPI. Fixed shell bars
// keep the chart workspace; fields, actions and rail/chart contents magnify.
constexpr bool ValidInterfaceScale(int percent) {
  return percent == 100 || percent == 125 || percent == 150;
}
constexpr int DisplayPixels(int pixels, int percent) {
  return (pixels * (ValidInterfaceScale(percent) ? percent : 100) + 50) / 100;
}
constexpr int DisplayFieldHeight(int percent) {
  return DisplayPixels(46, percent);
}
constexpr int DisplayFieldFont(int percent) {
  return DisplayPixels(16, percent);
}
constexpr int DisplayActionHeight(int percent, int base_height) {
  return DisplayPixels(base_height, percent);
}
constexpr int DisplayActionFont(int percent, int base_font) {
  return DisplayPixels(base_font, percent);
}
} // namespace opennav::ui

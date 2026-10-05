#pragma once
#include "plugin-adapters/ChartPresentationBindingV1.h"
namespace opennav::integration {
struct OChartsPointStyle {
  bool available = false;
  unsigned effective_point_style = 0;
};
// Validate copied module data; never manufacture a table from selection status.
inline bool ValidOChartsPointStyle(const SkagerChartPointStyleV1& value) {
  if (value.structBytes != sizeof(value) ||
      value.version != SKAGER_CHART_POINT_STYLE_VERSION) return false;
  for (auto word : value.reserved) if (word) return false;
  if (value.available == 0) return value.effectivePointStyle == 0;
  return value.available == 1 &&
      (value.effectivePointStyle == SKAGER_CHART_POINT_STYLE_SIMPLIFIED ||
       value.effectivePointStyle == SKAGER_CHART_POINT_STYLE_PAPER);
}
inline OChartsPointStyle DecodeOChartsPointStyle(const SkagerChartPointStyleV1& value) {
  if (!ValidOChartsPointStyle(value) || !value.available) return {};
  return {true, value.effectivePointStyle};
}
} // namespace opennav::integration

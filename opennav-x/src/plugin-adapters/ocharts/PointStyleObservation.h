#pragma once
#include "plugin-adapters/ChartPresentationBindingV1.h"
#include <atomic>

namespace skager::ocharts {
// Renderer reads are application-thread only. Revocation is atomic so an
// unexpected off-thread teardown still makes later observations unavailable.
// The renderer is borrowed for one read;
// no renderer address or callback is retained or crosses the private ABI.
class PointStyleObservation {
 public:
  void SetActive(bool active) { active_.store(active); }
  template<class Renderer>
  bool Read(SkagerChartPointStyleV1* out, bool main_thread, bool selected,
            const Renderer* renderer) const {
    if (!main_thread || !out || out->structBytes != sizeof(*out) ||
        out->version != SKAGER_CHART_POINT_STYLE_VERSION || out->available ||
        out->effectivePointStyle) return false;
    for (auto value : out->reserved) if (value) return false;
    SkagerChartPointStyleV1 result{};
    result.structBytes = sizeof(result);
    result.version = SKAGER_CHART_POINT_STYLE_VERSION;
    if (active_.load() && selected && renderer && renderer->m_bOK) {
      const auto actual = static_cast<uint32_t>(renderer->GetEffectiveSymbolStyle());
      if (actual != SKAGER_CHART_POINT_STYLE_SIMPLIFIED &&
          actual != SKAGER_CHART_POINT_STYLE_PAPER) return false;
      result.available = 1;
      result.effectivePointStyle = actual;
    }
    *out = result;
    return true;
  }
 private:
  std::atomic<bool> active_{false};
};
} // namespace skager::ocharts

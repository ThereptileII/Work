#include "plugin-adapters/ocharts/PointStyleObservation.h"
#include "integration/OChartsPointStyle.h"
#include <cstring>
#include <iostream>
#include <stdexcept>
using skager::ocharts::PointStyleObservation;
using opennav::integration::DecodeOChartsPointStyle;
using opennav::integration::ValidOChartsPointStyle;
static_assert(sizeof(SkagerChartBindingV1) == 4136);
static_assert(sizeof(SkagerChartPresentationStatusV1) == 48);
static_assert(sizeof(SkagerChartPointStyleV1) == 48);
namespace {
unsigned checks = 0;
void Check(bool value, const char* message) {
  ++checks;
  if (!value) throw std::runtime_error(message);
}
struct Renderer {
  bool m_bOK = true;
  unsigned actual = SKAGER_CHART_POINT_STYLE_PAPER;
  mutable unsigned reads = 0;
  unsigned GetEffectiveSymbolStyle() const { ++reads; return actual; }
};
SkagerChartPointStyleV1 Request() {
  SkagerChartPointStyleV1 value{};
  value.structBytes = sizeof(value);
  value.version = SKAGER_CHART_POINT_STYLE_VERSION;
  return value;
}
}
int main() {
  try {
    PointStyleObservation observation;
    Renderer renderer;
    auto read = [&](bool selected, const Renderer* source) {
      auto value = Request();
      Check(observation.Read(&value, true, selected, source), "valid request rejected");
      return value;
    };
    auto value = read(true, &renderer);
    Check(!value.available && !value.effectivePointStyle && !renderer.reads,
          "no observation before completed Init");
    observation.SetActive(true);
    value = read(true, &renderer);
    Check(value.available && value.effectivePointStyle == SKAGER_CHART_POINT_STYLE_PAPER,
          "must observe actual Paper, never infer Simplified from selected status");
    Check(DecodeOChartsPointStyle(value).effective_point_style == renderer.actual,
          "host copy differs from renderer");
    renderer.actual = SKAGER_CHART_POINT_STYLE_SIMPLIFIED;
    value = read(true, &renderer);
    Check(value.available && value.effectivePointStyle == renderer.actual && renderer.reads == 2,
          "query must read current renderer value, not cached construction state");
    Check(DecodeOChartsPointStyle(value).available, "host refused observed Simplified");
    const auto reads = renderer.reads;
    for (unsigned mode = 0; mode < 4; ++mode) {
      observation.SetActive(mode != 0); // DeInit/destructor/re-Init gate
      renderer.m_bOK = mode != 1;
      value = read(mode != 2, mode == 3 ? nullptr : &renderer);
      Check(!value.available && !value.effectivePointStyle,
            "inactive/failed/fallback/absent renderer must be unavailable");
      Check(ValidOChartsPointStyle(value), "host rejected a valid unavailable observation");
      Check(!DecodeOChartsPointStyle(value).available, "host invented unavailable value");
      Check(renderer.reads == reads, "unavailable state dereferenced renderer");
    }
    renderer.m_bOK = true;
    observation.SetActive(true);
    value = read(true, &renderer);
    Check(value.available, "same retained renderer unavailable after successful re-Init");
    auto reject = [&](SkagerChartPointStyleV1 bad, bool main_thread = true) {
      const auto before = bad;
      const auto count = renderer.reads;
      Check(!observation.Read(&bad, main_thread, true, &renderer), "malformed/off-thread request accepted");
      Check(!std::memcmp(&bad, &before, sizeof(bad)), "refusal changed caller bytes");
      Check(renderer.reads == count, "refusal read renderer");
    };
    reject(Request(), false);
    value = Request(); --value.structBytes; reject(value);
    value = Request(); ++value.version; reject(value);
    value = Request(); value.available = 1; reject(value);
    value = Request(); value.effectivePointStyle = 76; reject(value);
    for (unsigned i = 0; i < 8; ++i) { value = Request(); value.reserved[i] = 1; reject(value); }
    Check(!observation.Read<Renderer>(nullptr, true, true, &renderer), "null request accepted");
    renderer.actual = 999;
    value = Request(); const auto before = value;
    Check(!observation.Read(&value, true, true, &renderer), "unknown table accepted");
    Check(!std::memcmp(&value, &before, sizeof(value)), "unknown table changed caller bytes");
    // Independently hostile module response must never become host data.
    for (unsigned mutation = 0; mutation < 6; ++mutation) {
      value = Request(); value.available = 1; value.effectivePointStyle = 76;
      switch (mutation) {
        case 0: --value.structBytes; break;
        case 1: ++value.version; break;
        case 2: value.reserved[7] = 1; break;
        case 3: value.available = 2; break;
        case 4: value.effectivePointStyle = 999; break;
        case 5: value.available = 0; break;
      }
      Check(!ValidOChartsPointStyle(value), "malformed observation accepted as valid");
      const auto decoded = DecodeOChartsPointStyle(value);
      Check(!decoded.available && !decoded.effective_point_style, "host accepted malformed module data");
    }
    std::cout << "PASS " << checks << " private observation/host-copy checks; no plugin or renderer launch\n";
  } catch (const std::exception& failure) {
    std::cerr << failure.what() << '\n'; return 1;
  }
}

#pragma once
#include "smartnav/Advisories.h"
#include "ui/Controls.h"
#include <array>

namespace opennav::ui {
// Presentation of already-computed advice only. No navigation or device calls.
class XNavHorizon final : public wxPanel {
 public:
  explicit XNavHorizon(wxWindow *parent);
  void Update(const vessel::VesselState &state,
              const smartnav::NavigationAdvice &advice,
              vessel::Time now, LightMode mode);
 private:
  struct Item { wxString time, title, detail; std::uint32_t color = 0; };
  void Paint(wxPaintEvent &);
  std::array<Item, 4> items_;
  LightMode mode_ = LightMode::Day;
  std::string signature_;
};
} // namespace opennav::ui

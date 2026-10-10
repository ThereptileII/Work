#pragma once
#include "application/PassageView.h"
#include "ui/Controls.h"
#include <functional>
#include <wx/panel.h>

namespace opennav::ui {
// SCRUM-362: the prototype's .next-turn card on the chart. Painted, one
// click target (opens the passage). Hidden by its owner when not visible.
class XNavNextTurnCard final : public wxPanel {
 public:
  XNavNextTurnCard(wxWindow *parent, std::function<void()> open);
  // Returns true when the content or theme changed.
  bool Update(const application::NextTurnView &view, LightMode mode);
  wxSize DoGetBestClientSize() const override;

 private:
  void Paint(wxPaintEvent &);
  application::NextTurnView view_;
  LightMode mode_ = LightMode::Day;
  std::function<void()> open_;
};
}  // namespace opennav::ui

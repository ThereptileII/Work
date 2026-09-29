#pragma once
#include "application/FooterView.h"
#include "ui/Controls.h"
#include <functional>

namespace opennav::ui {
class FooterHealthButton;
// Native counterpart of the immutable prototype .statusbar. It owns only
// presentation values; it cannot reach OpenCPN navigation or renew source age.
class XNavStatusFooter final : public wxPanel {
 public:
  XNavStatusFooter(wxWindow *parent, std::function<void()> health);
  void Update(application::FooterView view, LightMode mode);
  bool Layout() override;
  bool MiddleVisible() const { return middle_visible_; }
  const application::FooterView &View() const { return view_; }
  wxRect LeftRegion() const { return left_; }
  wxRect MiddleRegion() const { return middle_; }
 private:
  void Paint(wxPaintEvent &);
  application::FooterView view_;
  LightMode mode_ = LightMode::Day;
  FooterHealthButton *health_;
  wxRect left_, middle_;
  bool middle_visible_ = true;
};
} // namespace opennav::ui

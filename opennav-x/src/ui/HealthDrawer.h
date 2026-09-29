#pragma once
#include "application/SourceHealthView.h"
#include "ui/Drawer.h"

namespace opennav::ui {
struct HealthDrawerActions {
  std::function<void(const application::HealthSignal &)> configure;
  std::function<void()> manage, diagnostics;
};
class XNavHealthDrawer final : public XNavDrawer {
public:
  XNavHealthDrawer(wxWindow &, HealthDrawerActions);
  void Update(application::SourceHealthView, LightMode);
private:
  void Build();
  void Toggle(std::size_t);
  HealthDrawerActions actions_;
  application::SourceHealthView view_;
  std::vector<XNavButton *> summaries_, configure_;
  std::vector<wxPanel *> details_;
  std::vector<bool> expanded_;
  wxPanel *intro_ = nullptr;
  XNavButton *manage_ = nullptr, *diagnostics_ = nullptr;
};
} // namespace opennav::ui

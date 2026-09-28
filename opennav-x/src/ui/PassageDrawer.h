#pragma once
#include "application/NavigationObjects.h"
#include "application/PassageView.h"
#include "ui/Drawer.h"
#include <wx/datetime.h>

namespace opennav::ui {
class XNavPassageDrawer final : public XNavDrawer {
public:
  XNavPassageDrawer(wxWindow &owner, application::NavigationActions actions);
  void Update(const vessel::VesselState &, const smartnav::NavigationAdvice &,
              const smartnav::EnergyPrediction &, vessel::Time, LightMode,
              wxDateTime wall_now = wxDateTime::Now());
  std::function<void()> on_library, on_plot;
  const application::PassageView &View() const { return view_; }

private:
  void Build();
  void Paint(wxPaintEvent &);
  void EndNavigation();
  application::NavigationActions actions_;
  application::PassageView view_;
  std::optional<application::Route> selected_;
  wxPanel *summary_ = nullptr;
  XNavButton *end_ = nullptr, *reverse_ = nullptr, *plot_ = nullptr;
  std::vector<XNavButton *> buttons_;
  std::vector<std::pair<std::string, XNavButton *>> edits_;
  bool changes_allowed_ = false;
  std::size_t rows_ = 0;
  wxDateTime wall_now_;
};
} // namespace opennav::ui

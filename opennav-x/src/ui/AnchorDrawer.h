#pragma once
#include "application/AnchorView.h"
#include "ui/Drawer.h"
#include "ui/Range.h"

namespace opennav::ui {
class XNavAnchorDrawer final : public XNavDrawer {
public:
  XNavAnchorDrawer(wxWindow &owner,application::NavigationActions actions);
  void Update(const application::AnchorState &,const vessel::VesselState &,
              vessel::Time,LightMode);
  const application::AnchorView &View() const { return view_; }
private:
  void Paint(wxPaintEvent &);
  void Command();
  application::NavigationActions actions_;
  application::AnchorView view_;
  wxPanel *summary_=nullptr,*note_=nullptr;
  XNavRange *radius_=nullptr;
  XNavButton *watch_=nullptr;
  bool changes_allowed_=false;
};
}

#pragma once
#include "application/HorizonView.h"
#include "ui/Controls.h"
#include <functional>

namespace opennav::ui {
class HorizonButton;
struct HorizonGeometry {
  wxRect horizon, heading, full_passage;
  std::array<wxRect,4> events;
};
// Native presentation only. Shell revalidates copied actions at activation.
class XNavHorizon final : public wxPanel {
 public:
  explicit XNavHorizon(wxWindow *parent,std::function<void()> passage={},
      std::function<void(const application::HorizonAction &)> activate={});
  void Update(application::HorizonView view,LightMode mode);
  bool Layout() override;
  HorizonGeometry Geometry() const;
  const application::HorizonView &View() const {return view_;}
  std::uint64_t PresentationChanges() const {return presentation_changes_;}
 private:
  void Paint(wxPaintEvent &);
  application::HorizonView view_;
  HorizonButton *passage_=nullptr;
  std::array<HorizonButton *,4> events_{};
  LightMode mode_=LightMode::Day;
  wxRect heading_;
  int inset_=25,event_y_=59,gap_=4,title_size_=13,detail_size_=10;
  bool narrow_=false,short_=false,mobile_=false;
  std::uint64_t presentation_changes_=0;
};
} // namespace opennav::ui

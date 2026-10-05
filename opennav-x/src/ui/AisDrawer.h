#pragma once
#include "application/OnlineAis.h"
#include "ui/Drawer.h"
#include "ui/ListView.h"
#include "ui/Range.h"
#include <wx/weakref.h>

namespace opennav::ui {
class XNavAisDrawer final : public XNavDrawer {
public:
  XNavAisDrawer(wxWindow &owner, application::OnlineAisActions actions,
                std::function<void(int)> show_on_chart);
  void Update(const vessel::AisState &display,
              const application::OnlineAisState &online, vessel::Time now,
              LightMode light);
  void List();
  void ShowSettings() { view_ = View::Settings; message_.clear(); Build(); }
  void Target(int mmsi);
  std::string PageTitle() const;
  int FilterEvent(wxEvent &event) override;
  std::function<void(int)> on_select;

private:
  enum class View { List, Target, Settings };
  void Build(bool reset_scroll = true);
  void CancelDrag();
  void ScrollWheel(const wxMouseEvent &event);
  void RefreshValues();
  void AddText(const wxString &text, int size = 12);
  XNavButton *Button(const wxString &, std::function<void()>,
                     bool enabled = true);
  void AddVisual(int height, std::function<void(XNavPainter &, int)>, int after = 16);
  std::optional<vessel::AisTarget> Selected() const;
  void StoreKey();
  View view_ = View::List;
  bool sort_range_ = false;
  int mmsi_ = 0;
  vessel::AisState display_;
  application::OnlineAisState online_;
  application::OnlineAisActions actions_;
  std::function<void(int)> show_on_chart_;
  vessel::Time now_{};
  XNavListView *list_ = nullptr;
  std::vector<wxPanel *> visuals_;
  std::vector<XNavButton *> buttons_;
  XNavButton *range_ = nullptr, *cpa_ = nullptr, *show_ = nullptr;
  XNavRange *radius_ = nullptr;
  XNavButton *apply_radius_ = nullptr;
  wxString message_;
  wxWeakRef<wxWindow> drag_origin_;
  wxPoint drag_start_, drag_last_;
  bool dragging_ = false;
  bool forwarding_drag_ = false;
  double wheel_remainder_ = 0;
};
} // namespace opennav::ui

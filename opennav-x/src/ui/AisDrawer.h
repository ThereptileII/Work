#pragma once
#include "application/OnlineAis.h"
#include "ui/Drawer.h"
#include "ui/ListView.h"

namespace opennav::ui {
class XNavAisDrawer final : public XNavDrawer {
public:
  XNavAisDrawer(wxWindow &owner, application::OnlineAisActions actions,
                std::function<void(int)> show_on_chart);
  void Update(const vessel::AisState &display,
              const application::OnlineAisState &online, vessel::Time now,
              LightMode light);
  void List();
  void Target(int mmsi);
  std::function<void(int)> on_select;

private:
  enum class View { List, Target, Settings };
  void Build();
  void RefreshValues();
  void AddText(const wxString &text, int size = 12);
  XNavButton *Button(const wxString &, std::function<void()>,
                     bool enabled = true);
  void AddVisual(int height, std::function<void(XNavPainter &, int)>);
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
  wxString message_;
};
} // namespace opennav::ui

#pragma once
#include "application/RouteContext.h"
#include "ui/Controls.h"
#include <wx/dialog.h>
#include <wx/eventfilter.h>

namespace opennav::ui {
enum class RouteContextAction { ViewOnChart, Details, Activate, Stop };
class XNavRouteContextCard final : public wxDialog, public wxEventFilter {
public:
  using Action = std::function<void(RouteContextAction, const std::string &)>;
  XNavRouteContextCard(wxWindow &owner, std::string selected_id, Action action,
                       std::function<void()> dismissed = {});
  ~XNavRouteContextCard() override;
  void UpdateRoute(const std::optional<application::Route> &route, LightMode mode);
  bool Place(const wxRect &chart);
  void Dismiss();
  int FilterEvent(wxEvent &) override;
  const application::RouteContextView &View() const { return view_; }
private:
  void Paint(wxPaintEvent &);
  application::RouteContextView view_;
  std::string selected_id_;
  Action action_;
  std::function<void()> dismissed_;
  wxPanel *content_ = nullptr;
  XNavIconButton *close_ = nullptr;
  XNavButton *view_button_ = nullptr, *details_button_ = nullptr;
  // Activate an inactive route, or stop the active one; never both.
  XNavButton *navigate_button_ = nullptr;
  LightMode light_ = LightMode::Day;
  bool closing_ = false, filter_added_ = false;
};
} // namespace opennav::ui

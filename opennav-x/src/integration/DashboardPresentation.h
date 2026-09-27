#pragma once

#include <wx/aui/aui.h>
#include <wx/weakref.h>
#include <optional>
#include <vector>

namespace opennav::integration {
void EnableDashboardPresentation();
void FinishDashboardPresentation();
// Application-thread presentation only. Registration comes from the bundled
// Dashboard's actual pane windows, never translated captions or pane names.
// The plugin continues receiving data. Neither this state nor suppression is
// persisted; Legacy owns the normal workspace.
class DashboardPresentation {
 public:
  explicit DashboardPresentation(wxAuiManager& manager) : manager_(manager) {}
  ~DashboardPresentation();
  void Register(wxWindow* window);
  void Unregister(wxWindow* window);
  void Enable(bool enabled);
  void BeginLayout();
  void EndLayout();
  bool Enabled() const { return enabled_; }
 private:
  struct Pane {
    explicit Pane(wxWindow* value) : window(value) {}
    wxWeakRef<wxWindow> window;
    std::optional<wxAuiPaneInfo> original;
  };
  void Restore();
  void Suppress();
  void Prune();
  wxAuiManager& manager_;
  std::vector<Pane> panes_;
  unsigned depth_ = 0;
  bool enabled_ = false;
};
}  // namespace opennav::integration

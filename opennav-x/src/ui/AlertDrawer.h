#pragma once
#include "application/Alerts.h"
#include "ui/Drawer.h"

namespace opennav::ui {
struct AlertDrawerActions {
  std::function<void(application::AlertArea)> inspect;
  std::function<void(const std::string &, std::uint64_t)> acknowledge;
};
// Owns copied presentation episodes only. Acknowledgement never controls
// equipment, clears a condition or acknowledges an upstream OpenCPN alarm.
class XNavAlertDrawer final : public XNavDrawer {
public:
  XNavAlertDrawer(wxWindow &, AlertDrawerActions);
  void Update(const std::vector<application::Alert> &, bool replay, LightMode);

private:
  void Rebuild();
  AlertDrawerActions actions_;
  std::vector<application::Alert> alerts_;
  bool replay_ = false, built_ = false, queued_ = false;
};
} // namespace opennav::ui

#pragma once
// Application-thread boundary for SKAGER Cloud (SCRUM-44, SCRUM-369). Mirrors
// OnlineWeather: the only preference in wxFileConfig is whether boat data is
// shared (default off); the device token lives in platform secret storage.
#include "application/NavigationObjects.h"
#include "cloud/CloudService.h"
#include <memory>
class wxFileConfig;

namespace opennav::integration {
class OnlineCloud final {
public:
  OnlineCloud(wxFileConfig &config, std::unique_ptr<cloud::CloudService> service);
  ~OnlineCloud();
  application::CommandResult Link();
  application::CommandResult CancelLink();
  application::CommandResult Unlink();
  application::CommandResult Share(bool enabled);
  void Refresh() { service_->RefreshStatus(); }
  // Live, measured vessel state only; the service enforces cadence and gates.
  void Observe(const vessel::VesselState &state, vessel::Time now);
  // After each OpenCPN anchor-watch pass (SCRUM-387): OpenCPN's own watch and
  // alarm, copied for SKAGER Companion. Live operation only.
  void ObserveAnchor(const application::AnchorState &anchor, vessel::Time now);
  // Active critical XNav alerts as (id, title), excluding the anchor alarm.
  void ObserveAlerts(const std::vector<std::pair<std::string, std::string>> &critical, vessel::Time now);
  cloud::CloudSnapshot Read();

private:
  wxFileConfig &config_;
  std::unique_ptr<cloud::CloudService> service_;
  bool saved_share_ = false;
};
}  // namespace opennav::integration

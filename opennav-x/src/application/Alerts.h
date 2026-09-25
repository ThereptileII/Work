#pragma once
#include "adapters/Autopilot.h"
#include "application/NavigationObjects.h"
#include "smartnav/Energy.h"
#include <map>
#include <set>

namespace opennav::application {
enum class AlertLevel { Info, Advisory, Warning, Critical };
enum class AlertArea { Sources, Ais, Anchor, Energy, Pilot };
struct Alert {
  std::string id, title, action, source;
  AlertLevel level = AlertLevel::Info;
  AlertArea area = AlertArea::Sources;
  vessel::Time first_observed{};
  std::uint64_t episode = 0;
  bool acknowledged = false;
};
struct AlertInput {
  vessel::VesselState vessel;
  vessel::AisState ais;
  AnchorState anchor;
  smartnav::EnergyPrediction energy;
  adapters::PilotView pilot;
  vessel::Time now{};
};
// Presentation only. No adapter reference, command, acknowledgement of upstream
// alarms, or persistence of a simulated alert into live operation.
class AlertCenter {
public:
  void Observe(const AlertInput &);
  bool Acknowledge(const std::string &id, std::uint64_t episode);
  const std::vector<Alert> &Current() const { return current_; }

private:
  std::vector<Alert> current_;
  std::set<std::string> seen_;
  std::string mode_;
  vessel::Time previous_{};
  bool clock_started_ = false, pilot_engaged_ = false;
  std::uint64_t next_episode_ = 1;
};
const char *AlertLevelName(AlertLevel);
} // namespace opennav::application

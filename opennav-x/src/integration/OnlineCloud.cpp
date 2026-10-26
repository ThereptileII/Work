#include "integration/OnlineCloud.h"
#include <wx/fileconf.h>
#include <wx/log.h>
#include <wx/thread.h>
#include <wx/utils.h>

namespace opennav::integration {
namespace {
// Separate from the versioned vessel settings and from settings backups.
constexpr const char *share_key = "/OpenNav/Cloud/v1/ShareBoatData";
}  // namespace

OnlineCloud::OnlineCloud(wxFileConfig &config, std::unique_ptr<cloud::CloudService> service)
    : config_(config), service_(std::move(service)) {
  wxASSERT(wxIsMainThread());
  wxString value;
  const bool share = config_.Read(share_key, &value) && value == "1";
  if (!value.empty() && value != "0" && value != "1")
    wxLogWarning("SKAGER Cloud sharing setting invalid; boat data sharing off");
  service_->SetShareEnabled(share);
  saved_share_ = share;
}
OnlineCloud::~OnlineCloud() = default;

cloud::CloudSnapshot OnlineCloud::Read() {
  wxASSERT(wxIsMainThread());
  auto snapshot = service_->Read();
  // Revocation and missing credentials also withdraw this link's consent.
  // Persist on the application thread, including while the settings page is closed.
  if (snapshot.initialized && !snapshot.share_enabled && saved_share_) {
    if (config_.Write(share_key, wxString("0")) && config_.Flush()) saved_share_ = false;
    else snapshot.message = "Sharing is off; the saved sharing preference could not be cleared.";
  }
  return snapshot;
}

void OnlineCloud::Observe(const vessel::VesselState &state, vessel::Time now) {
  Read();
  service_->Observe(state, now);
}

void OnlineCloud::ObserveAnchor(const application::AnchorState &anchor, vessel::Time now) {
  cloud::AnchorReport report;
  // A watch exists when OpenCPN has a watched mark with a (signed) radius.
  report.armed = !anchor.waypoint_id.empty() && anchor.anchor && anchor.radius_m && *anchor.radius_m != 0;
  if (report.armed) {
    report.alarm = anchor.alarm;
    report.latitude_deg = anchor.anchor->latitude_deg;
    report.longitude_deg = anchor.anchor->longitude_deg;
    report.radius_m = anchor.radius_m;
    report.source = anchor.source;
    // Distance only from a current position, with that position's receive time.
    const auto assessed = vessel::Assess(anchor.distance_m, now);
    if (assessed.value && (assessed.quality == vessel::Quality::Live || assessed.quality == vessel::Quality::Aging)) {
      report.distance_m = *assessed.value;
      report.distance_observed_at = anchor.distance_m.observed_at;
    }
  }
  service_->ObserveAnchor(report, now);
}

void OnlineCloud::ObserveAlerts(const std::vector<std::pair<std::string, std::string>> &critical, vessel::Time now) {
  service_->ObserveAlerts(critical, now);
}

application::CommandResult OnlineCloud::Link() {
  if (!wxIsMainThread()) return {false, "SKAGER Cloud unavailable"};
  const auto state = service_->Read().link;
  if (state == cloud::LinkState::Linked) return {false, "This SKAGER is already linked"};
  if (state != cloud::LinkState::Unlinked) return {true, "Pairing already in progress"};
  const auto off = Share(false);
  if (!off.ok) return off;
  // The name shown in the web account when approving and in the device list.
  auto host = wxGetHostName().ToStdString();
  service_->StartLink(host.empty() ? "SKAGER" : "SKAGER on " + host);
  wxLogMessage("SKAGER Cloud: pairing requested by user");
  return {true, "Requesting a pairing code"};
}

application::CommandResult OnlineCloud::CancelLink() {
  if (!wxIsMainThread()) return {false, "SKAGER Cloud unavailable"};
  service_->CancelLink();
  return {true, "Pairing cancelled"};
}

application::CommandResult OnlineCloud::Unlink() {
  if (!wxIsMainThread()) return {false, "SKAGER Cloud unavailable"};
  const auto off = Share(false);
  service_->Unlink();
  wxLogMessage("SKAGER Cloud: unlink requested by user");
  if (!off.ok) return {false, "Unlink requested; the saved sharing preference could not be cleared"};
  return {true, "Unlinking this SKAGER"};
}

application::CommandResult OnlineCloud::Share(bool enabled) {
  if (!wxIsMainThread()) return {false, "SKAGER Cloud unavailable"};
  const auto snapshot = service_->Read();
  if (enabled && (snapshot.link != cloud::LinkState::Linked || snapshot.unlink_pending))
    return {false, "Link this SKAGER before enabling boat data sharing"};
  // OFF is immediate even if the preference cannot be saved.
  if (!enabled) service_->SetShareEnabled(false);
  wxString previous;
  const bool existed = config_.Read(share_key, &previous);
  if (!config_.Write(share_key, wxString(enabled ? "1" : "0")) || !config_.Flush()) {
    if (existed) config_.Write(share_key, previous);
    else config_.DeleteEntry(share_key);
    config_.Flush();
    return {false, enabled ? "Sharing setting could not be saved"
                           : "Sharing stopped for this session; OFF could not be saved"};
  }
  service_->SetShareEnabled(enabled);
  saved_share_ = enabled;
  wxLogMessage("SKAGER Cloud boat data: %s", enabled ? "shared by user choice" : "not shared");
  return {true, enabled ? "Boat data sharing on" : "Boat data sharing off"};
}
}  // namespace opennav::integration

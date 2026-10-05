#include "integration/OnlineAis.h"
#include "integration/AisViewport.h"
#include <algorithm>
#include <wx/fileconf.h>
#include <wx/log.h>
#include <wx/thread.h>

namespace opennav::integration {
namespace {
// Separate from the versioned vessel configuration so older XNav versions
// never reject the vessel settings because an online-only option was added.
constexpr const char *enabled_key = "/OpenNav/OnlineAIS/v1/Enabled";
constexpr const char *radius_key = "/OpenNav/OnlineAIS/v1/RadiusNm";
}
OnlineAis::OnlineAis(wxFileConfig &config, std::unique_ptr<ais::IAisCredentials> credentials,
                     std::unique_ptr<ais::IOnlineAisProvider> provider)
    : config_(config), credentials_(std::move(credentials)), provider_(std::move(provider)) {
  wxASSERT(wxIsMainThread());
  wxString value;
  enabled_ = config_.Read(enabled_key, &value) && value == "1";
  // Do not log invalid external settings or credential contents.
  if (!value.empty() && value != "0" && value != "1")
    wxLogWarning("SKAGER Online AIS setting invalid; networking disabled");
  wxString radius_text;
  long radius = 0;
  if (config_.Read(radius_key, &radius_text) && radius_text.ToLong(&radius) &&
      radius >= ais::MinimumRadiusNm && radius <= ais::MaximumRadiusNm)
    radius_nm_ = static_cast<int>(radius);
  credential_present_ = credentials_->Read().status == ais::CredentialStatus::Ready;
  // Construction does not connect. Normal application processing must first
  // supply the actual viewport and confirm that live operation is permitted.
}
OnlineAis::~OnlineAis() = default;
void OnlineAis::ObserveViewport(std::optional<ais::Viewport> viewport, bool live_allowed) {
  wxASSERT(wxIsMainThread());
  if (!wxIsMainThread()) return;
  viewport_ = viewport;
  live_allowed_ = live_allowed;
  radius_area_ = viewport ? AisRadiusViewport(*viewport, radius_nm_) : std::nullopt;
  const bool allowed = enabled_ && live_allowed && radius_area_ &&
      provider_->ObserveViewport(*radius_area_);
  provider_->SetEnabled(allowed);
}
ais::ProviderSnapshot OnlineAis::Read(vessel::Time now) const {
  auto value = provider_->Read(now);
  auto &targets = value.targets.targets;
  value.cached_position_count = static_cast<std::uint32_t>(targets.size());
  if (!enabled_ || !live_allowed_ || !radius_area_ || !radius_area_->center) {
    targets.clear();
    return value;
  }
  const auto center = *radius_area_->center;
  targets.erase(std::remove_if(targets.begin(),targets.end(),[&](const auto &target) {
    return !target.latitude_deg.value || !target.longitude_deg.value ||
        !WithinAisRadius(center,radius_nm_,*target.latitude_deg.value,*target.longitude_deg.value);
  }),targets.end());
  return value;
}
application::CommandResult OnlineAis::SetRadiusNm(int radius_nm) {
  if (!wxIsMainThread()) return {false, "Online AIS settings unavailable"};
  if (radius_nm < ais::MinimumRadiusNm || radius_nm > ais::MaximumRadiusNm)
    return {false, "Choose an AIS radius from 1 to 200 nm"};
  if (radius_nm == radius_nm_) return {true, "AIS radius unchanged"};
  wxString previous;
  const bool existed = config_.Read(radius_key,&previous);
  if (!config_.Write(radius_key,wxString::Format("%d",radius_nm)) || !config_.Flush()) {
    if (existed) config_.Write(radius_key,previous); else config_.DeleteEntry(radius_key);
    config_.Flush();
    return {false, "AIS radius could not be saved; previous radius retained"};
  }
  radius_nm_ = radius_nm;
  ObserveViewport(viewport_,live_allowed_);
  return {true, "AIS radius saved; subscription update queued"};
}
application::CommandResult OnlineAis::Enable(bool enabled) {
  if (!wxIsMainThread()) return {false, "Online AIS settings unavailable"};
  // OFF is immediate even if profile storage fails. Never leave networking
  // running because a user's disable request could not be persisted.
  if (!enabled) { enabled_ = false; provider_->SetEnabled(false); }
  wxString previous;
  const bool existed = config_.Read(enabled_key, &previous);
  if (!config_.Write(enabled_key, wxString(enabled ? "1" : "0")) || !config_.Flush()) {
    if (existed) config_.Write(enabled_key, previous);
    else config_.DeleteEntry(enabled_key);
    const bool restored = config_.Flush();
    if (!enabled) return {false, "Online AIS stopped for this session; OFF could not be saved. Check profile storage before restarting."};
    return {false, restored ? "Online AIS setting could not be saved"
                            : "Online AIS setting could not be saved; check profile storage"};
  }
  enabled_ = enabled;
  wxLogMessage("SKAGER Online AIS: %s", enabled ? "enabled by user" : "disabled by user");
  return {true, enabled ? "Online AIS enabled; supplemental internet traffic"
                        : "Online AIS disabled"};
}
application::CommandResult OnlineAis::StoreKey(const ais::Secret &key) {
  if (!wxIsMainThread()) return {false, "Credential storage unavailable"};
  const auto result = credentials_->Store(key);
  credential_present_ = credentials_->Read().status == ais::CredentialStatus::Ready;
  if (result == ais::CredentialStatus::Ready && credential_present_) {
    provider_->CredentialChanged();
    return {true, "AISStream key saved in Windows Credential Manager"};
  }
  if (result == ais::CredentialStatus::ReadOnly)
    return {false, "This development build reads AISSTREAM_API_KEY from its environment"};
  return {false, "AISStream key could not be saved"};
}
application::CommandResult OnlineAis::RemoveKey() {
  if (!wxIsMainThread()) return {false, "Credential storage unavailable"};
  const auto disabled = Enable(false);
  const auto result = credentials_->Remove();
  credential_present_ = credentials_->Read().status == ais::CredentialStatus::Ready;
  provider_->CredentialChanged();
  if (result == ais::CredentialStatus::Removed)
    return {true, disabled.ok ? "AISStream key removed; Online AIS disabled"
                             : "AISStream key removed; OFF preference could not be saved"};
  if (result == ais::CredentialStatus::ReadOnly)
    return {false, "Remove AISSTREAM_API_KEY from the development environment"};
  return {false, "Key removal could not be verified; Online AIS remains disabled"};
}
} // namespace opennav::integration

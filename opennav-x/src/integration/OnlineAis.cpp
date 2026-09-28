#include "integration/OnlineAis.h"
#include <wx/fileconf.h>
#include <wx/log.h>
#include <wx/thread.h>

namespace opennav::integration {
namespace {
// Separate from the versioned vessel configuration so older XNav versions
// never reject the vessel settings because an online-only option was added.
constexpr const char *enabled_key = "/OpenNav/OnlineAIS/v1/Enabled";
}
OnlineAis::OnlineAis(wxFileConfig &config, std::unique_ptr<ais::IAisCredentials> credentials,
                     std::unique_ptr<ais::IOnlineAisProvider> provider)
    : config_(config), credentials_(std::move(credentials)), provider_(std::move(provider)) {
  wxASSERT(wxIsMainThread());
  wxString value;
  enabled_ = config_.Read(enabled_key, &value) && value == "1";
  // Do not log invalid external settings or credential contents.
  if (!value.empty() && value != "0" && value != "1")
    wxLogWarning("OpenNav Online AIS setting invalid; networking disabled");
  credential_present_ = credentials_->Read().status == ais::CredentialStatus::Ready;
  // Construction does not connect. Normal application processing must first
  // supply the actual viewport and confirm that live operation is permitted.
}
OnlineAis::~OnlineAis() = default;
void OnlineAis::ObserveViewport(std::optional<ais::Viewport> viewport, bool live_allowed) {
  wxASSERT(wxIsMainThread());
  if (!wxIsMainThread()) return;
  const bool allowed = enabled_ && live_allowed && viewport &&
      provider_->ObserveViewport(*viewport);
  provider_->SetEnabled(allowed);
}
ais::ProviderSnapshot OnlineAis::Read(vessel::Time now) const {
  return provider_->Read(now);
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
  wxLogMessage("OpenNav Online AIS: %s", enabled ? "enabled by user" : "disabled by user");
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

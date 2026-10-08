#include "integration/OnlineWeather.h"
#include "weather/GribStream.h"
#include <wx/fileconf.h>
#include <wx/log.h>
#include <wx/thread.h>

namespace opennav::integration {
namespace {
// Separate from the versioned vessel settings and from settings backups.
// The token is never written here (platform secret storage only).
constexpr const char *enabled_key = "/OpenNav/Weather/v1/Enabled";
constexpr const char *model_key = "/OpenNav/Weather/v1/Model";
#ifdef __WXMSW__
constexpr const char *saved_message = "GRIBstream token saved in Windows Credential Manager";
#else
constexpr const char *saved_message = "GRIBstream token saved";
#endif
} // namespace

OnlineWeather::OnlineWeather(wxFileConfig &config,
                             std::unique_ptr<ais::IAisCredentials> credentials,
                             std::unique_ptr<weather::IForecastProvider> provider)
    : config_(config), credentials_(std::move(credentials)), provider_(std::move(provider)),
      model_(weather::gribstream::kDefaultModel) {
  wxASSERT(wxIsMainThread());
  wxString value;
  enabled_ = config_.Read(enabled_key, &value) && value == "1";
  if (!value.empty() && value != "0" && value != "1")
    wxLogWarning("SKAGER weather setting invalid; forecast networking disabled");
  wxString model;
  if (config_.Read(model_key, &model)) {
    const auto text = model.ToStdString();
    if (weather::gribstream::ValidModel(text)) model_ = text;
    else wxLogWarning("SKAGER weather model setting invalid; using gfs");
  }
  token_present_ = credentials_->Read().status == ais::CredentialStatus::Ready;
  provider_->SetModel(model_);
  provider_->CredentialChanged(token_present_);
  provider_->SetEnabled(enabled_);
}
OnlineWeather::~OnlineWeather() = default;

application::CommandResult OnlineWeather::Enable(bool enabled) {
  if (!wxIsMainThread()) return {false, "Weather settings unavailable"};
  // OFF is immediate even if the preference cannot be saved.
  if (!enabled) {
    enabled_ = false;
    provider_->SetEnabled(false);
  }
  wxString previous;
  const bool existed = config_.Read(enabled_key, &previous);
  if (!config_.Write(enabled_key, wxString(enabled ? "1" : "0")) || !config_.Flush()) {
    if (existed) config_.Write(enabled_key, previous);
    else config_.DeleteEntry(enabled_key);
    config_.Flush();
    return {false, enabled ? "Weather setting could not be saved"
                           : "Weather forecast stopped for this session; OFF could not be saved"};
  }
  enabled_ = enabled;
  provider_->SetEnabled(enabled);
  wxLogMessage("SKAGER weather forecast: %s", enabled ? "enabled by user" : "disabled by user");
  if (enabled && !token_present_)
    return {true, "Weather forecast enabled; add a GRIBstream API token to fetch data"};
  return {true, enabled ? "Weather forecast enabled; internet forecast data"
                        : "Weather forecast disabled"};
}

application::CommandResult OnlineWeather::StoreToken(const ais::Secret &token) {
  if (!wxIsMainThread()) return {false, "Credential storage unavailable"};
  if (token.Empty()) return {false, "Enter a GRIBstream API token"};
  const auto result = credentials_->Store(token);
  token_present_ = credentials_->Read().status == ais::CredentialStatus::Ready;
  provider_->CredentialChanged(token_present_);
  if (result == ais::CredentialStatus::Ready && token_present_) return {true, saved_message};
  if (result == ais::CredentialStatus::ReadOnly)
    return {false, "This development build reads SKAGER_GRIBSTREAM_TOKEN from its environment"};
  return {false, "GRIBstream token could not be saved"};
}

application::CommandResult OnlineWeather::RemoveToken() {
  if (!wxIsMainThread()) return {false, "Credential storage unavailable"};
  const auto result = credentials_->Remove();
  token_present_ = credentials_->Read().status == ais::CredentialStatus::Ready;
  provider_->CredentialChanged(token_present_);
  if (result == ais::CredentialStatus::Removed) return {true, "GRIBstream token removed"};
  if (result == ais::CredentialStatus::ReadOnly)
    return {false, "Remove SKAGER_GRIBSTREAM_TOKEN from the development environment"};
  return {false, "Token removal could not be verified"};
}

application::CommandResult OnlineWeather::TestConnection() {
  if (!wxIsMainThread()) return {false, "Weather settings unavailable"};
  if (!token_present_) return {false, "No GRIBstream API token stored"};
  provider_->Test();
  return {true, "Testing GRIBstream connection"};
}

void OnlineWeather::Request(const weather::ForecastQuery &query) {
  if (enabled_) provider_->SetQuery(query);
}
weather::ForecastSnapshot OnlineWeather::Read(weather::WallTime now) const {
  return provider_->Read(now);
}
weather::ConnectionTest OnlineWeather::LastTest() const { return provider_->LastTest(); }
} // namespace opennav::integration

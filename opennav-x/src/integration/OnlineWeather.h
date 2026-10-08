#pragma once
// Application-thread settings/lifecycle boundary for the GRIBstream forecast
// (SCRUM-324/325). Mirrors OnlineAis: preferences in wxFileConfig (enabled
// flag and model only, never the token), token in platform secret storage.
#include "ais/Credentials.h"
#include "application/NavigationObjects.h"
#include "weather/ForecastService.h"
#include <memory>
class wxFileConfig;

namespace opennav::integration {
class OnlineWeather final {
public:
  OnlineWeather(wxFileConfig &config, std::unique_ptr<ais::IAisCredentials> credentials,
                std::unique_ptr<weather::IForecastProvider> provider);
  ~OnlineWeather();
  bool Enabled() const { return enabled_; }
  bool TokenPresent() const { return token_present_; }
  const std::string &Model() const { return model_; }
  application::CommandResult Enable(bool enabled);
  application::CommandResult StoreToken(const ais::Secret &token);
  application::CommandResult RemoveToken();
  application::CommandResult TestConnection();
  void Request(const weather::ForecastQuery &query);
  weather::ForecastSnapshot Read(weather::WallTime now) const;
  weather::ConnectionTest LastTest() const;
  bool TestPending() const { return provider_->TestPending(); }

private:
  wxFileConfig &config_;
  std::unique_ptr<ais::IAisCredentials> credentials_;
  std::unique_ptr<weather::IForecastProvider> provider_;
  bool enabled_ = false, token_present_ = false;
  std::string model_;
};
} // namespace opennav::integration

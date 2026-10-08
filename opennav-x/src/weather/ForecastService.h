#pragma once
// Background forecast worker. All network I/O and credential reads happen on
// its own thread; every public call returns immediately (UI never blocks).
// The transport is injected so the lifecycle is testable without a network.
#include "ais/Credentials.h"
#include "weather/ForecastSession.h"
#include <memory>

namespace opennav::weather {
class IForecastTransport {
public:
  virtual ~IForecastTransport() = default;
  // Blocking HTTPS POST on the worker thread. Must honour its own connect /
  // transfer timeouts and the response size bound, and return promptly after
  // Cancel(). Never logs the token, headers or body.
  virtual HttpResult Post(const std::string &url, const std::string &body,
                          const ais::Secret &token) = 0;
  virtual void Cancel() = 0;  // Any thread; aborts the current Post.
};

class IForecastProvider {
public:
  virtual ~IForecastProvider() = default;
  virtual void SetEnabled(bool enabled) = 0;
  virtual void SetModel(const std::string &model) = 0;
  virtual void SetQuery(const ForecastQuery &query) = 0;
  virtual void CredentialChanged(bool present) = 0;
  virtual void Test() = 0;
  virtual ForecastSnapshot Read(WallTime now) const = 0;
  virtual ConnectionTest LastTest() const = 0;
  virtual bool TestPending() const = 0;
};

class ForecastService final : public IForecastProvider {
public:
  ForecastService(std::unique_ptr<ais::IAisCredentials> credentials,
                  std::unique_ptr<IForecastTransport> transport);
  ~ForecastService() override;  // Cancels the transport and joins.
  ForecastService(const ForecastService &) = delete;
  ForecastService &operator=(const ForecastService &) = delete;
  void SetEnabled(bool enabled) override;
  void SetModel(const std::string &model) override;
  void SetQuery(const ForecastQuery &query) override;
  void CredentialChanged(bool present) override;
  void Test() override;
  ForecastSnapshot Read(WallTime now) const override;
  ConnectionTest LastTest() const override;
  bool TestPending() const override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace opennav::weather

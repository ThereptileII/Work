#include "ais/Credentials.h"
#include <cstdlib>
#include <mutex>
#include <string>

namespace opennav::ais {
namespace {
class EnvironmentCredentials final : public IAisCredentials {
public:
  explicit EnvironmentCredentials(const char *variable) : variable_(variable) {}
  CredentialResult Read() const override {
    CredentialResult result;
    const char *value = std::getenv(variable_);
    if (!value || !*value)
      return result;
    // Bound inspection before constructing a view/string from external input.
    std::size_t size = 0;
    while (size <= 512 && value[size])
      ++size;
    result.status = size <= 512 && result.key.Assign({value, size})
                        ? CredentialStatus::Ready
                        : CredentialStatus::Invalid;
    return result;
  }
  CredentialStatus Store(const Secret &) override {
    return CredentialStatus::ReadOnly;
  }
  CredentialStatus Remove() override { return CredentialStatus::ReadOnly; }

private:
  const char *variable_;
};
// Development stand-in for the SKAGER Cloud link: pairing works within one
// session; nothing is persisted. One instance per process, shared by readers.
class MemoryCredentials final : public IAisCredentials {
public:
  CredentialResult Read() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    CredentialResult result;
    if (!seeded_) {
      seeded_ = true;
      auto env = EnvironmentCredentials("SKAGER_CLOUD_TOKEN").Read();
      if (env.status == CredentialStatus::Ready) value_ = std::string(env.key.View());
    }
    if (!value_.empty() && result.key.Assign(value_)) result.status = CredentialStatus::Ready;
    return result;
  }
  CredentialStatus Store(const Secret &key) override {
    std::lock_guard<std::mutex> lock(mutex_);
    seeded_ = true;
    value_ = std::string(key.View());
    return CredentialStatus::Ready;
  }
  CredentialStatus Remove() override {
    std::lock_guard<std::mutex> lock(mutex_);
    seeded_ = true;
    value_.clear();
    return CredentialStatus::Removed;
  }

private:
  static inline std::mutex mutex_;
  static inline bool seeded_ = false;
  static inline std::string value_;
};
} // namespace
std::unique_ptr<IAisCredentials> CreateAisCredentials() {
  return std::make_unique<EnvironmentCredentials>("AISSTREAM_API_KEY");
}
std::unique_ptr<IAisCredentials> CreateWeatherCredentials() {
  return std::make_unique<EnvironmentCredentials>("SKAGER_GRIBSTREAM_TOKEN");
}
std::unique_ptr<IAisCredentials> CreateCloudCredentials() {
  return std::make_unique<MemoryCredentials>();
}
} // namespace opennav::ais

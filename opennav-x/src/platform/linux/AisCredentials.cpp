#include "ais/Credentials.h"
#include <cstdlib>

namespace opennav::ais {
namespace {
class EnvironmentCredentials final : public IAisCredentials {
public:
  CredentialResult Read() const override {
    CredentialResult result;
    const char *value = std::getenv("AISSTREAM_API_KEY");
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
};
} // namespace
std::unique_ptr<IAisCredentials> CreateAisCredentials() {
  return std::make_unique<EnvironmentCredentials>();
}
} // namespace opennav::ais

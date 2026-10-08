#pragma once
#include <array>
#include <memory>
#include <string>
#include <string_view>

namespace opennav::ais {
// Deliberately not streamable/copyable. No plaintext key in settings or health.
class Secret {
public:
  Secret() = default;
  ~Secret();
  Secret(const Secret &) = delete;
  Secret &operator=(const Secret &) = delete;
  Secret(Secret &&other) noexcept;
  Secret &operator=(Secret &&other) noexcept;
  bool Assign(std::string_view bytes);
  void Clear() noexcept;
  std::string_view View() const { return {bytes_.data(), size_}; }
  bool Empty() const { return size_ == 0; }

private:
  std::array<char, 512> bytes_{};
  std::size_t size_ = 0;
};
enum class CredentialStatus {
  Ready,
  Missing,
  Invalid,
  Unavailable,
  ReadOnly,
  Removed
};
struct CredentialResult {
  CredentialStatus status = CredentialStatus::Missing;
  Secret key;
};
class IAisCredentials {
public:
  virtual ~IAisCredentials() = default;
  virtual CredentialResult Read() const = 0;
  virtual CredentialStatus Store(const Secret &) = 0;
  virtual CredentialStatus Remove() = 0;
};
// Windows: per-user, local-machine-persistent Credential Manager generic entry.
// Linux development: environment-only read; never writes a plaintext file.
std::unique_ptr<IAisCredentials> CreateAisCredentials();
// Separate GRIBstream weather token entry (SCRUM-324/325). Same storage
// rules: Windows Credential Manager target "OpenNavX/GRIBstream/v1";
// Linux development reads SKAGER_GRIBSTREAM_TOKEN only and never writes.
std::unique_ptr<IAisCredentials> CreateWeatherCredentials();
#ifdef OPENNAV_CREDENTIAL_TEST_SCOPE
// This factory exists only in the dedicated test library/executable. It cannot
// address the product entry or any unrelated Windows credential.
std::unique_ptr<IAisCredentials>
CreateTestAisCredentials(const std::string &unique_id);
#endif
} // namespace opennav::ais

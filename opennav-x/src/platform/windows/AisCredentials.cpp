#include "ais/Credentials.h"
#define WIN32_LEAN_AND_MEAN
#include <utility>
#include <windows.h>

#include <wincred.h>

namespace opennav::ais {
namespace {
class WindowsCredentials final : public IAisCredentials {
public:
  explicit WindowsCredentials(std::wstring target,
                              std::wstring user = L"SKAGER AISStream")
      : target_(std::move(target)), user_(std::move(user)) {}
  CredentialResult Read() const override {
    CredentialResult result;
    PCREDENTIALW credential = nullptr;
    if (!CredReadW(target_.c_str(), CRED_TYPE_GENERIC, 0, &credential)) {
      result.status = GetLastError() == ERROR_NOT_FOUND
                          ? CredentialStatus::Missing
                          : CredentialStatus::Unavailable;
      return result;
    }
    if (credential && credential->Type == CRED_TYPE_GENERIC &&
        credential->CredentialBlob && credential->CredentialBlobSize > 0 &&
        credential->CredentialBlobSize <= 512 &&
        result.key.Assign(
            {reinterpret_cast<const char *>(credential->CredentialBlob),
             credential->CredentialBlobSize}))
      result.status = CredentialStatus::Ready;
    else
      result.status = CredentialStatus::Invalid;
    if (credential) {
      if (credential->CredentialBlob && credential->CredentialBlobSize)
        SecureZeroMemory(credential->CredentialBlob,
                         credential->CredentialBlobSize);
      CredFree(credential);
    }
    return result;
  }
  CredentialStatus Store(const Secret &key) override {
    if (key.Empty())
      return CredentialStatus::Invalid;
    CREDENTIALW entry{};
    entry.Type = CRED_TYPE_GENERIC;
    entry.TargetName = const_cast<wchar_t *>(target_.c_str());
    entry.Persist =
        CRED_PERSIST_LOCAL_MACHINE; // this user, this PC; no roaming
    entry.UserName = const_cast<wchar_t *>(user_.c_str());
    entry.CredentialBlob =
        reinterpret_cast<LPBYTE>(const_cast<char *>(key.View().data()));
    entry.CredentialBlobSize = static_cast<DWORD>(key.View().size());
    if (!CredWriteW(&entry, 0))
      return CredentialStatus::Unavailable;
    // Verify actual stored bytes; a successful API call is not our only gate.
    auto verified = Read();
    return verified.status == CredentialStatus::Ready &&
                   verified.key.View() == key.View()
               ? CredentialStatus::Ready
               : CredentialStatus::Unavailable;
  }
  CredentialStatus Remove() override {
    if (!CredDeleteW(target_.c_str(), CRED_TYPE_GENERIC, 0) &&
        GetLastError() != ERROR_NOT_FOUND)
      return CredentialStatus::Unavailable;
    return Read().status == CredentialStatus::Missing
               ? CredentialStatus::Removed
               : CredentialStatus::Unavailable;
  }

private:
  std::wstring target_, user_;
};
} // namespace
std::unique_ptr<IAisCredentials> CreateAisCredentials() {
  return std::make_unique<WindowsCredentials>(L"OpenNavX/AISStream/v1");
}
std::unique_ptr<IAisCredentials> CreateWeatherCredentials() {
  return std::make_unique<WindowsCredentials>(L"OpenNavX/GRIBstream/v1",
                                              L"SKAGER GRIBstream");
}
#ifdef OPENNAV_CREDENTIAL_TEST_SCOPE
std::unique_ptr<IAisCredentials>
CreateTestAisCredentials(const std::string &id) {
  if (id.empty() || id.size() > 64)
    return {};
  for (char c : id)
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || c == '-'))
      return {};
  return std::make_unique<WindowsCredentials>(
      L"OpenNavX/Tests/AISStream/" + std::wstring(id.begin(), id.end()));
}
#endif
} // namespace opennav::ais

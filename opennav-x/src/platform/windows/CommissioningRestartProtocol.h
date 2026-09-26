#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace opennav::platform::commissioning {
inline constexpr char Protocol[] = "OpenNavX.CommissioningRestart.1";
inline constexpr std::size_t MaximumFrame = 65536;
inline constexpr std::uint64_t MaximumPermitTicks = 100000000; // FILETIME: 10 s.
enum class GuardState { Unarmed, Armed, Invalid };
struct Binding {
  GuardState state = GuardState::Unarmed;
  std::string session, record_sha256;
};
Binding ReadBinding(const std::optional<std::string>& session,
                    const std::optional<std::string>& record_sha256);
bool IsHash(const std::string& value);
bool IsMode(const std::string& value);
bool IsUtf8(const std::string& value);
struct Request {
  Binding binding;
  std::string nonce;
  std::uint64_t parent_pid = 0, parent_created = 0, helper_pid = 0,
                helper_created = 0, windows_session = 0;
  std::string executable, executable_sha256, helper, helper_sha256,
              working_directory, path;
  std::vector<std::string> arguments;
};
std::optional<std::string> EncodeRequest(const Request& request);
// Each string is UTF-8; the outer pipe framing is a separate uint32 LE length.
std::optional<std::string> EncodeFields(const std::vector<std::string>& fields);
std::optional<std::vector<std::string>> DecodeFields(const std::string& payload);
struct Permit {
  std::string session, record_sha256, nonce, request_sha256;
  std::uint64_t issued = 0, expires = 0;
  std::string executable, executable_sha256, helper, helper_sha256,
              profile, profile_sha256, working_directory, path, id;
};
std::optional<Permit> DecodePermit(const std::string& payload);
bool ValidatePermit(const Request& request, const Permit& permit,
                    const std::string& request_sha256, std::uint64_t now);
std::string EncodeReceipt(const Request& request, const Permit& permit,
                          const std::string& request_sha256,
                          std::uint64_t child_pid, std::uint64_t child_created,
                          std::uint32_t error);
} // namespace opennav::platform::commissioning

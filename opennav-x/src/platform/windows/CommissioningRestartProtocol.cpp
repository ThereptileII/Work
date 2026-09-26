#include "platform/windows/CommissioningRestartProtocol.h"

#include <algorithm>
#include <limits>

namespace opennav::platform::commissioning {
namespace {
bool Text(const std::string& value) {
  return !value.empty() && value.size() <= 32768 && IsUtf8(value) &&
         std::none_of(value.begin(), value.end(), [](unsigned char c) { return c < 32; });
}
bool Number(const std::string& value, std::uint64_t& result) {
  if (value.empty() || (value.size() > 1 && value[0] == '0')) return false;
  result = 0;
  for (const unsigned char c : value) {
    if (c < '0' || c > '9' || result > (UINT64_MAX - (c - '0')) / 10) return false;
    result = result * 10 + c - '0';
  }
  return true;
}
std::string Quoted(const std::string& value) {
  std::string out = "\"";
  for (const char c : value) {
    if (c == '\\' || c == '"') out += '\\';
    out += c;
  }
  return out + '"';
}
void U32(std::string& out, std::uint32_t n) {
  for (int i = 0; i != 4; ++i) out += static_cast<char>(n >> (i * 8));
}
bool U32(const std::string& bytes, std::size_t& at, std::uint32_t& value) {
  if (at > bytes.size() || bytes.size() - at < 4) return false;
  value = 0;
  for (int i = 0; i != 4; ++i)
    value |= static_cast<std::uint32_t>(static_cast<unsigned char>(bytes[at++])) << (i * 8);
  return true;
}
} // namespace
bool IsHash(const std::string& value) {
  return value.size() == 64 && std::all_of(value.begin(), value.end(), [](char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
  });
}
bool IsMode(const std::string& value) {
  return value == "--xnav" || value == "--legacy" || value == "--safe-mode";
}
bool IsUtf8(const std::string& value) {
  for (std::size_t i = 0; i < value.size();) {
    const auto first = static_cast<unsigned char>(value[i++]);
    if (first < 0x80) continue;
    int count;
    std::uint32_t cp, minimum;
    if (first >= 0xc2 && first <= 0xdf) { count = 1; cp = first & 31; minimum = 0x80; }
    else if (first >= 0xe0 && first <= 0xef) { count = 2; cp = first & 15; minimum = 0x800; }
    else if (first >= 0xf0 && first <= 0xf4) { count = 3; cp = first & 7; minimum = 0x10000; }
    else return false;
    while (count--) {
      if (i == value.size()) return false;
      const auto c = static_cast<unsigned char>(value[i++]);
      if ((c & 0xc0) != 0x80) return false;
      cp = (cp << 6) | (c & 63);
    }
    if (cp < minimum || cp > 0x10ffff || (cp >= 0xd800 && cp <= 0xdfff)) return false;
  }
  return true;
}
Binding ReadBinding(const std::optional<std::string>& session,
                    const std::optional<std::string>& hash) {
  if (!session && !hash) return {};
  if (!session || !hash || !IsHash(*session) || !IsHash(*hash))
    return {GuardState::Invalid, {}, {}};
  return {GuardState::Armed, *session, *hash};
}
std::optional<std::string> EncodeRequest(const Request& r) {
  if (r.binding.state != GuardState::Armed || !IsHash(r.binding.session) ||
      !IsHash(r.binding.record_sha256) || !IsHash(r.nonce) || !r.parent_pid ||
      !r.parent_created || !r.helper_pid || !r.helper_created ||
      r.arguments.size() != 1 || !IsMode(r.arguments[0]) ||
      !IsHash(r.executable_sha256) || !IsHash(r.helper_sha256) ||
      !Text(r.executable) || !Text(r.helper) || !Text(r.working_directory) || !Text(r.path)) return {};
  std::string out = "{\"protocol\":1,\"kind\":\"request\"";
  const auto add = [&](const char* name, const std::string& value) {
    out += "," + Quoted(name) + ":" + Quoted(value);
  };
  add("session", r.binding.session); add("recordSha256", r.binding.record_sha256); add("nonce", r.nonce);
  add("parentPid", std::to_string(r.parent_pid)); add("parentCreatedFiletime", std::to_string(r.parent_created));
  add("parentExitCode", "0"); add("helperPid", std::to_string(r.helper_pid));
  add("helperCreatedFiletime", std::to_string(r.helper_created)); add("windowsSessionId", std::to_string(r.windows_session));
  add("executable", r.executable); add("executableSha256", r.executable_sha256);
  add("helper", r.helper); add("helperSha256", r.helper_sha256);
  add("workingDirectory", r.working_directory); add("path", r.path);
  out += ",\"arguments\":[" + Quoted(r.arguments[0]) + "]}";
  if (out.size() > MaximumFrame) return {};
  return out;
}
std::optional<std::string> EncodeFields(const std::vector<std::string>& fields) {
  if (fields.empty() || fields.size() > 32) return {};
  std::string out; U32(out, static_cast<std::uint32_t>(fields.size()));
  for (const auto& field : fields) {
    if (!Text(field)) return {};
    U32(out, static_cast<std::uint32_t>(field.size())); out += field;
    if (out.size() > MaximumFrame) return {};
  }
  return out;
}
std::optional<std::vector<std::string>> DecodeFields(const std::string& payload) {
  if (payload.empty() || payload.size() > MaximumFrame) return {};
  std::size_t at = 0; std::uint32_t count = 0;
  if (!U32(payload, at, count) || count == 0 || count > 32) return {};
  std::vector<std::string> result;
  while (count--) {
    std::uint32_t bytes;
    if (!U32(payload, at, bytes) || bytes == 0 || bytes > 32768 || bytes > payload.size() - at) return {};
    auto value = payload.substr(at, bytes); at += bytes;
    if (!Text(value)) return {};
    result.push_back(std::move(value));
  }
  if (at != payload.size()) return {};
  return result;
}
std::optional<Permit> DecodePermit(const std::string& payload) {
  const auto f = DecodeFields(payload);
  if (!f || f->size() != 17 || (*f)[0] != Protocol || (*f)[1] != "ALLOW") return {};
  Permit p;
  p.session=(*f)[2];p.record_sha256=(*f)[3];p.nonce=(*f)[4];p.request_sha256=(*f)[5];
  if (!Number((*f)[6], p.issued) || !Number((*f)[7], p.expires)) return {};
  p.executable=(*f)[8];p.executable_sha256=(*f)[9];p.helper=(*f)[10];p.helper_sha256=(*f)[11];
  p.profile=(*f)[12];p.profile_sha256=(*f)[13];p.working_directory=(*f)[14];p.path=(*f)[15];p.id=(*f)[16];
  return p;
}
bool ValidatePermit(const Request& r, const Permit& p, const std::string& hash, std::uint64_t now) {
  return EncodeRequest(r).has_value() && IsHash(hash) && IsHash(p.id) &&
         p.session == r.binding.session && p.record_sha256 == r.binding.record_sha256 &&
         p.nonce == r.nonce && p.request_sha256 == hash && p.issued <= now &&
         p.expires > now && p.expires >= p.issued && p.expires - p.issued <= MaximumPermitTicks &&
         p.executable == r.executable && p.executable_sha256 == r.executable_sha256 &&
         p.helper == r.helper && p.helper_sha256 == r.helper_sha256 &&
         Text(p.profile) && IsHash(p.profile_sha256) &&
         p.working_directory == r.working_directory && Text(p.path);
}
std::string EncodeReceipt(const Request& r, const Permit& p, const std::string& hash,
                          std::uint64_t pid, std::uint64_t created, std::uint32_t error) {
  return "{\"protocol\":1,\"kind\":\"receipt\",\"session\":" + Quoted(r.binding.session) +
      ",\"recordSha256\":" + Quoted(r.binding.record_sha256) + ",\"nonce\":" + Quoted(r.nonce) +
      ",\"requestSha256\":" + Quoted(hash) + ",\"permitId\":" + Quoted(p.id) +
      ",\"status\":" + Quoted(pid ? "started" : "failed") + ",\"childPid\":" + Quoted(std::to_string(pid)) +
      ",\"childCreatedFiletime\":" + Quoted(std::to_string(created)) +
      ",\"win32Error\":" + Quoted(std::to_string(error)) + "}";
}
} // namespace opennav::platform::commissioning

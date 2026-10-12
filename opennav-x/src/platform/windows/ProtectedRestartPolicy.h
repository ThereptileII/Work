#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace opennav::platform {
// Routing only. Neither a parsed path nor a mode argument grants selection,
// image trust, profile access or permission to bypass broker authentication.
enum class ProtectedRestartKind { Ordinary, BrokerCurrent, Refuse };
struct ProtectedRestartRoute {
  ProtectedRestartKind kind = ProtectedRestartKind::Refuse;
  std::wstring generation;
  std::wstring argument;
};
inline std::wstring RestartPathFold(std::wstring text) {
  for (auto &ch : text) {
    if (ch == L'/')
      ch = L'\\';
    if (ch >= L'A' && ch <= L'Z')
      ch += L'a' - L'A';
  }
  return text;
}
inline bool RestartPathBelow(const std::wstring &path,
                             const std::wstring &root) {
  return path.size() > root.size() && path.compare(0, root.size(), root) == 0 &&
         path[root.size()] == L'\\';
}
inline bool LooksProtectedRestartNamespace(const std::wstring &image,
                                           const std::wstring &program_files) {
  const auto path = RestartPathFold(image);
  const auto root = RestartPathFold(program_files) + L"\\skager";
  return path == root || RestartPathBelow(path, root) ||
         path.find(L"\\skager\\applications") != std::wstring::npos;
}
inline bool RestartUserSID(const std::wstring &sid) {
  if (sid.compare(0, 9, L"S-1-5-21-") != 0)
    return false;
  std::size_t offset = 9;
  for (int i = 0; i < 4; ++i) {
    const auto end = sid.find(L'-', offset);
    const auto part =
        sid.substr(offset, end == std::wstring::npos ? end : end - offset);
    if (part.empty() || part.size() > 10 ||
        (part.size() > 1 && part[0] == L'0'))
      return false;
    std::uint64_t number = 0;
    for (const auto ch : part) {
      if (ch < L'0' || ch > L'9')
        return false;
      number = number * 10 + ch - L'0';
    }
    if (number > UINT32_MAX || (i == 3) != (end == std::wstring::npos))
      return false;
    offset = end + 1;
  }
  return true;
}
// These names are deny-only fences for the existing helper/qualification
// flows. They are never removed to manufacture ordinary launch permission.
inline bool ProtectedRestartEnvironmentEntryAllowed(const std::wstring &entry) {
  const auto equal =
      entry.find(L'=', entry.empty() || entry[0] != L'=' ? 0 : 1);
  const auto key = RestartPathFold(entry.substr(0, equal));
  return key != L"opennav_commissioning_restart_session" &&
         key != L"opennav_commissioning_restart_record_sha256" &&
         key.compare(0, 14, L"skager_update_") != 0;
}
inline ProtectedRestartRoute
ProtectedRestartPolicy(const std::wstring &actual_image,
                       const std::wstring &program_files,
                       const std::vector<std::string> &arguments) {
  if (!LooksProtectedRestartNamespace(actual_image, program_files))
    return {ProtectedRestartKind::Ordinary, {}, {}};
  if (program_files.empty() || actual_image.find(L'/') != std::wstring::npos ||
      actual_image.find(L'\0') != std::wstring::npos)
    return {};
  const auto folded = RestartPathFold(actual_image);
  const auto root = RestartPathFold(program_files);
  if (!RestartPathBelow(folded, root))
    return {};
  std::vector<std::wstring> parts;
  auto offset = program_files.size() + 1;
  while (offset <= actual_image.size()) {
    const auto end = actual_image.find(L'\\', offset);
    auto part = actual_image.substr(
        offset, end == std::wstring::npos ? end : end - offset);
    if (part.empty() || part == L"." || part == L".." || part.back() == L'.' ||
        part.back() == L' ' ||
        part.find_first_of(L":/<>\"|?*") != std::wstring::npos)
      return {};
    parts.push_back(part);
    if (end == std::wstring::npos)
      break;
    offset = end + 1;
  }
  if (parts.size() != 7 || RestartPathFold(parts[0]) != L"skager" ||
      RestartPathFold(parts[1]) != L"applications" ||
      !RestartUserSID(parts[2]) ||
      RestartPathFold(parts[3]) != L"generations" ||
      RestartPathFold(parts[5]) != L"app" ||
      RestartPathFold(parts[6]) != L"opencpn.exe" || parts[4].size() != 32 ||
      arguments.size() != 1)
    return {};
  for (const auto ch : parts[4])
    if (!((ch >= L'0' && ch <= L'9') || (ch >= L'a' && ch <= L'f')))
      return {};
  std::wstring suffix;
  if (arguments[0] == "--legacy")
    suffix = L":legacy";
  else if (arguments[0] == "--safe-mode")
    suffix = L":safe";
  else if (arguments[0] != "--xnav")
    return {};
  return {ProtectedRestartKind::BrokerCurrent, parts[4],
          L"--broker-current=" + parts[4] + suffix};
}
} // namespace opennav::platform

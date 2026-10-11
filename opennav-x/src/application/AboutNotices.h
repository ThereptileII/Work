#pragma once
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

namespace opennav::application {
enum class AboutNotice { Sources, Components, SkagerLicense, OpenCpnLicense };
inline const char *AboutNoticeFile(AboutNotice notice) {
  switch (notice) {
    case AboutNotice::Sources: return "SOURCE_AND_LICENSES.md";
    case AboutNotice::Components: return "THIRD_PARTY_NOTICES.md";
    case AboutNotice::SkagerLicense: return "licenses/SKAGER-COPYING.txt";
    case AboutNotice::OpenCpnLicense: return "licenses/OpenCPN/COPYING.gplv2";
  }
  return "";
}
// Both the portable package and versioned installer payload preserve app/ and
// docs/ siblings. Never search the working directory or another installation.
inline std::filesystem::path AboutDocsDirectory(const std::filesystem::path &executable) {
  if (!executable.is_absolute()) return {};
  return executable.parent_path().parent_path() / "docs";
}
struct AboutNoticeText { bool available = false; std::string text; };
inline AboutNoticeText ReadAboutNotice(const std::filesystem::path &executable,
                                      AboutNotice notice) {
  const auto docs = AboutDocsDirectory(executable);
  const std::string name = AboutNoticeFile(notice);
  const auto unavailable = [&] {
    return AboutNoticeText{false, "This packaged notice is missing or unreadable: docs/" + name +
      ". Restore or repair this SKAGER package to read its notices. No substitute notice has been loaded."};
  };
  if (docs.empty() || name.empty()) return unavailable();
  const auto file = docs / name;
  std::error_code error;
  if (!std::filesystem::is_regular_file(file, error) || error) return unavailable();
  const auto size = std::filesystem::file_size(file, error);
  // Text only, bounded before allocation. Never execute or interpret Markdown.
  if (error || size == 0 || size > 2 * 1024 * 1024) return unavailable();
  std::ifstream input(file, std::ios::binary);
  std::string text(static_cast<std::size_t>(size), '\0');
  if (!input.read(text.data(), static_cast<std::streamsize>(text.size())) ||
      text.find('\0') != std::string::npos) return unavailable();
  return {true, std::move(text)};
}
} // namespace opennav::application

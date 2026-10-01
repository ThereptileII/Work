#ifndef OPENNAV_DOWNLOADER_TEST_WX_FILENAME_H
#define OPENNAV_DOWNLOADER_TEST_WX_FILENAME_H

#include <cerrno>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

class wxString {
 public:
  wxString() = default;
  explicit wxString(std::string value) : value_(std::move(value)) {}
  std::string ToStdString() const { return value_; }

 private:
  std::string value_;
};

class wxFileName {
 public:
  explicit wxFileName(const std::string& path) : path_(path) {}

  static wxString CreateTempFileName(const std::string& prefix) {
    std::string pattern = prefix + "XXXXXX";
    std::vector<char> writable(pattern.begin(), pattern.end());
    writable.push_back('\0');
    const int descriptor = mkstemp(writable.data());
    if (descriptor < 0) throw std::runtime_error("mkstemp failed");
    close(descriptor);
    return wxString(writable.data());
  }

  wxString GetPathWithSep() const {
    std::filesystem::path parent = std::filesystem::path(path_).parent_path();
    if (parent.empty()) parent = ".";
    return wxString(parent.string() + "/");
  }

 private:
  std::string path_;
};

inline bool wxRenameFile(const std::string& source,
                         const std::string& destination, bool overwrite) {
  std::error_code error;
  if (overwrite) std::filesystem::remove(destination, error);
  error.clear();
  std::filesystem::rename(source, destination, error);
  return !error;
}

#endif

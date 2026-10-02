// Test-only context for byte-identical slices of pinned plugin_handler.cpp.
// No application profile, plugin loader, or marine code is initialized.
#include <algorithm>
#include <archive.h>
#include <archive_entry.h>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <curl/curl.h>
#include "model/downloader.h"

namespace fs = std::filesystem;
using pathmap_t = std::unordered_map<std::string, std::string>;
static std::string SEP("/");
static fs::path fixture_root;
static unsigned archive_entries = 0;
static unsigned archive_opens = 0;
#define MESSAGE_LOG std::cerr
#define DEBUG_LOG std::cerr

struct PluginMetadata {
  std::string name, version, tarball_url;
};

class PluginHandler {
 public:
  std::string last_error_msg;
  bool InstallPlugin(PluginMetadata plugin);
  bool InstallPlugin(PluginMetadata plugin, std::string path);
  bool ArchiveCheck(int r, const char* msg, struct archive* a);
  bool ExplodeTarball(struct archive* src, struct archive* dest,
                      std::string& filelist, const std::string& metadata_path,
                      bool only_metadata);
  bool ExtractTarball(const std::string path, std::string& filelist,
                      const std::string metadata_path = "",
                      bool only_metadata = false);
  static std::string FileListPath(std::string name);
  static std::string VersionPath(std::string name);
  // Archive-error rollback is deliberately outside this probe. If reached,
  // fail the probe rather than substitute a successful cleanup implementation.
  static void Cleanup(const std::string&, const std::string&) {
    throw std::runtime_error("unexpected archive-error cleanup boundary");
  }
};

// Fixture-only adapters for application paths and temporary allocation.
static std::string pluginsConfigDir() {
  return (fixture_root / "records").string();
}
static std::string tmpfile_path() {
  const auto path = fixture_root / "temporary" / "download.tar";
  if (fs::exists(path)) throw std::runtime_error("fixture temp path already exists");
  std::ofstream owned(path, std::ios::binary);
  if (!owned) throw std::runtime_error("cannot allocate fixture temp path");
  owned.close();
  return path.string();
}
static pathmap_t getInstallPaths() {
  return {{"share", (fixture_root / "installed").string()}};
}
static bool entry_set_install_path(struct archive_entry* entry, pathmap_t paths) {
  // Only the two inert, known fixture entries may ever be written.
  const std::string name = archive_entry_pathname(entry);
  if ((name != "fixture/share/existing.txt" &&
       name != "fixture/share/new.txt") ||
      archive_entry_filetype(entry) != AE_IFREG ||
      archive_entry_symlink(entry) || archive_entry_hardlink(entry)) {
    throw std::runtime_error("unexpected fixture archive entry");
  }
  const auto dest = fs::path(paths.at("share")) / fs::path(name).filename();
  archive_entry_set_pathname(entry, dest.string().c_str());
  ++archive_entries;
  return true;
}

// Observation only: delegate every archive allocation to real libarchive.
static struct archive* observed_archive_read_new() {
  ++archive_opens;
  return archive_read_new();
}
#define archive_read_new observed_archive_read_new
#include "plugin-handler-slices.inc"
#undef archive_read_new

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  fixture_root = fs::absolute(argv[2]);
  if (!fs::is_directory(fixture_root / "installed") ||
      !fs::is_directory(fixture_root / "records") ||
      !fs::is_directory(fixture_root / "temporary")) return 3;
  if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) return 4;
  try {
    PluginHandler handler;
    const bool ok = handler.InstallPlugin({"Fixture", "2", argv[1]});
    std::cout << "install_ok=" << (ok ? "true" : "false") << "\n"
              << "archive_opens=" << archive_opens << "\n"
              << "archive_entries=" << archive_entries << "\n"
              << "error=" << handler.last_error_msg << "\n";
    curl_global_cleanup();
    return ok ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "probe boundary failure: " << error.what() << "\n";
    curl_global_cleanup();
    return 5;
  }
}

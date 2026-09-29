#pragma once
// Test-fixture readiness files only. Never linked into the product or helper.
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <stdexcept>

namespace opennav::tests {
enum class MarkerStage { Written, Closed };
using MarkerObserver = std::function<void(MarkerStage,const std::filesystem::path &)>;

// A marker has exactly one producer. Its final name is a readiness signal:
// readers must never observe that name while a writer still owns its stream.
// Same-directory rename publishes the closed file atomically on the fixture's
// local filesystem. The Windows rename also refuses an existing destination.
inline bool PublishMarker(const std::filesystem::path &destination,
                          const std::string &contents,std::string &error,
                          const MarkerObserver &observe={}) {
  namespace fs=std::filesystem;
  static std::atomic<unsigned long long> serial{0};
  struct Cleanup {
    fs::path directory;
    ~Cleanup(){if(!directory.empty()){std::error_code ignored;fs::remove_all(directory,ignored);}}
  } cleanup;
  error.clear();
  try {
    const auto final=fs::absolute(destination);
    if(fs::exists(fs::symlink_status(final)))throw std::runtime_error("marker already exists");
    const auto token=std::chrono::steady_clock::now().time_since_epoch().count();
    // Atomic directory reservation avoids sharing an open temporary path with
    // another fixture process, without ever using the final readiness name.
    for(unsigned attempt=0;attempt<32;++attempt) {
      const auto candidate=final.parent_path()/
          (".opennav-marker-"+std::to_string(token)+"-"+std::to_string(serial.fetch_add(1)));
      if(fs::create_directory(candidate)){cleanup.directory=candidate;break;}
    }
    if(cleanup.directory.empty())throw std::runtime_error("marker staging reservation failed");
    const auto staged=cleanup.directory/"closed-marker.tmp";
    {
      std::ofstream stream;
      stream.exceptions(std::ios::failbit|std::ios::badbit);
      stream.open(staged,std::ios::binary|std::ios::out);
      stream.write(contents.data(),static_cast<std::streamsize>(contents.size()));
      if(observe)observe(MarkerStage::Written,staged);
      stream.flush();
      stream.close(); // checked before any final name is made visible
    }
    if(observe)observe(MarkerStage::Closed,staged);
    if(fs::exists(fs::symlink_status(final)))throw std::runtime_error("marker appeared before publication");
    fs::rename(staged,final);
    return true;
  } catch(const std::exception &failure) {
    error=failure.what();return false;
  }
}
} // namespace opennav::tests

#include "platform/PlatformIntegration.h"

#include <cerrno>
#include <unistd.h>

namespace opennav::platform {

bool RestartAfterExit(const std::string& executable,
                      const std::vector<std::string>& arguments) {
  std::vector<char*> argv;
  argv.push_back(const_cast<char*>(executable.c_str()));
  for (const auto& value : arguments) argv.push_back(const_cast<char*>(value.c_str()));
  argv.push_back(nullptr);
  int channel[2];
  if (pipe(channel) != 0) return false;
  const pid_t child = fork();
  if (child < 0) { close(channel[0]); close(channel[1]); return false; }
  if (child == 0) {
    close(channel[1]);
    char ignored;
    ssize_t result;
    do { result = read(channel[0], &ignored, 1); } while (result < 0 && errno == EINTR);
    close(channel[0]);
    if (result != 0) _exit(125);
    // Only async-signal-safe calls after fork. EOF means parent process exited.
    setsid();
    execv(executable.c_str(), argv.data());
    _exit(127);
  }
  close(channel[0]);
  // Intentionally keep the parent's write end open until process termination.
  // Do not close it in a static destructor before wx/OpenCPN shutdown completes.
  return true;
}

}  // namespace opennav::platform

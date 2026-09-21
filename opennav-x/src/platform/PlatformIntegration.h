#pragma once

#include <string>
#include <vector>

namespace opennav::platform {

// Launch through the companion process, which waits for this process to exit.
// Arguments remain separate; no shell is used. Called only after clean shutdown.
bool RestartAfterExit(const std::string& executable,
                      const std::vector<std::string>& arguments);

}  // namespace opennav::platform

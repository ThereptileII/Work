#pragma once

#include "platform/windows/CommissioningRestartProtocol.h"
#include <string>
#include <vector>

namespace opennav::platform::commissioning {
// Read-only capability from the linked Windows guard implementation.
int ProtocolCapability();
// Captured by static initialization, before plugins can modify the environment.
const Binding& StartupBinding();
bool SpawnGuardedHelper(const std::wstring& helper, const std::wstring& executable,
                        const std::vector<std::string>& arguments);
int RunGuardedHelper(int argc, wchar_t** argv);
} // namespace opennav::platform::commissioning

#pragma once
// ixwebsocket HTTPS transport for the SKAGER Cloud worker. Linked only into the
// OpenCPN integration (opennav_cloud_runtime); unit tests use fakes.
#include "cloud/CloudService.h"
#include <memory>

namespace opennav::cloud {
// System CA store, hostname validation on, no redirects (the device token is
// never forwarded), 10 s connect / 20 s transfer timeouts, 16 KiB response cap.
std::unique_ptr<ICloudTransport> CreateIxCloudTransport();
}  // namespace opennav::cloud

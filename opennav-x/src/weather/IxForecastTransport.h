#pragma once
// ixwebsocket HTTPS transport for the forecast worker. Linked only into the
// OpenCPN integration (opennav_weather_runtime); unit tests use fakes.
#include "weather/ForecastService.h"
#include <memory>

namespace opennav::weather {
// System CA store, hostname validation on, no redirects (the bearer token is
// never forwarded), 10 s connect / 30 s transfer timeouts, body bounded by
// gribstream::kMaxResponseBytes, no gzip (no decompression bomb).
std::unique_ptr<IForecastTransport> CreateIxForecastTransport();
} // namespace opennav::weather

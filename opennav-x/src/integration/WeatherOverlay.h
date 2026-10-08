#pragma once
// Seam between the weather runtime (OpenCPN integration) and the chart
// overlay. Application thread only. Holds no OpenCPN pointers beyond a paint.
#include "weather/Weather.h"
#include <functional>
#include <optional>
#include <string>
class ocpnDC;
class ViewPort;
class ChartCanvas;

namespace opennav::integration {
// Integration registers the owned snapshot source once at Attach; empty
// clears it (mode restart, shutdown).
void SetWeatherSource(std::function<weather::ForecastSnapshot()> source);
// Forecast time step chosen in the UI; nullopt = nearest to now.
void SetWeatherDisplayTime(std::optional<weather::WallTime> valid_time);
std::optional<weather::WallTime> WeatherDisplayTime();
// User toggle (Chart layers → Wind). Off by default.
void SetWeatherOverlayVisible(bool visible);
bool WeatherOverlayVisible();
// Called from the chart canvas overlay paint (software and GL). Draws
// forecast wind arrows with scale-dependent density; never fetches data.
void DrawWeatherOverlay(ocpnDC &dc, ViewPort &vp, ChartCanvas &canvas);
// Credential-free one-line state for the Chart layers drawer's Wind row.
std::string WeatherLayerReason();
} // namespace opennav::integration

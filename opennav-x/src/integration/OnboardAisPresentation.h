#pragma once
#include "integration/OnboardAisBody.h"
class ocpnDC;
class ChartCanvas;
namespace opennav::integration {
bool DrawChartOnboardAis(ocpnDC &dc, ChartCanvas &canvas,
                        const OnboardAisAppearance &appearance,
                        double x, double y, double north_angle,
                        double user_scale, int attenuation);
} // namespace opennav::integration

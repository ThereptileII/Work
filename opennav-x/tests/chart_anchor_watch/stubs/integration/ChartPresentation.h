#pragma once
class ChartCanvas;
namespace opennav::integration {
bool XNavChartPresentationActive();
double ChartDisplayScale();
int ChartDisplayPixels(ChartCanvas&, int logical_pixels);
}

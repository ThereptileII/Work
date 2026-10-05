#pragma once
#include <string>
#include <vector>

namespace opennav::application {
struct ChartInfoObject {
  std::string title, kind;
  std::vector<std::string> summary;
  // Complete decoded text for this upstream section, including unknown fields
  // and attachment references. This is text, never executable HTML or links.
  std::string details;
};
struct ChartInfo {
  std::vector<ChartInfoObject> objects;
  double latitude = 0, longitude = 0;
  bool position_valid = false, truncated = false;
  std::string notice;
};
// Presentation only: upstream still selects all overlapping native/plugin
// chart objects, lights, overlays and AIS notices and formats their values.
// A bounded conversion preserves unrecognized content in expandable details.
ChartInfo ParseChartInfo(const std::string &upstream_html, double latitude,
                        double longitude);
} // namespace opennav::application

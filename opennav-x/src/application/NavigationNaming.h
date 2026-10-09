#pragma once
#include "application/NavigationObjects.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <tuple>
#include <sstream>

namespace opennav::application {
inline bool ValidNavigationName(const std::string &name) {
  if (name.empty() || name.size() > 128) return false;
  bool visible = false;
  for (std::size_t i = 0; i < name.size();) {
    const auto c = static_cast<unsigned char>(name[i]);
    if (c < 32 || c == 127) return false;
    if (c < 128) { visible |= c != ' '; ++i; continue; }
    const int count = c >= 0xc2 && c <= 0xdf ? 2 :
                      c >= 0xe0 && c <= 0xef ? 3 :
                      c >= 0xf0 && c <= 0xf4 ? 4 : 0;
    if (!count || i + count > name.size()) return false;
    unsigned codepoint = c & (0x7f >> count);
    for (int n = 1; n < count; ++n) {
      const auto next = static_cast<unsigned char>(name[i + n]);
      if ((next & 0xc0) != 0x80) return false;
      codepoint = (codepoint << 6) | (next & 0x3f);
    }
    if ((count == 3 && codepoint < 0x800) ||
        (count == 4 && codepoint < 0x10000) || codepoint > 0x10ffff ||
        (codepoint >= 0xd800 && codepoint <= 0xdfff)) return false;
    visible = true;
    i += count;
  }
  return visible;
}
struct ChartNameCandidate {
  std::string name;
  double distance_nm = 0;
  std::string feature;  // S-57 acronym; empty when the producer did not supply it.
};
// Search radii in nautical miles, tried in order until a name is found
// (SCRUM-350). A single fixed cap left most waypoints unnamed.
inline const std::vector<double> &ChartNameSearchSteps() {
  static const std::vector<double> steps{1., 2., 4.};
  return steps;
}
// Lower ranks win. A harbour or a shore mark names a place better than a buoy
// that merely happens to float nearer to it, so type outranks distance.
inline int ChartNameFeatureRank(const std::string &feature) {
  static const std::vector<std::string> order{
      "HRBFAC", "ACHBRT", "BERTHS", "PILPNT", "LNDMRK", "LIGHTS",
      "BOYLAT", "BOYCAR", "BOYISD", "BOYSAW", "BOYSPP",
      "BCNLAT", "BCNCAR", "BCNISD", "BCNSAW", "BCNSPP"};
  const auto it = std::find(order.begin(), order.end(), feature);
  return it == order.end() ? static_cast<int>(order.size())
                           : static_cast<int>(it - order.begin());
}
inline bool RelevantChartNameFeature(const std::string &feature) {
  // Nearby identifiable places and navigation marks; not depths, hazards,
  // broad sea/land areas, or descriptive text masquerading as an object name.
  static const std::vector<std::string> types{
      "HRBFAC", "ACHBRT", "BERTHS", "PILPNT", "LNDMRK", "LIGHTS",
      "BOYLAT", "BOYCAR", "BOYISD", "BOYSAW", "BOYSPP",
      "BCNLAT", "BCNCAR", "BCNISD", "BCNSAW", "BCNSPP"};
  return std::find(types.begin(), types.end(), feature) != types.end();
}
inline NavigationNameSuggestion SuggestNavigationName(
    Coordinate position, bool route, const std::vector<ChartNameCandidate> &candidates,
    double limit_nm = 0) {
  // Keep the nearest occurrence of each name, and the best rank seen for it.
  struct Best { double distance_nm; int rank; };
  std::map<std::string, Best> by_name;
  for (const auto &candidate : candidates) {
    if (!ValidNavigationName(candidate.name) || !std::isfinite(candidate.distance_nm) ||
        candidate.distance_nm < 0) continue;
    if (limit_nm > 0 && candidate.distance_nm > limit_nm) continue;
    const Best entry{candidate.distance_nm, ChartNameFeatureRank(candidate.feature)};
    auto [it, added] = by_name.emplace(candidate.name, entry);
    if (!added) {
      it->second.distance_nm = std::min(it->second.distance_nm, entry.distance_nm);
      it->second.rank = std::min(it->second.rank, entry.rank);
    }
  }
  // Rank first, then distance, then name so the result never depends on the
  // order the chart happened to return objects in. A near-tie is resolved by
  // this ordering instead of abandoning the name, which was the old behaviour
  // and left coordinates wherever two marks sat close together.
  std::vector<std::tuple<int, double, std::string>> ordered;
  ordered.reserve(by_name.size());
  for (const auto &entry : by_name)
    ordered.push_back({entry.second.rank, entry.second.distance_nm, entry.first});
  std::sort(ordered.begin(), ordered.end());
  if (!ordered.empty()) {
    const auto name = (route ? "To " : "") + std::get<2>(ordered.front());
    if (ValidNavigationName(name)) return {name, true};
  }
  std::ostringstream fallback; fallback.imbue(std::locale::classic());
  fallback << (route ? "Route" : "Waypoint");
  if (std::isfinite(position.latitude_deg) && std::abs(position.latitude_deg) <= 90 &&
      std::isfinite(position.longitude_deg) && std::abs(position.longitude_deg) <= 180) {
    fallback << ' ' << std::fixed << std::setprecision(5)
             << std::abs(position.latitude_deg) << (position.latitude_deg < 0 ? 'S' : 'N')
             << ' ' << std::abs(position.longitude_deg) << (position.longitude_deg < 0 ? 'W' : 'E');
  }
  return {fallback.str(), false};
}
// A route is named by where it goes, not by what is nearest one end of it.
inline NavigationNameSuggestion SuggestRouteName(const std::string &first,
                                                 const std::string &last) {
  if (ValidNavigationName(first) && ValidNavigationName(last)) {
    const auto name = "From " + first + " to " + last;
    if (ValidNavigationName(name)) return {name, true};
  }
  if (ValidNavigationName(last)) {
    const auto name = "To " + last;
    if (ValidNavigationName(name)) return {name, true};
  }
  return {"Route", false};
}
class NavigationNameDraft {
 public:
  explicit NavigationNameDraft(std::string name) : original_(name), value_(std::move(name)) {}
  const std::string &Value() const { return value_; }
  void Set(std::string value) { value_ = std::move(value); }
  bool Changed() const { return value_ != original_; }
  void Cancel() { value_ = original_; }
  CommandResult Save(const std::function<CommandResult(const std::string &)> &save) {
    if (!ValidNavigationName(value_)) return {false, "Enter a non-empty, shorter name", {}};
    if (!save) return {false, "Name is read-only", {}};
    const auto result = save(value_);
    if (result.ok) original_ = value_;
    return result;
  }
 private:
  std::string original_, value_;
};
} // namespace opennav::application

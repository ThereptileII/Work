#pragma once
#include "application/NavigationObjects.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
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
};
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
    Coordinate position, bool route, const std::vector<ChartNameCandidate> &candidates) {
  std::map<std::string, double> nearest_by_name;
  for (const auto &candidate : candidates) {
    if (!ValidNavigationName(candidate.name) || !std::isfinite(candidate.distance_nm) ||
        candidate.distance_nm < 0 || candidate.distance_nm > .5) continue;
    auto [it, added] = nearest_by_name.emplace(candidate.name, candidate.distance_nm);
    if (!added) it->second = std::min(it->second, candidate.distance_nm);
  }
  std::vector<std::pair<double, std::string>> ordered;
  for (const auto &candidate : nearest_by_name) ordered.push_back({candidate.second, candidate.first});
  std::sort(ordered.begin(), ordered.end());
  // Near-ties within about 46 m are not an unambiguous nearest feature.
  if (!ordered.empty() && (ordered.size() == 1 || ordered[1].first - ordered[0].first > .025)) {
    const auto name = (route ? "To " : "") + ordered.front().second;
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

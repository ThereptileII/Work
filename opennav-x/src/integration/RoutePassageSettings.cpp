#include "integration/RoutePassageSettings.h"
#include "application/NavigationObjects.h"
#include "model/route.h"
#include "model/route_point.h"
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <wx/fileconf.h>
#include <wx/thread.h>

namespace opennav::integration {
namespace {
wxFileConfig* profile = nullptr;
bool storage_uncertain = false;
constexpr const char* visible_key = "/OpenNav/RoutePassageVisibleV1";
void MainThread() {
  if (!wxIsMainThread()) throw std::logic_error("Route passage settings require the application thread");
}
application::RoutePassageSettings Invalid(const std::string& reason) {
  application::RoutePassageSettings value;
  value.valid = false;
  value.reason = reason;
  return value;
}
std::optional<wxString> Key(const std::string& id) {
  if (id.empty() || id.size() > 128) return {};
  // Encode every byte, so imported identifiers cannot introduce config paths
  // or collide through normalization/case folding. No route geometry is stored.
  constexpr char hex[] = "0123456789abcdef";
  std::string encoded;
  encoded.reserve(id.size() * 2);
  for (unsigned char c : id) {
    encoded += hex[c >> 4]; encoded += hex[c & 15];
  }
  return wxString("/OpenNav/RoutePassageV1/") + wxString::FromUTF8(encoded);
}
bool Number(const std::string& text, double& value) {
  if (text.empty() || text.size() > 32 ||
      text.find_first_not_of("0123456789.eE+-") != std::string::npos) return false;
  std::istringstream stream(text);
  stream.imbue(std::locale::classic());
  stream >> std::noskipws >> value;
  return !stream.fail() && stream.eof() && std::isfinite(value);
}
std::string Encode(const application::RoutePassageSettings& value) {
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << "v1|" << std::setprecision(17);
  if (value.arrival_radius_m) stream << *value.arrival_radius_m;
  else stream << "stock";
  stream << '|' << value.corridor_width_m;
  return stream.str();
}
application::RoutePassageSettings Decode(const wxString& stored) {
  const auto text = stored.ToStdString(wxConvUTF8);
  const auto split = text.find('|', 3);
  if (text.size() > 96 || text.compare(0, 3, "v1|") != 0 ||
      split == std::string::npos || text.find('|', split + 1) != std::string::npos)
    return Invalid("Stored route passage settings are invalid; automatic passage is withheld");
  application::RoutePassageSettings value;
  const auto radius = text.substr(3, split - 3);
  double parsed = 0;
  if (radius != "stock") {
    if (!Number(radius, parsed))
      return Invalid("Stored waypoint passage distance is invalid; automatic passage is withheld");
    value.arrival_radius_m = parsed;
  }
  if (!Number(text.substr(split + 1), value.corridor_width_m))
    return Invalid("Stored corridor width is invalid; automatic passage is withheld");
  const auto reason = application::ValidateRoutePassage(value);
  return reason.empty() ? value : Invalid(reason);
}
application::CommandResult Store(const wxString& key, const wxString& text,
                                  const std::string& identity) {
  if (!profile || storage_uncertain)
    return {false, "Route passage storage unavailable; settings were not saved", identity};
  const bool existed = profile->HasEntry(key);
  wxString previous;
  if (existed && !profile->Read(key, &previous))
    return {false, "Previous route passage settings could not be read", identity};
  if (!profile->Write(key, text) || !profile->Flush()) {
    bool restored = existed ? profile->Write(key, previous)
        : (!profile->HasEntry(key) || profile->DeleteEntry(key));
    restored = profile->Flush() && restored;
    if (!restored) storage_uncertain = true;
    return {false, restored ? "Save failed; previous route passage settings restored"
        : "Save and rollback failed; automatic passage withheld until profile storage is recovered",
        identity};
  }
  return {true, "Route passage settings saved in this profile; GPX exports do not include them", identity};
}
} // namespace

void ConfigureRoutePassage(wxFileConfig* config) {
  MainThread();
  if (config != profile) storage_uncertain = false;
  profile = config;
}
application::RoutePassageSettings ReadRoutePassage(const std::string& route_id) {
  MainThread();
  const auto key = Key(route_id);
  if (!key) return Invalid("Route identity is unavailable or too long for passage settings");
  if (!profile || storage_uncertain)
    return Invalid("Route passage storage unavailable; automatic passage is withheld");
  if (!profile->HasEntry(*key)) {
    application::RoutePassageSettings value;
    value.storage_revision = "absent";
    return value;
  }
  wxString stored;
  if (!profile->Read(*key, &stored))
    return Invalid("Stored route passage settings could not be read; automatic passage is withheld");
  // Bind malformed records too, so repairing one cannot overwrite a different
  // record saved after selection. Oversized/unreadable values cannot provide a
  // bounded exact identity and therefore cannot be repaired through this API.
  if (stored.size() > 96)
    return Invalid("Stored route passage settings are too long; profile repair is required and automatic passage is withheld");
  const auto text = stored.ToStdString(wxConvUTF8);
  if (text.size() > 96)
    return Invalid("Stored route passage settings are too long; profile repair is required and automatic passage is withheld");
  auto value = Decode(stored);
  value.storage_revision = "record:" + text;
  return value;
}
bool PassageVisible() {
  MainThread();
  if (!profile || storage_uncertain) return false;
  if (!profile->HasEntry(visible_key)) return true;
  wxString value;
  return profile->Read(visible_key, &value) && value == "1";
}
application::CommandResult SetPassageVisible(bool visible) {
  MainThread();
  auto result = Store(visible_key, visible ? "1" : "0", {});
  if (result.ok) result.message = visible ? "Active route passage overlay shown" : "Route passage overlay hidden";
  return result;
}
application::CommandResult detail::StoreRoutePassage(
    const std::string& route_id, const application::RoutePassageSettings& value) {
  MainThread();
  const auto reason = application::ValidateRoutePassage(value);
  const auto key = Key(route_id);
  if (!reason.empty()) return {false, reason, route_id};
  if (!key) return {false, "Route identity is unavailable or too long", route_id};
  return Store(*key, wxString::FromUTF8(Encode(value)), route_id);
}
std::optional<double> ResolveRouteArrivalRadiusNm(
    const ::Route& route, const ::RoutePoint& point, double upstream_radius) {
  MainThread();
  // Layer behavior stays native. With no explicit radius, preserve the exact
  // stock policy in XNav, Legacy and Safe; invalid saved data never falls back.
  if (route.m_bIsInLayer || point.m_bIsInLayer) return std::nullopt;
  const auto value = ReadRoutePassage(route.GetGUID().ToStdString(wxConvUTF8));
  if (!application::ValidateRoutePassage(value).empty()) return -1.;
  if (!value.arrival_radius_m) return std::nullopt;
  // An opt-in radius cannot defeat emergency never-arrive sentinels.
  if (!std::isfinite(upstream_radius) || upstream_radius < 0 ||
      point.m_WaypointArrivalRadius < 0) return -1.;
  if (!route.pRoutePointList) return -1.;
  for (auto* node = route.pRoutePointList->GetFirst(); node; node = node->GetNext()) {
    auto* waypoint = node->GetData();
    if (!waypoint || waypoint->GetIconName() == "mob") return -1.;
  }
  return *value.arrival_radius_m / 1852.;
}
} // namespace opennav::integration

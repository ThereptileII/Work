#include "integration/SettingsStore.h"
#include <cmath>
#include <wx/log.h>
#include <wx/thread.h>
namespace opennav::integration {
namespace {
constexpr const char *key = "/OpenNav/AlphaSettings";
constexpr const char *name_key = "/OpenNav/VesselName";
constexpr const char *chart_key = "/Settings/GlobalState/S52_MAR_SAFETY_CONTOUR";
constexpr const char *display_key = "/OpenNav/DisplayPreferencesV1";
bool ValidName(const std::string &name) {
  if (name.empty()) return true;  // Unconfigured is a real profile state.
  if (name.size() > 128 ||
      name.find_first_not_of(" \t\r\n") == std::string::npos) return false;
  for (unsigned char c : name) if (c < 32 || c == 127) return false;
  return !wxString::FromUTF8(name).empty();
}
}
SettingsStore::SettingsStore(wxFileConfig &config) : config_(config) {
  if (!wxIsMainThread())
    throw std::logic_error("Settings require the application thread");
  wxString stored_display;
  if (config_.Read(display_key, &stored_display)) {
    try {
      display_ = application::DecodeDisplayPreferences(
          stored_display.ToStdString(wxConvUTF8));
    } catch (const std::exception &) {
      display_status_ = "Stored display preferences rejected; using 100% and Balanced";
    }
  }
  wxString stored_name;
  if (config_.Read(name_key, &stored_name)) {
    const auto candidate = stored_name.ToStdString(wxConvUTF8);
    if (ValidName(candidate)) vessel_name_ = candidate;
    else status_ = "Stored vessel display name is invalid; name unavailable";
  }
  wxString text;
  if (!config_.Read(key, &text)) {
    if (status_.empty()) status_ = "Live vessel settings unconfigured";
    return;
  }
  try {
    settings_ = application::DecodeSettings(text.ToStdString(wxConvUTF8));
    if (status_.empty()) status_ = "Loaded OpenCPN profile settings";
  } catch (const std::exception &e) {
    status_ =
        std::string("Settings rejected; live model disabled: ") + e.what();
    wxLogWarning("SKAGER %s", wxString::FromUTF8(status_));
  }
}
application::CommandResult SettingsStore::SaveDisplay(
    const application::DisplayPreferences &next) {
  if (!wxIsMainThread()) return {false, "Settings require the application thread"};
  try {
    const auto encoded = application::EncodeDisplayPreferences(next);
    wxString previous;
    const bool existed = config_.Read(display_key, &previous);
    if (!config_.Write(display_key, wxString::FromUTF8(encoded)) || !config_.Flush()) {
      const bool restored = (existed ? config_.Write(display_key, previous)
          : (!config_.HasEntry(display_key) || config_.DeleteEntry(display_key))) &&
          config_.Flush();
      return {false, restored ? "Display preferences could not be saved; previous values restored"
                              : "Display save/restore failed; inspect storage before restarting"};
    }
    display_ = next;
    display_status_ = "Display preferences saved in OpenCPN profile";
    return {true, display_status_};
  } catch (const std::exception &e) { return {false, e.what()}; }
}
application::CommandResult SettingsStore::SaveVessel(
    const application::Settings &next, const std::string &name,
    double chart_safety_depth_m) {
  if (!wxIsMainThread()) return {false, "Settings require the application thread"};
  try {
    application::ValidateSettings(next);
    if (!ValidName(name) ||
        !(std::isnan(chart_safety_depth_m) ||
          (std::isfinite(chart_safety_depth_m) &&
           chart_safety_depth_m >= 0 && chart_safety_depth_m <= 1000000)))
      return {false, "Invalid vessel name or chart safety depth"};
    const wxString encoded = wxString::FromUTF8(application::EncodeSettings(next));
    const bool update_chart = std::isfinite(chart_safety_depth_m);
    const wxString chart = update_chart
        ? wxString::FromUTF8(application::SettingNumber(chart_safety_depth_m))
        : wxString{};
    wxString previous[3];
    const char *keys[]{key, name_key, chart_key};
    bool existed[3]{};
    for (int i = 0; i < 3; ++i) existed[i] = config_.Read(keys[i], &previous[i]);
    const bool written = config_.Write(key, encoded) &&
                         config_.Write(name_key, wxString::FromUTF8(name)) &&
                         (!update_chart || config_.Write(chart_key, chart));
    if (!written || !config_.Flush()) {
      bool restored = true;
      for (int i = 0; i < 3; ++i)
        restored = (existed[i] ? config_.Write(keys[i], previous[i])
                               : (!config_.HasEntry(keys[i]) || config_.DeleteEntry(keys[i]))) && restored;
      restored = config_.Flush() && restored;
      wxLogError("SKAGER vessel profile save failed; disk restore: %s",
                 restored ? "confirmed" : "failed");
      return {false, restored ? "Vessel profile could not be saved; previous values restored"
                              : "Vessel profile save/restore failed; inspect storage before restarting"};
    }
    settings_ = next;
    vessel_name_ = name;
    status_ = update_chart ? "Saved vessel profile and OpenCPN chart safety depth"
                           : "Saved vessel profile; OpenCPN chart safety depth unchanged";
    return {true, status_};
  } catch (const std::exception &e) {
    return {false, e.what()};
  }
}
application::CommandResult
SettingsStore::Save(const application::Settings &settings) {
  if (!wxIsMainThread())
    return {false, "Settings require the application thread"};
  try {
    const auto text = application::EncodeSettings(settings);
    wxString old;
    const bool existed = config_.Read(key, &old);
    if (!config_.Write(key, wxString::FromUTF8(text)) || !config_.Flush()) {
      if (existed)
        config_.Write(key, old);
      else
        config_.DeleteEntry(key);
      const bool restored = config_.Flush();
      wxLogError("SKAGER settings save failed; previous in-memory settings "
                 "retained; disk restore: %s",
                 restored ? "confirmed" : "failed");
      return {false,
              restored
                  ? "Settings could not be saved; previous settings restored"
                  : "Settings save/restore failed; inspect storage before "
                    "restarting"};
    }
    settings_ = settings;
    status_ = "Saved explicit vessel configuration in OpenCPN profile";
    wxLogMessage("SKAGER settings saved (no sensor observations changed)");
    return {true, status_};
  } catch (const std::exception &e) {
    return {false, e.what()};
  }
}
} // namespace opennav::integration

#include "integration/SettingsStore.h"
#include <wx/log.h>
#include <wx/thread.h>
namespace opennav::integration {
namespace {
constexpr const char *key = "/OpenNav/AlphaSettings";
}
SettingsStore::SettingsStore(wxFileConfig &config) : config_(config) {
  if (!wxIsMainThread())
    throw std::logic_error("Settings require the application thread");
  wxString text;
  if (!config_.Read(key, &text)) {
    status_ = "Live vessel settings unconfigured";
    return;
  }
  try {
    settings_ = application::DecodeSettings(text.ToStdString(wxConvUTF8));
    status_ = "Loaded OpenCPN profile settings";
  } catch (const std::exception &e) {
    status_ =
        std::string("Settings rejected; live model disabled: ") + e.what();
    wxLogWarning("OpenNav %s", wxString::FromUTF8(status_));
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
      wxLogError("OpenNav settings save failed; previous in-memory settings "
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
    wxLogMessage("OpenNav settings saved (no sensor observations changed)");
    return {true, status_};
  } catch (const std::exception &e) {
    return {false, e.what()};
  }
}
} // namespace opennav::integration

#include "integration/TrackPresentation.h"

#include <wx/fileconf.h>
#include <wx/log.h>
#include <wx/thread.h>

namespace opennav::integration {
namespace {
wxFileConfig *profile = nullptr;
bool xnav_mode = false;
bool visible = true;
bool storage_uncertain = false;
constexpr const char *visible_key = "/OpenNav/TrackLayerVisibleV1";
} // namespace

void ConfigureTrackPresentation(wxFileConfig *config, bool xnav) {
  if (!wxIsMainThread()) return;
  profile = config;
  xnav_mode = xnav;
  visible = true;
  storage_uncertain = false;
  if (!profile || !xnav_mode || !profile->HasEntry(visible_key)) return;
  wxString stored;
  if (!profile->Read(visible_key, &stored) || (stored != "0" && stored != "1")) {
    // Unknown input must not silently hide existing navigation history.
    wxLogWarning("SKAGER track visibility setting is invalid; tracks remain shown");
    return;
  }
  visible = stored == "1";
}

bool XNavTrackLayerHidden() {
  return wxIsMainThread() && xnav_mode && !visible;
}

bool TrackLayerVisible() { return !XNavTrackLayerHidden(); }

application::CommandResult SetTrackLayerVisible(bool show) {
  if (!wxIsMainThread() || !xnav_mode || !profile || storage_uncertain)
    return {false, "Track display settings are unavailable in this interface"};
  const bool existed = profile->HasEntry(visible_key);
  wxString previous;
  if (existed && !profile->Read(visible_key, &previous))
    return {false, "Previous track display setting could not be read"};
  if (!profile->Write(visible_key, wxString(show ? "1" : "0")) || !profile->Flush()) {
    bool restored = existed ? profile->Write(visible_key, previous)
        : (!profile->HasEntry(visible_key) || profile->DeleteEntry(visible_key));
    restored = profile->Flush() && restored;
    storage_uncertain = !restored;
    return {false, restored
        ? "Track display setting was not saved; previous setting restored"
        : "Track display save and rollback failed; check profile storage before retrying"};
  }
  // Publish only after persistence succeeds. The navigation bridge invalidates
  // all canvas/GL caches; this helper never touches tracks or recording state.
  visible = show;
  return {true, show ? "Tracks shown according to their individual visibility"
                     : "Tracks hidden on the SKAGER chart; recording is unchanged"};
}
} // namespace opennav::integration

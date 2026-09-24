#include "integration/RecoveryStore.h"
#include <filesystem>
#include <wx/file.h>
#include <wx/filename.h>
#include <wx/log.h>
namespace opennav::integration {
RecoveryStore::RecoveryStore(const wxString &directory) {
  path_ = wxFileName(directory, "opennav-startup.state").GetFullPath();
  try {
    const auto path = std::filesystem::u8path(path_.ToStdString(wxConvUTF8));
    if (std::filesystem::is_symlink(path))
      throw std::runtime_error("Startup record is a link");
    if (!std::filesystem::exists(path))
      return;
    if (!std::filesystem::is_regular_file(path) ||
        std::filesystem::file_size(path) > 128)
      throw std::runtime_error("Invalid startup record size/type");
    wxFile file(path_);
    wxString text;
    if (!file.IsOpened() || !file.ReadAll(&text, wxConvUTF8))
      throw std::runtime_error("Cannot read startup record");
    file.Close();
    const auto previous = DecodeRecovery(text.ToStdString(wxConvUTF8));
    record_ = ObservePreviousStart(previous);
    if (previous.pending) {
      wxLogWarning("OpenNav startup recovery: previous XNav startup did not "
                   "finish; consecutive failures %u",
                   record_.failed_starts);
      Save();
    }
    if (RecoveryRequired(record_))
      reason_ = "Two XNav startups did not finish. Safe Mode prevents another "
                "automatic attempt.";
  } catch (const std::exception &e) {
    failed_ = true;
    reason_ = std::string("Startup recovery record could not be verified: ") +
              e.what();
    wxLogWarning("OpenNav %s", wxString::FromUTF8(reason_));
  }
}
bool RecoveryStore::Save() {
  wxTempFile file(path_);
  if (!file.IsOpened() ||
      !file.Write(wxString::FromUTF8(EncodeRecovery(record_)), wxConvUTF8) ||
      !file.Commit()) {
    failed_ = true;
    reason_ = "Cannot persist XNav startup recovery state. Check profile "
              "storage and permissions.";
    wxLogWarning("OpenNav %s", wxString::FromUTF8(reason_));
    return false;
  }
  return true;
}
bool RecoveryStore::BeginXNav() {
  if (RequiresSafe())
    return false;
  record_.pending = true;
  attempt_ = Save();
  return attempt_;
}
void RecoveryStore::ObserveHealthy(bool ready, vessel::Time now) {
  if (!attempt_ || !ready || failed_)
    return;
  if (!healthy_since_)
    healthy_since_ = now;
  if (now - *healthy_since_ < std::chrono::seconds(30))
    return;
  record_ = {};
  if (Save()) {
    attempt_ = false;
    wxLogMessage("OpenNav XNav startup healthy after 30 seconds of normal "
                 "application processing");
  }
}
void RecoveryStore::CleanClose() {
  if (!attempt_)
    return;
  record_ = {};
  if (Save()) {
    attempt_ = false;
    wxLogMessage("OpenNav XNav clean close; startup recovery cleared");
  }
}
bool RecoveryStore::Retry() {
  // Preserve the prior record as diagnostic evidence before resetting it.
  try {
    const auto p = std::filesystem::u8path(path_.ToStdString(wxConvUTF8));
    if (std::filesystem::is_symlink(p) ||
        (std::filesystem::exists(p) && (!std::filesystem::is_regular_file(p) ||
                                        std::filesystem::file_size(p) > 128)))
      return false;
  } catch (const std::exception &) {
    return false;
  }
  if (wxFileExists(path_)) {
    const auto copy =
        path_ +
        wxString::Format(".retry-%lld",
                         static_cast<long long>(
                             vessel::Clock::now().time_since_epoch().count()));
    if (!wxCopyFile(path_, copy, false))
      return false;
  }
  record_ = {};
  failed_ = false;
  healthy_since_.reset();
  if (!Save())
    return false;
  wxLogMessage("OpenNav startup recovery explicitly reset for a "
               "human-requested XNav retry");
  return true;
}
} // namespace opennav::integration

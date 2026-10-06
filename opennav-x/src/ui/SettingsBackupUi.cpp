#include "ui/SettingsBackupUi.h"
#include "ui/Sheet.h"
#include <wx/file.h>
#include <wx/filedlg.h>
namespace opennav::ui {
namespace {
const char *filter="SKAGER settings backup (*.skager-backup)|*.skager-backup";
void Result(wxWindow &parent, LightMode light, int scale, const std::string &message) {
  ConfirmSheet(parent, light, "Settings backup", wxString::FromUTF8(message), "Done", scale);
}
}
void ExportSettingsBackup(wxWindow &parent, LightMode light, int scale,
    const std::function<application::SettingsBackup()> &read) {
  if (!read) return;
  try {
    const auto bytes=application::EncodeSettingsBackup(read());
    wxFileDialog choose(&parent,"Export SKAGER settings",{},"SKAGER-settings.skager-backup",
        filter,wxFD_SAVE|wxFD_OVERWRITE_PROMPT);
    if (choose.ShowModal()!=wxID_OK) return;
    // Temp-file commit preserves the selected existing file on write failure.
    wxTempFile file(choose.GetPath());
    if (!file.IsOpened() || !file.Write(bytes.data(),bytes.size()) || !file.Commit())
      throw std::runtime_error("Could not export backup; check the destination and available space");
    Result(parent,light,scale,"Settings backup exported. Charts, licenses, credentials, navigation objects and pilot permissions are excluded.");
  } catch (const std::exception &e) { Result(parent,light,scale,e.what()); }
}
void ImportSettingsBackup(wxWindow &parent, LightMode light, int scale,
    const std::function<application::CommandResult(const application::SettingsBackup &)> &restore) {
  if (!restore) return;
  try {
    wxFileDialog choose(&parent,"Import SKAGER settings",{},{},filter,wxFD_OPEN|wxFD_FILE_MUST_EXIST);
    if (choose.ShowModal()!=wxID_OK) return;
    wxFile file(choose.GetPath(),wxFile::read);
    if (!file.IsOpened()) throw std::runtime_error("Could not open backup; nothing changed");
    // Read at most limit+1 bytes; a growing/untrusted file cannot evade a size check.
    std::string bytes(application::SettingsBackupLimit+1,'\0');
    const auto count=file.Read(bytes.data(),bytes.size());
    if (count<0) throw std::runtime_error("Could not read backup; nothing changed");
    bytes.resize(static_cast<std::size_t>(count));
    const auto backup=application::DecodeSettingsBackup(bytes);
    if (!ConfirmSheet(parent,light,"Review settings restore",
        wxString::FromUTF8(application::SettingsBackupPreview(backup)),"Restore settings",scale)) return;
    const auto result=restore(backup);
    Result(parent,light,scale,result.message);
  } catch (const std::exception &e) { Result(parent,light,scale,std::string(e.what())+"\nNothing was restored."); }
}
}

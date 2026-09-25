#include "diagnostics/Recorder.h"
#include "ui/ProductPanel.h"
#include "ui/Sheet.h"
#include <wx/filedlg.h>
#include <wx/wfstream.h>
#include <wx/zipstrm.h>
namespace opennav::ui {
void ProductPanel::FieldReportPanel() {
  Heading("Field diagnostic bundle", "Reviewable ZIP / no automatic upload");
  Text(
      "Includes build and recovery state, sensor values/age/quality, source "
      "protocol/PGN/instance and anonymous source aliases, energy assumptions, "
      "adapter state, current SmartNav events and the last 128 state "
      "transitions.");
  Text("Default export omits positions, route and waypoint names, AIS "
       "identities, "
       "network/device names, full configuration and raw logs. No folders are "
       "searched. Crash reporting is startup-recovery state, not a memory "
       "dump.");
  BeginActions(2);
  Action(
      "Export Diagnostic Bundle", [this] { ExportFieldReport(false); },
      bool(actions_.field_bundle));
  Action(
      "Export with selected recording...", [this] { ExportFieldReport(true); },
      bool(actions_.field_bundle));
  Action("Commissioning & recordings",
         [this] { ShowPage(ProductPage::Commissioning, mode_); });
  Action("System diagnostics", actions_.diagnostics);
  EndActions();
  Text(
      "A selected recording may contain device/interface names and, if enabled "
      "during capture, vessel positions and route geometry. Inspect the ZIP "
      "before sending it with a description, approximate failure time and "
      "screenshots.");
  Text("Export is bounded: report files at most 256 KiB each, one optional "
       "validated recording at most 8 MiB. A failed export does not replace an "
       "existing bundle. No control command is sent by export.");
}
void ProductPanel::ExportFieldReport(bool include_recording) {
  try {
    std::optional<std::string> recording;
    if (include_recording) {
      wxFileDialog file(this, "Explicitly select recording to share", {}, {},
                        "OpenNav recording (*.onxr)|*.onxr",
                        wxFD_OPEN | wxFD_FILE_MUST_EXIST);
      if (file.ShowModal() != wxID_OK)
        return;
      const auto r = diagnostics::LoadRecording(
          std::filesystem::u8path(file.GetPath().ToStdString(wxConvUTF8)));
      if (!ConfirmSheet(*this, mode_, "Include this recording?",
                        wxString("Device/interface names may be present. "
                                 "Navigation data included: ") +
                            (r.navigation_included
                                 ? "YES. Positions and route geometry may "
                                   "identify your vessel's movements."
                                 : "NO.") +
                            " Review before sharing.",
                        "Include selected recording"))
        return;
      recording = diagnostics::EncodeRecording(r);
    }
    wxFileDialog save(this, "Export Diagnostic Bundle", {},
                      "OpenNavX-Field-Report.zip", "ZIP archive (*.zip)|*.zip",
                      wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
    if (save.ShowModal() != wxID_OK)
      return;
    const auto entries = actions_.field_bundle(recording);
    wxTempFileOutputStream output(save.GetPath());
    if (!output.IsOk())
      throw std::runtime_error("Cannot create bundle in selected directory");
    {
      wxZipOutputStream zip(output);
      for (const auto &e : entries) {
        if (e.name.empty() ||
            e.name.find_first_of("/\\:") != std::string::npos || e.name == "..")
          throw std::runtime_error("Invalid diagnostic archive entry");
        if (!zip.PutNextEntry(wxString::FromUTF8(e.name)))
          throw std::runtime_error("Cannot write bundle entry");
        zip.Write(e.bytes.data(), e.bytes.size());
        if (!zip.IsOk())
          throw std::runtime_error("Cannot write diagnostic bundle");
      }
      if (!zip.Close())
        throw std::runtime_error("Cannot finish diagnostic bundle");
    }
    if (!output.Commit())
      throw std::runtime_error("Cannot publish diagnostic bundle");
    Result({true, "Diagnostic ZIP exported. Review its contents before "
                  "sharing; nothing was uploaded."});
  } catch (const std::exception &e) {
    Result({false, e.what()});
  }
}
} // namespace opennav::ui

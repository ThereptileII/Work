#include "smartnav/VesselEnergy.h"
#include "integration/BuildFeatures.h"
#include "ui/ProductPanel.h"
#include "ui/Sheet.h"
#include "vessel/DataItems.h"
#include <fstream>
#include <wx/filedlg.h>
#include <wx/utils.h>
namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
wxString FilePath(const std::filesystem::path &path) {
  const auto bytes = path.u8string();
  return wxString::FromUTF8(reinterpret_cast<const char *>(bytes.data()),
                            bytes.size());
}
std::filesystem::path Path(const wxString &s) {
  return std::filesystem::u8path(s.ToStdString(wxConvUTF8));
}
} // namespace
void ProductPanel::CommissioningPanel() {
  Heading("Boat commissioning",
          "Source health / bounded recordings / isolated replay");
  const auto service = actions_.commissioning;
  if (!service) {
    Text("Recording service unavailable in this interface mode");
    return;
  }
  BeginActions(3);
  Action("Data sources & precedence",
         [this] { ShowPage(ProductPage::Sources, mode_); });
  Action("System diagnostics", actions_.diagnostics);
  Action("Open recording folder", [service] {
    std::error_code error;
    std::filesystem::create_directories(service->Directory(), error);
    if (!error)
      wxLaunchDefaultApplication(FilePath(service->Directory()));
  });
  EndActions();
  LiveText([service](const auto &) {
    const auto s = service->RecordingStatus();
    return !s.error.empty()
               ? "RECORDING ERROR / " + W(s.error)
               : wxString::Format("Recording %s / %llu captured / %llu saved / "
                                  "%llu retained segments",
                                  s.active ? "ON" : "OFF",
                                  static_cast<unsigned long long>(s.captured),
                                  static_cast<unsigned long long>(s.published),
                                  static_cast<unsigned long long>(s.segments));
  });
  Text("Recording is off at startup. Up to one frame per second; checkpoints "
       "every 10 frames and on Stop. Retains up to three 8 MiB segments per "
       "session. Transport/device names remain in source provenance. No raw "
       "bus stream or connection credentials.");
  BeginActions(2);
  auto start = [this, service](bool navigation) {
    if (navigation &&
        !ConfirmSheet(
            *this, mode_, "Include navigation in recording?",
            "This session will include vessel positions, route names and "
            "waypoint geometry. Share these files only deliberately.",
            "Record with navigation"))
      return;
    auto config = state_.settings;
#if XNAV_ENABLE_TEST_FIXTURES
    if (state_.vessel.simulated) {
      config.energy.battery = smartnav::PreviewEnergyModel(true);
      config.energy.consumption = smartnav::ConsumptionModel::MeasuredPack;
      config.energy.battery_device_id = "DEMO / pack-1";
    }
#endif
    Result(service->StartRecording(config, navigation, vessel::Clock::now()));
  };
  Action("Record instruments only", [start] { start(false); });
  Action("Record with navigation...", [start] { start(true); });
  Action("Stop & save recording",
         [this, service] { Result(service->StopRecording()); });
  Action("Open recording for REPLAY...", [this, service] {
    wxFileDialog file(
        this, "Open normalized recording", FilePath(service->Directory()), "",
        "OpenNav recording (*.onxr)|*.onxr", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (file.ShowModal() != wxID_OK)
      return;
    Result(service->OpenReplay(Path(file.GetPath()), vessel::Clock::now()));
  });
  EndActions();
  LiveText([service](const auto &) {
    const auto replay = service->ReadReplay(vessel::Clock::now());
    return replay ? wxString::Format(
                        "REPLAY / %s / %.1f of %.1f s / hardware control OFF",
                        replay->paused  ? "PAUSED"
                        : replay->ended ? "ENDED - observations age"
                                        : "PLAYING",
                        replay->elapsed.count() / 1000.0,
                        replay->duration.count() / 1000.0)
                  : wxString("Replay OFF");
  });
  Text("REPLAY displays historical OpenNav data on a separate clock. It does "
       "not move OpenCPN's ownship, activate a route or operate equipment. AIS "
       "and anchor state are not recorded in this format. At the end, retained "
       "observations age into stale state.");
  BeginActions(3);
  Action("Pause / resume REPLAY", [service] {
    const auto now = vessel::Clock::now();
    const auto r = service->ReadReplay(now);
    if (r)
      service->PauseReplay(!r->paused, now);
  });
  Action("Rewind REPLAY",
         [service] { service->RewindReplay(vessel::Clock::now()); });
  Action("Stop REPLAY", [service] { service->StopReplay(); });
  EndActions();
  Action("Export calibration observations...", [this, service] {
    wxFileDialog input(
        this, "Recording to export", FilePath(service->Directory()), "",
        "OpenNav recording (*.onxr)|*.onxr", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (input.ShowModal() != wxID_OK)
      return;
    auto device = state_.settings.energy.battery_device_id;
#if XNAV_ENABLE_TEST_FIXTURES
    if (state_.vessel.simulated) device = "DEMO / pack-1";
#endif
    auto fields = EditSheet(
        *this, mode_, "Calibration observations",
        "Export finite, fresh, coherent speed/power pairs for review. This "
        "does not install a boat power curve. Use the exact recorded device "
        "identity.",
        {{"Speed reference: STW or SOG", "STW", 3},
         {"Power: whole-pack, motor-electrical or shaft", "whole-pack", 20},
         {"Power device identity",
          W(device),
          1024}},
        "Export...");
    if (!fields)
      return;
    try {
      diagnostics::CalibrationSelection selection;
      if ((*fields)[0] == "STW")
        selection.speed = smartnav::SpeedReference::ThroughWater;
      else if ((*fields)[0] == "SOG")
        selection.speed = smartnav::SpeedReference::OverGround;
      else
        throw std::invalid_argument("Choose STW or SOG explicitly");
      if ((*fields)[1] == "whole-pack")
        selection.power = smartnav::PowerBasis::WholePack;
      else if ((*fields)[1] == "motor-electrical")
        selection.power = smartnav::PowerBasis::MotorElectrical;
      else if ((*fields)[1] == "shaft")
        selection.power = smartnav::PowerBasis::Shaft;
      else
        throw std::invalid_argument("Unknown power basis");
      selection.device = (*fields)[2];
      const auto output = diagnostics::ExportCalibration(
          diagnostics::LoadRecording(Path(input.GetPath())), selection);
      if (!output.pairs)
        throw std::invalid_argument(
            "No valid coherent observations for this reference/device. Check "
            "identity, age, speed and power sign.");
      wxFileDialog save(this, "Save reviewed calibration observations",
                        FilePath(service->Directory()),
                        "calibration-observations.csv", "CSV (*.csv)|*.csv",
                        wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
      if (save.ShowModal() != wxID_OK)
        return;
      std::ofstream file(Path(save.GetPath()),
                         std::ios::binary | std::ios::trunc);
      file.exceptions(std::ios::failbit | std::ios::badbit);
      file << output.csv;
      file.close();
      Result({true, "Exported " + std::to_string(output.pairs) +
                        " pairs; omitted " + std::to_string(output.rejected) +
                        " unsuitable frames and " +
                        std::to_string(output.duplicate) +
                        " duplicate observations. Review before "
                        "deriving/importing a power curve."});
    } catch (const std::exception &e) {
      Result({false, e.what()});
    }
  });
  Heading("Bench overview",
          "Verify one subsystem at a time / source detail in Data Sources");
  for (const auto &item : vessel::DataItems(state_.vessel)) {
    const std::string name = item.name, unit = item.unit;
    LiveText([name, unit](const auto &s) {
      for (const auto &v : vessel::DataItems(s.vessel))
        if (name == v.name) {
          const auto a = vessel::Assess(*v.sample, s.now);
          return W(name) + " / " +
                 (a.value ? wxString::Format("%.2f ", *a.value) + W(unit)
                          : "No data") +
                 " / " + W(vessel::QualityName(a.quality)) +
                 (a.age ? wxString::Format(" / %.2f s", a.age->count() / 1000.0)
                        : "") +
                 " / " + W(v.sample->source);
        }
      return wxString{};
    });
  }
}
} // namespace opennav::ui

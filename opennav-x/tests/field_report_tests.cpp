#include "diagnostics/FieldReport.h"
#include "vessel/DemoSource.h"
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace std::chrono_literals;
void Check(bool ok, const char *why) {
  if (!ok)
    throw std::runtime_error(why);
}
diagnostics::FieldSnapshot Fixture() {
  diagnostics::FieldSnapshot s;
  s.now = vessel::Time{100s};
  s.vessel = vessel::DemoFixture(vessel::DemoScenario::Cruise, 0, s.now);
  s.vessel.battery.soc_percent.source =
      "NMEA2000/tcp://private:password@192.168.0.1/NAME-secret/instance-0/"
      "PGN-127506";
  s.vessel.battery.soc_percent.device_id = "secret-device";
  s.settings.energy.battery_device_id = "secret-device";
  s.settings.energy.curve.source = "C:/private/secret-file.csv";
  s.settings.pilot.interface_id = "secret-interface";
  s.sources.push_back({vessel::Quantity::BatterySoc, "secret-id",
                       s.vessel.battery.soc_percent, true, 4, 2.5, 8, 2});
  smartnav::AdvisoryEvent e;
  e.kind = smartnav::EventKind::AisEncounter;
  e.title = "secret AIS name";
  e.detail = "secret MMSI";
  e.source = "secret AIS source";
  e.identity = "secret AIS identity";
  e.seconds_from_now = 60;
  s.advice.events.push_back(e);
  return s;
}
std::string Join(const std::vector<diagnostics::BundleEntry> &entries) {
  std::string s;
  for (const auto &e : entries)
    s += e.name + e.bytes;
  return s;
}
void Privacy() {
  auto s = Fixture();
  diagnostics::FieldJournal journal;
  journal.Observe(s, s.now);
  const auto entries = diagnostics::BuildFieldReport(s, {}, journal, s.now);
  auto text = Join(entries);
  for (const auto *bad : {"secret", "password", "192.168.0.1", "private",
                          "DEMO-route", "selected-recording.onxr"})
    Check(text.find(bad) == std::string::npos, "Private input leaked");
  Check(text.find("PGN 127506 / instance 0") != std::string::npos,
        "Protocol metadata missing");
  Check(text.find("selected 1 / Hz 2.5 / received 8 / invalid 2") !=
            std::string::npos,
        "Source diagnostics missing");
  Check(text.find("Latitude / deg / withheld") != std::string::npos,
        "Position not withheld");
  for (const auto &e : entries)
    Check(e.name.find_first_of("/\\:") == std::string::npos,
          "Archive path escape");
  auto changed = s;
  changed.now += 20s;
  journal.Observe(changed, changed.now);
  text = Join(diagnostics::BuildFieldReport(changed, {}, journal, changed.now));
  Check(text.find("STALE") != std::string::npos &&
            text.find("age 20 s") != std::string::npos,
        "Export refreshed retained samples");
}
void Limits() {
  auto s = Fixture();
  diagnostics::FieldJournal journal;
  for (int i = 0; i < 1000; ++i) {
    s.pilot.enabled = i % 2;
    s.pilot.command.request.id = i;
    journal.Observe(s, s.now + i * 1s);
  }
  Check(journal.Size() == 128, "Journal not bounded");
  Check(journal.Export(s.now + 1000s).size() < 32768,
        "Journal text not bounded");
  diagnostics::FieldEnvironment e;
  e.build.push_back(std::string(10000, 'x') + "\nBadLine");
  auto text = Join(diagnostics::BuildFieldReport(s, e, journal, s.now));
  Check(text.find("BadLine") == std::string::npos,
        "Oversize metadata not bounded");
  s.sources.resize(1025);
  bool rejected = false;
  try {
    diagnostics::BuildFieldReport(s, {}, journal, s.now);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  Check(rejected, "Source count not bounded");
}
void Recording() {
  auto s = Fixture();
  diagnostics::Recording r;
  r.frames.push_back(diagnostics::CaptureFrame(s.vessel, s.now, s.now, false));
  auto bytes = diagnostics::EncodeRecording(r);
  diagnostics::FieldJournal journal;
  const auto entries =
      diagnostics::BuildFieldReport(s, {}, journal, s.now, bytes);
  bool found = false;
  for (const auto &e : entries)
    if (e.name == "selected-recording.onxr") {
      found = true;
      Check(e.bytes == bytes, "Selected recording changed");
    }
  Check(found, "Explicit recording absent");
  bool rejected = false;
  try {
    diagnostics::BuildFieldReport(s, {}, journal, s.now, "corrupt recording");
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  Check(rejected, "Corrupt recording bundled");
}
int main(int argc, char **argv) {
  try {
    std::string g = argc > 1 ? argv[1] : "";
    if (g == "privacy")
      Privacy();
    else if (g == "limits")
      Limits();
    else if (g == "recording")
      Recording();
    else
      throw std::runtime_error("Unknown group");
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}

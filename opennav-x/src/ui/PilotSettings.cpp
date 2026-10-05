#include "ui/ProductPanel.h"
#include "ui/Sheet.h"
#include "integration/BuildFeatures.h"
#include "application/PilotPresentation.h"

namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
} // namespace
void ProductPanel::PilotSettings() {
  const bool test_output = integration::PilotLoopbackTestsEnabled();
  if (!test_output) {
    Heading("Autopilot status", "Live feedback through OpenCPN");
    LiveText([](const auto &s) {
      const auto view = application::PresentPilot(
          s.pilot, s.now, false, s.vessel.replayed);
      return W(view.state + " / " + view.connection);
    });
    Text("Uses the existing OpenCPN receive connections, including the connection used by AutoTrack. No separate SKAGER pilot setup is required. Status appears only after a supported device identity and fresh physical pilot feedback are observed.");
    LiveText([](const auto &s) { return W(s.pilot.adapter_status); });
    Text("Status only. SKAGER equipment commands are unavailable; use the physical helm. Connection settings and AutoTrack configuration remain in OpenCPN.");
    BeginActions(2);
    Action("Back to autopilot", [this] { ShowPage(ProductPage::Pilot, mode_); });
    Action("OpenCPN preferences", [this] {
      if (actions_.navigation.legacy_settings) actions_.navigation.legacy_settings();
    }, bool(actions_.navigation.legacy_settings));
    EndActions();
    return;
  }
  Heading("Autopilot setup", test_output ? "Developer loopback test only" : "Pilot status / equipment control unavailable");
  const auto &b = state_.settings.pilot;
  LiveText([](const auto &s) {
    return wxString(!integration::PilotLoopbackTestsEnabled() ? "Status only / SKAGER cannot command equipment" : s.settings.pilot.permit_control ? "Loopback test permission configured" : "Autopilot control OFF") +
           (s.pilot.fresh ? " / Connected" : " / Waiting for pilot feedback");
  });
  Text(test_output ? "Loopback testing only. Manual control must also be enabled each session. SmartNav never steers the vessel."
                  : "This product displays observed pilot status. Physical equipment commands are unavailable. Use the physical helm. Saved permissions from older builds cannot enable SKAGER control.");
  BeginActions(2);
  Action("Back to manual autopilot", [this] { ShowPage(ProductPage::Pilot, mode_); });
  if (test_output) Action(b.permit_control ? "Return to display-only" : "Permit loopback test control...", [this] {
    auto s = actions_.settings();
    if (!s.pilot.permit_control && !ConfirmSheet(*this, mode_, "Permit local pilot test commands?",
        "Only the verified local TCP test peer can receive output. Actual test control stays OFF until you enable it for this session.",
        "Save test permission")) return;
    s.pilot.permit_control = !s.pilot.permit_control;
    SaveSettings(s);
  }, !b.interface_id.empty() && !state_.vessel.replayed && !state_.vessel.simulated);
  Action(pilot_advanced_ ? "Hide connection details" : "Advanced connection setup", [this] {
    pilot_advanced_ = !pilot_advanced_; Build();
  });
  EndActions();
  if (!pilot_advanced_) {
    Text(test_output ? "STANDBY, AUTO and course changes are restricted to a verified local test peer. TRACK and WIND remain unavailable."
                    : "STANDBY, AUTO, TRACK, WIND and course controls are unavailable. Advanced setup binds read-only feedback to the correct observed device.");
    return;
  }
  Heading("Connection & diagnostics", "Advanced / Exact device identity");
  Text("The ST4000 translator publishes physical SeaTalk feedback through the NMEA 2000 adapter. Binding an observed identity enables status display only; it does not qualify an equipment control path.");
  Text("Interface: " + W(b.interface_id.empty() ? "Unconfigured" : b.interface_id) +
       "\nNAME: " + W(b.name.empty() ? "Unconfigured" : b.name));
  LiveText([](const auto &s) { return W(s.pilot.adapter_status); });
  BeginActions(2);
  Action(
      "Configure translator identity",
      [this] {
        auto s = actions_.settings();
        const auto fields = EditSheet(
            *this, mode_, "ST4000 translator identity",
            "Use the exact observed OpenCPN interface and 16 lowercase "
            "hexadecimal NAME digits. "
            "Do not use a guessed address or a decimal message label. Saving "
            "always returns to display-only.",
            {{"OpenCPN NMEA2000 interface", W(s.pilot.interface_id), 200},
             {"Observed translator NAME", W(s.pilot.name), 16}});
        if (!fields)
          return;
        s.pilot = {(*fields)[0], (*fields)[1], false};
        SaveSettings(s);
      },
      !state_.vessel.replayed);
  if (test_output) Action(
      "Refresh device identity",
      [this] {
        if (actions_.pilot_identity)
          Result(actions_.pilot_identity());
      },
      !b.interface_id.empty() && !state_.vessel.replayed &&
          !state_.vessel.simulated);
  Action(
      "Remove translator binding",
      [this] {
        if (!ConfirmSheet(*this, mode_, "Remove translator binding?",
                          "SKAGER control will be disabled. This cannot recall a "
                          "command already transmitted; "
                          "use physical STANDBY if its outcome is uncertain.",
                          "Remove binding"))
          return;
        auto s = actions_.settings();
        s.pilot = {};
        SaveSettings(s);
      },
      !b.interface_id.empty() && !state_.vessel.replayed);
  EndActions();
  Heading("Observed compatible identities",
          "Actual address claims / not permission to operate equipment");
  LiveText([](const auto &s) {
    if (s.pilot_sources.empty())
      return wxString("No compatible translator claim observed. "
                      "Verify the receive connection and wait for an observed address claim. "
                      "The installed product does not request identity on the bus.");
    wxString text;
    for (const auto &identity : s.pilot_sources)
      text += W(identity) + "\n";
    return text;
  });
  LiveText([](const auto &s) {
    wxString log = "RECENT COMMANDS";
    const auto start = s.pilot_log.size() > 8 ? s.pilot_log.size() - 8 : 0;
    for (std::size_t i = start; i < s.pilot_log.size(); ++i)
      log += "\n" + W(adapters::CommandStateName(s.pilot_log[i].state)) + " / " + W(s.pilot_log[i].detail);
    return log;
  });

}
} // namespace opennav::ui

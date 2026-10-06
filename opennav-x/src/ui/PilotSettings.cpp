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
  if (!test_output && !integration::PilotManualSerialEnabled()) {
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
  Heading("Autopilot setup", test_output ? "Developer loopback test only" : "Manual serial commissioning / control starts OFF");
  const auto &b = state_.settings.pilot;
  LiveText([](const auto &s) {
    return wxString(s.settings.pilot.permit_control ? "Manual commissioning permission configured" : "Autopilot control OFF") +
           (s.pilot.fresh ? " / Connected" : " / Waiting for pilot feedback");
  });
  Text(test_output ? "Loopback testing only. Manual control must also be enabled each session. SmartNav never steers the vessel."
                  : "Manual commissioning uses the existing bidirectional OpenCPN Actisense serial connection. Verify the observed translator identity, permit manual control, then explicitly enable this session. Keep physical STANDBY available. SmartNav never steers.");
  BeginActions(2);
  Action("Back to manual autopilot", [this] { ShowPage(ProductPage::Pilot, mode_); });
  Action(b.permit_control ? "Return to display-only" : "Permit manual commissioning...", [this] {
    auto s = actions_.settings();
    if (!s.pilot.permit_control && !ConfirmSheet(*this, mode_, "Permit manual pilot commands?",
        "Only the exact observed translator on the selected connection can receive the six manual commands. Control stays OFF until you enable it for this session. Keep the physical helm available.",
        "Save manual permission")) return;
    s.pilot.permit_control = !s.pilot.permit_control;
    SaveSettings(s);
  }, !b.interface_id.empty() && !b.name.empty() && !state_.vessel.replayed && !state_.vessel.simulated);
  Action(pilot_advanced_ ? "Hide connection details" : "Advanced connection setup", [this] {
    pilot_advanced_ = !pilot_advanced_; Build();
  });
  EndActions();
  if (!pilot_advanced_) {
    Text(test_output ? "STANDBY, AUTO and course changes are restricted to a verified local test peer. TRACK and WIND remain unavailable."
                    : "Only STANDBY, AUTO and -1/+1/-10/+10 are supported. TRACK and WIND remain unavailable. Reconnects and stale feedback disable the session.");
    return;
  }
  Heading("Connection & diagnostics", "Advanced / Exact device identity");
  Text("Select the same existing serial connection used by AutoTrack. Refresh identity if this PC joined the bus after the translator. Copy only an actually observed NAME. A heading address alone never identifies a pilot.");
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
            "hexadecimal NAME digits. Leave NAME empty to request identity first. "
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
  Action(
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
                      "Refresh device identity sends one non-steering address-claim request on the selected bidirectional connection; it does not enable control.");
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

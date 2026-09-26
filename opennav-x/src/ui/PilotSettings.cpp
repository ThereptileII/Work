#include "ui/ProductPanel.h"
#include "ui/Sheet.h"

namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
} // namespace
void ProductPanel::PilotSettings() {
  Heading("Autopilot setup", "Display-only until deliberately enabled");
  const auto &b = state_.settings.pilot;
  LiveText([](const auto &s) {
    return wxString(s.settings.pilot.permit_control ? "Manual control permitted" : "Autopilot control OFF") +
           (s.pilot.fresh ? " / Connected" : " / Waiting for pilot feedback");
  });
  Text("Saving permission does not engage the pilot. Manual control must also be enabled each session. SmartNav never steers the vessel.");
  BeginActions(2);
  Action("Back to manual autopilot", [this] { ShowPage(ProductPage::Pilot, mode_); });
  Action(b.permit_control ? "Return to display-only" : "Permit manual live control...", [this] {
    auto s = actions_.settings();
    if (!s.pilot.permit_control && !ConfirmSheet(*this, mode_, "Permit physical pilot commands?",
        "Verify the connected pilot, transport and secured-vessel commissioning checks first. Actual control stays OFF until you enable it for this session.",
        "Save manual permission")) return;
    s.pilot.permit_control = !s.pilot.permit_control;
    SaveSettings(s);
  }, !b.interface_id.empty() && !state_.vessel.replayed && !state_.vessel.simulated);
  Action(pilot_advanced_ ? "Hide connection details" : "Advanced connection setup", [this] {
    pilot_advanced_ = !pilot_advanced_; Build();
  });
  EndActions();
  if (!pilot_advanced_) {
    Text("STANDBY, AUTO and manual course changes are available only with a verified compatible pilot. TRACK and WIND remain unavailable.");
    return;
  }
  Heading("Connection & diagnostics", "Advanced / Exact device identity");
  Text("The ST4000 translator publishes physical SeaTalk feedback through the NMEA 2000 adapter. Control requires an existing bidirectional TCP Actisense connection. Other transports are status-only.");
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
                          "XNav control will be disabled. This cannot recall a "
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
                      "Verify the connection; refresh a configured identity or "
                      "power-cycle the bridge during secured commissioning.");
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

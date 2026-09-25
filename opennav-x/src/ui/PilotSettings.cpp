#include "ui/ProductPanel.h"
#include "ui/Sheet.h"

namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
} // namespace
void ProductPanel::PilotSettings() {
  Heading("Autopilot configuration",
          "Exact device binding / display-only by default");
  Action("Back to manual autopilot",
         [this] { ShowPage(ProductPage::Pilot, mode_); });
  Text("The supported ST4000 translator must publish fresh physical SeaTalk "
       "status. "
       "Live output currently uses an existing bidirectional OpenCPN TCP "
       "connection "
       "carrying Actisense complete-PGN ASCII. Other transports remain "
       "status-only. "
       "TRACK and WIND commands remain unavailable pending physical "
       "validation.");
  const auto &b = state_.settings.pilot;
  Text("Interface: " + W(b.interface.empty() ? "Unconfigured" : b.interface) +
       "\nNAME: " + W(b.name.empty() ? "Unconfigured" : b.name) +
       "\nPermission: " +
       (b.permit_control
            ? "Manual control permitted / session enable still required"
            : "DISPLAY ONLY / control OFF"));
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
            {{"OpenCPN NMEA2000 interface", W(s.pilot.interface), 200},
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
      !b.interface.empty() && !state_.vessel.replayed &&
          !state_.vessel.simulated);
  Action(
      b.permit_control ? "Return to display-only"
                       : "Permit manual live control...",
      [this] {
        auto s = actions_.settings();
        if (!s.pilot.permit_control &&
            !ConfirmSheet(
                *this, mode_, "Permit physical pilot commands?",
                "This saves permission for the configured device only. Actual "
                "control stays OFF until "
                "you deliberately enable each session. Verify the translator "
                "firmware and transport "
                "in the boat commissioning checklist before enabling.",
                "Save manual permission"))
          return;
        s.pilot.permit_control = !s.pilot.permit_control;
        SaveSettings(s);
      },
      !b.interface.empty() && !state_.vessel.replayed &&
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
      !b.interface.empty() && !state_.vessel.replayed);
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
}
} // namespace opennav::ui

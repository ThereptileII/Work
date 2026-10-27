// SCRUM-379: SKAGER Helm settings. Presentation only: the Helm Server owns
// pairing, sessions and the screen stream. The pairing secret is shown here
// (QR code and key) only while the owner has the pairing window open.
#include "ui/ProductPanel.h"
#include "ui/Sheet.h"
#include <algorithm>

namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }

wxString StatusLine(const helm::HelmSnapshot &s) {
  switch (s.state) {
  case helm::HostState::Off: return "SKAGER Helm: off";
  case helm::HostState::Missing: return "SKAGER Helm is not included in this installation.";
  case helm::HostState::Starting: return "SKAGER Helm: starting...";
  case helm::HostState::Failed: return s.message.empty() ? wxString("SKAGER Helm stopped.") : W(s.message);
  case helm::HostState::Ready: break;
  }
  wxString text = s.connected_name.empty() ? wxString("SKAGER Helm: on. No phone or tablet connected.")
                                           : "SKAGER Helm: on. Shown on " + W(s.connected_name) + ".";
  if (!s.connected_name.empty() && !s.screen_available)
    text += "\nThe SKAGER window is minimized or the computer is locked: the phone shows a notice.";
  return text;
}

// "192.168.1.20:43872" → "192.168.1.20" (the app adds the default port).
wxString Address(const helm::HelmSnapshot &s) {
  if (s.addresses.empty()) return {};
  std::string a = s.addresses.front();
  const auto colon = a.rfind(':');
  if (colon != std::string::npos && a.substr(colon + 1) == "43872") a.resize(colon);
  return W(a);
}

wxString Day(const std::string &iso) { return iso.size() >= 10 ? W(iso.substr(0, 10)) : wxString(); }
}  // namespace

// True when the page needs different controls, not merely different text.
bool ProductPanel::RefreshHelm() {
  if (!actions_.helm.read) return false;
  helm_ = actions_.helm.read();
  std::string shape = std::to_string(static_cast<int>(helm_.state)) + (helm_.pairing ? "P" + helm_.pairing->key : "-");
  for (const auto &d : helm_.devices) shape += "|" + d.id + (d.connected ? "*" : "");
  const bool changed = shape != helm_shape_;
  helm_shape_ = shape;
  return changed;
}

void ProductPanel::HelmPage() {
  RefreshHelm();
  Heading("SKAGER Helm", "This screen on phones and tablets on board");
  if (!actions_.helm.read || !actions_.helm.enable) {
    Text("SKAGER Helm is unavailable in this build.");
    return;
  }
  const auto rebuild = [this] {
    RefreshHelm();
    Build();
  };
  const bool on = actions_.helm.enabled && actions_.helm.enabled();

  if (!on || helm_.state == helm::HostState::Off) {
    Text("Use SKAGER from your phone or tablet anywhere on board: see the chart, instruments and "
         "alarms and touch them as on this screen. Install the SKAGER Helm app, switch Helm on "
         "here and pair your phone with the code SKAGER shows.");
    Text("Helm works over the boat's own Wi-Fi and needs no internet. Only paired phones can "
         "connect, and the picture and touches are encrypted end to end.", 12);
    BeginActions(1, 260);
    Action("Switch on SKAGER Helm", [this, rebuild] {
      actions_.helm.enable(true);
      rebuild();
    })->SetRole(ButtonRole::Primary);
    EndActions();
    Text("Autopilot commands are refused from SKAGER Helm: steer and engage the pilot at the helm.", 12);
    return;
  }
  if (helm_.state == helm::HostState::Missing) {
    Text("This SKAGER installation does not include the Helm server (skager-helm). "
         "Install a SKAGER version with SKAGER Helm to use it.");
    BeginActions(1, 200);
    Action("Switch off", [this, rebuild] { actions_.helm.enable(false); rebuild(); })->SetRole(ButtonRole::Quiet);
    EndActions();
    return;
  }

  LiveText([this](const ProductState &) { return StatusLine(helm_); });

  if (helm_.pairing) {
    const auto pairing = *helm_.pairing;
    const int side = 236;
    Visual("Pairing code", side + 112, [this, pairing, side](XNavPainter &p, wxDC &dc, int width) {
      p.Card(0, 0, width, side + 108, "PAIR A PHONE OR TABLET");
      // Dark modules on light, with the quiet zone, in every light mode: cameras need contrast.
      const int modules = pairing.qr_size + 8;
      const int cell = std::max(2, p.D(side - 24) / modules);
      const int x0 = p.D(20), y0 = p.D(48);
      dc.SetPen(*wxTRANSPARENT_PEN);
      dc.SetBrush(wxBrush(wxColour(255, 255, 255)));
      dc.DrawRectangle(x0, y0, cell * modules, cell * modules);
      dc.SetBrush(wxBrush(wxColour(0, 0, 0)));
      for (int y = 0; y < pairing.qr_size; ++y)
        for (int x = 0; x < pairing.qr_size; ++x)
          if (pairing.qr_rows[static_cast<size_t>(y)][static_cast<size_t>(x)] == '1')
            dc.DrawRectangle(x0 + (x + 4) * cell, y0 + (y + 4) * cell, cell, cell);
      const int text_x = side + 12, text_width = width - text_x - 20;
      if (text_width < 160) return;
      p.Wrapped("Open the SKAGER Helm app on your phone and tap Scan pairing code. Or scan with the phone's camera.",
                text_x, 52, 14, 21, text_width, p.c.secondary, 4);
      p.Text("PAIRING KEY", text_x, 150, 11, p.c.muted, true);
      p.TextWeight(W(pairing.key), text_x, 168, 22, p.c.primary, 650, text_width);
      p.Text("ADDRESS", text_x, 214, 11, p.c.muted, true);
      p.TextWeight(Address(helm_), text_x, 232, 18, p.c.primary, 500, text_width);
    });
    LiveText([this](const ProductState &) {
      if (!helm_.pairing) return wxString();
      const auto left = std::chrono::duration_cast<std::chrono::seconds>(
          helm_.pairing->expires - std::chrono::system_clock::now()).count();
      if (left <= 0) return wxString("The code has expired. Tap Pair a phone or tablet for a new one.");
      return wxString::Format("The code works once and for %lld:%02lld more minutes.",
                              static_cast<long long>(left / 60), static_cast<long long>(left % 60));
    });
    BeginActions(1, 200);
    Action("Cancel pairing", [this, rebuild] {
      if (actions_.helm.cancel_pair) actions_.helm.cancel_pair();
      rebuild();
    })->SetRole(ButtonRole::Quiet);
    EndActions();
    Text("Show this code only to people you want to give control of SKAGER.", 12);
  } else {
    BeginActions(1, 260);
    Action("Pair a phone or tablet", [this, rebuild] {
      if (actions_.helm.pair) actions_.helm.pair();
      rebuild();
    }, helm_.state == helm::HostState::Ready && bool(actions_.helm.pair))->SetRole(ButtonRole::Primary);
    EndActions();
  }

  Text("Paired phones and tablets", 20);
  if (helm_.devices.empty()) {
    Text("None yet.", 12);
  } else {
    for (const auto &d : helm_.devices) {
      const wxString id = W(d.id);
      const wxString name = W(d.name);
      const wxString status = d.connected ? wxString("Connected now")
                              : d.last_seen.empty() ? "Paired " + Day(d.paired_at)
                                                    : "Last used " + Day(d.last_seen);
      Text(name + "  /  " + status, 14);
      BeginActions(d.connected ? 2 : 1, 200);
      if (d.connected)
        Action("Disconnect", [this, rebuild] {
          if (actions_.helm.disconnect) actions_.helm.disconnect();
          rebuild();
        })->SetRole(ButtonRole::Quiet);
      Action("Remove " + name, [this, rebuild, id, name] {
        if (!actions_.helm.revoke ||
            !ConfirmSheet(*this, mode_, "Remove " + name,
                          name + " can no longer connect to this SKAGER. To use it again, pair it again here.",
                          "Remove", interface_scale_))
          return;
        actions_.helm.revoke(id.ToStdString(wxConvUTF8));
        rebuild();
      })->SetRole(ButtonRole::Critical);
      EndActions();
    }
  }

  Text("How it works", 20);
  Text("A paired phone shows this screen and can touch it like this screen. One phone or tablet "
       "at a time; another paired phone can take over. Touches that arrive late are dropped, and a "
       "phone that loses the connection can no longer touch anything.", 12);
  Text("Autopilot commands are refused when they come from SKAGER Helm: the phone shows the "
       "pilot's status, and the pilot is steered at the helm.", 12);
  BeginActions(1, 200);
  Action("Switch off SKAGER Helm", [this, rebuild] {
    actions_.helm.enable(false);
    rebuild();
  })->SetRole(ButtonRole::Quiet);
  EndActions();
}
}  // namespace opennav::ui

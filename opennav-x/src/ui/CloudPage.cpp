// SCRUM-44/369: SKAGER Cloud settings. Presentation only: pairing, status and
// sharing run on the cloud worker; no token is ever displayed. SKAGER keeps
// working without an account; Cloud adds automatic updates and the web view.
#include "ui/ProductPanel.h"
#include "ui/Sheet.h"
#include <wx/datetime.h>

namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
wxString LocalTime(cloud::WallTime t) {
  return wxDateTime(static_cast<time_t>(
      std::chrono::duration_cast<std::chrono::seconds>(t.time_since_epoch()).count()))
      .Format("%H:%M");
}
// "2026-11-19T10:00:00+00:00" -> "2026-11-19". The day is enough here.
wxString Day(const std::string &iso) { return iso.size() >= 10 ? W(iso.substr(0, 10)) : wxString(); }
wxString LinkAddress(const cloud::CloudSnapshot &s) {
  auto url = s.verification_url.empty() ? std::string("https://beta.skager.app/link/") : s.verification_url;
  for (const char *prefix : {"https://", "http://"})
    if (url.rfind(prefix, 0) == 0) url = url.substr(std::string(prefix).size());
  if (!url.empty() && url.back() == '/') url.pop_back();
  return W(url);
}
wxString CloudLine(const cloud::CloudSnapshot &s) {
  if (!s.account) return "Checking your account...";
  const auto &a = *s.account;
  if (!a.cloud_active) return "SKAGER Cloud: not active";
  if (a.cancel_at_period_end && !a.renews_at.empty()) return "SKAGER Cloud: active until " + Day(a.renews_at);
  if (a.cloud_status == "past_due") return "SKAGER Cloud: payment due - update your card in the account";
  return a.renews_at.empty() ? wxString("SKAGER Cloud: active") : "SKAGER Cloud: active, renews " + Day(a.renews_at);
}
wxString ShareLine(const cloud::CloudSnapshot &s) {
  if (!s.share_enabled) return "Boat data: not shared";
  if (s.account && !s.account->cloud_active) return "Boat data: on, waiting for SKAGER Cloud";
  if (s.last_shared) return "Boat data: shared, last sent " + LocalTime(*s.last_shared);
  return "Boat data: on, waiting for measured data";
}
}  // namespace

// True when the page needs different controls, not merely different text.
bool ProductPanel::RefreshCloud() {
  if (!actions_.cloud.read) return false;
  cloud_ = actions_.cloud.read();
  const int shape = static_cast<int>(cloud_.link) * 16 + (cloud_.account ? 8 : 0) +
                    (cloud_.account && cloud_.account->cloud_active ? 4 : 0) + (cloud_.share_enabled ? 2 : 0);
  const bool changed = shape != cloud_shape_;
  cloud_shape_ = shape;
  return changed;
}

void ProductPanel::CloudPage() {
  RefreshCloud();
  Heading("SKAGER Cloud", "Your account, automatic updates and boat data");
  if (!actions_.cloud.read) {
    Text("SKAGER Cloud is unavailable in this build.");
    return;
  }
  const auto result = [this](application::CommandResult r) {
    RefreshCloud();
    Build();
    Result(std::move(r));
  };
  const auto message = [this](const ProductState &) { return W(cloud_.message); };
  switch (cloud_.link) {
  case cloud::LinkState::Unlinked:
  case cloud::LinkState::Starting: {
    Text("Link this SKAGER to your SKAGER account. With SKAGER Cloud you get automatic "
         "updates and can follow your boat on beta.skager.app.");
    Text("SKAGER keeps working without an account. Without SKAGER Cloud you can still "
         "download new versions from your account and install them yourself.", 12);
    const bool starting = cloud_.link == cloud::LinkState::Starting;
    BeginActions(1, 260);
    Action(starting ? "Requesting code..." : "Link to my account", [this, result] {
      if (actions_.cloud.link) result(actions_.cloud.link());
    }, !starting && bool(actions_.cloud.link))->SetRole(ButtonRole::Primary);
    EndActions();
    LiveText(message);
    return;
  }
  case cloud::LinkState::WaitingForApproval: {
    Visual("Pairing code", 196, [this](XNavPainter &p, wxDC &, int width) {
      p.Card(0, 0, width, 192, "PAIRING CODE");
      p.TextWeight(W(cloud::FormatUserCode(cloud_.user_code)), 24, 44, 40, p.c.primary, 700, width - 48);
      p.Wrapped("On your phone or computer, open " + LinkAddress(cloud_) +
                ", sign in and enter this code.", 24, 108, 14, 21, width - 48, p.c.secondary, 3);
    });
    LiveText([this](const ProductState &) {
      const auto left = std::chrono::duration_cast<std::chrono::seconds>(
          cloud_.code_expires - std::chrono::system_clock::now()).count();
      if (left <= 0) return wxString("The code has expired.");
      return wxString::Format("The code is valid for %lld:%02lld more minutes.",
                              static_cast<long long>(left / 60), static_cast<long long>(left % 60));
    });
    BeginActions(1, 200);
    Action("Cancel", [this, result] {
      if (actions_.cloud.cancel_link) result(actions_.cloud.cancel_link());
    }, bool(actions_.cloud.cancel_link))->SetRole(ButtonRole::Quiet);
    EndActions();
    Text("Only approve a code that this screen is showing right now.", 12);
    return;
  }
  case cloud::LinkState::Linked: break;
  }

  LiveText([this](const ProductState &) {
    wxString text = cloud_.account ? "Account: " + W(cloud_.account->email) : wxString("Account: linked");
    text += "\n" + CloudLine(cloud_);
    if (cloud_.account)
      text += cloud_.account->cloud_active ? "\nAutomatic updates: on"
                                           : "\nAutomatic updates: off (part of SKAGER Cloud)";
    text += "\n" + ShareLine(cloud_);
    return text;
  });
  if (cloud_.account && !cloud_.account->cloud_active)
    Text("Start SKAGER Cloud under Cloud in your account at beta.skager.app. "
         "SKAGER notices the change within a few minutes, or choose Check now.", 12);

  Text("Share boat data", 20);
  BeginActions(2);
  const auto share = [this, result](bool on) {
    if (actions_.cloud.share) result(actions_.cloud.share(on));
  };
  auto *off = Action("Off", [share] { share(false); }, bool(actions_.cloud.share));
  off->SetSelected(!cloud_.share_enabled);
  auto *on = Action("On", [share] { share(true); }, bool(actions_.cloud.share));
  on->SetSelected(cloud_.share_enabled);
  EndActions();
  Text("When on, SKAGER sends position, speed, wind, depth, battery, motor and tank readings "
       "with their age and source every 15 seconds while it runs. Only you can see them in "
       "your account. History is kept for 30 days. Simulated and replayed data is never sent.", 12);

  BeginActions(2);
  Action("Check now", [this] {
    if (actions_.cloud.refresh) actions_.cloud.refresh();
  }, bool(actions_.cloud.refresh))->SetRole(ButtonRole::Quiet);
  Action("Unlink this SKAGER", [this, result] {
    if (!actions_.cloud.unlink ||
        !ConfirmSheet(*this, mode_, "Unlink this SKAGER",
                      "Automatic updates and boat data sharing stop on this computer. "
                      "Your account and SKAGER Cloud subscription are not changed.",
                      "Unlink", interface_scale_))
      return;
    result(actions_.cloud.unlink());
  }, bool(actions_.cloud.unlink))->SetRole(ButtonRole::Critical);
  EndActions();
  LiveText(message);
}
}  // namespace opennav::ui

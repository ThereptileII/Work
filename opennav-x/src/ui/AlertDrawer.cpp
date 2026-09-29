#include "ui/AlertDrawer.h"
#include <array>
#include <wx/dcbuffer.h>
#include <wx/tokenzr.h>

namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
const char *InspectLabel(application::AlertArea area) {
  switch (area) {
  case application::AlertArea::Sources:
    return "View source health";
  case application::AlertArea::Ais:
    return "View traffic";
  case application::AlertArea::Anchor:
    return "View anchor watch";
  case application::AlertArea::Energy:
    return "View energy";
  case application::AlertArea::Pilot:
    return "View autopilot";
  }
  return "View source health";
}
bool Same(const application::Alert &a, const application::Alert &b) {
  return a.id == b.id && a.episode == b.episode && a.title == b.title &&
         a.action == b.action && a.source == b.source && a.level == b.level &&
         a.area == b.area && a.acknowledged == b.acknowledged;
}
wxColour CalloutBacking(const Palette &c, application::AlertLevel level) {
  const bool attention = level != application::AlertLevel::Info;
  if (!attention)
    return Colour(c.selected);
  const auto ink = Colour(level == application::AlertLevel::Critical
                              ? c.alarm
                              : prototype_ink::warning);
  const auto base = Colour(c.background);
  const int alpha = prototype_ink::warning_callout_alpha;
  return wxColour(
      (ink.Red() * alpha + base.Red() * (255 - alpha) + 127) / 255,
      (ink.Green() * alpha + base.Green() * (255 - alpha) + 127) / 255,
      (ink.Blue() * alpha + base.Blue() * (255 - alpha) + 127) / 255);
}
} // namespace
XNavAlertDrawer::XNavAlertDrawer(wxWindow &owner, AlertDrawerActions actions)
    : XNavDrawer(owner, "OpenNav alerts"), actions_(std::move(actions)) {
  SetHeading("NOTIFICATION CENTRE", "A watchful eye", false);
}
void XNavAlertDrawer::Update(const std::vector<application::Alert> &alerts,
                             bool replay, LightMode light) {
  const bool changed = !built_ || replay_ != replay || light_ != light ||
                       alerts_.size() != alerts.size() ||
                       !std::equal(alerts_.begin(), alerts_.end(),
                                   alerts.begin(), alerts.end(), Same);
  if (!changed)
    return;
  alerts_ = alerts;
  replay_ = replay;
  built_ = true;
  SetLight(light);
  Rebuild();
}
void XNavAlertDrawer::Rebuild() {
  const int scroll = body_->GetViewStart().y;
  ClearBody();
  const auto visual = [&](int height, const wxString &name, auto paint) {
    auto *panel = new wxPanel(body_, wxID_ANY);
    panel->SetBackgroundColour(Colour(Theme(light_).background));
    panel->SetName(name);
    panel->SetMinSize(FromDIP(wxSize(300, height)));
    panel->SetBackgroundStyle(wxBG_STYLE_PAINT);
    panel->Bind(wxEVT_PAINT, [this, panel, paint](wxPaintEvent &) {
      wxAutoBufferedPaintDC dc(panel);
      XNavPainter p(*panel, dc, light_);
      dc.SetBackground(wxBrush(Colour(p.c.background)));
      dc.Clear();
      paint(p, dc, panel->ToDIP(panel->GetClientSize().x));
    });
    EnableScrollGesture(*panel);
    content_->Add(panel, 0, wxEXPAND);
    return panel;
  };
  if (replay_)
    visual(46, "Historical alerts", [](XNavPainter &p, wxDC &, int w) {
      p.Wrapped("REPLAY / Historical conditions. Return to live navigation to "
                "inspect current sources.",
                0, 0, 12, 20, w, p.c.attention, 2);
    });
  if (alerts_.empty())
    visual(86, "No current alerts", [](XNavPainter &p, wxDC &, int w) {
      p.Wrapped("No current XNav alerts. Continue to monitor the chart, "
                "instruments and surroundings.",
                0, 20, 12, 20, w, p.c.secondary, 3);
    });
  for (const auto &a : alerts_) {
    // The final prototype callout has 15 px padding and ~20 px body lines.
    // Measure actual text, so a real source fault is never hidden under
    // actions.
    const int width = std::max(280, ToDIP(body_->GetClientSize().x) - 44);
    wxClientDC measure(this);
    measure.SetFont(UiFont(*this, 12));
    const auto body =
        W(a.action) +
        (a.acknowledged ? " Acknowledged; condition remains active." : "");
    wxStringTokenizer words(body, " ");
    wxString line;
    int lines = 1;
    while (words.HasMoreTokens()) {
      const auto word = words.GetNextToken(),
                 candidate = line.empty() ? word : line + " " + word;
      if (!line.empty() &&
          measure.GetTextExtent(candidate).x > FromDIP(width - 34)) {
        ++lines;
        line = word;
      } else
        line = candidate;
    }
    const int button_y = 38 + 20 * lines + 20, height = button_y + 48 + 16;
    content_->AddSpacer(FromDIP(20));
    auto *panel = visual(
        height, "Alert " + W(a.title),
        [a, body, lines, height](XNavPainter &p, wxDC &dc, int w) {
          const bool critical = a.level == application::AlertLevel::Critical;
          const bool attention = critical ||
                                 a.level == application::AlertLevel::Warning ||
                                 a.level == application::AlertLevel::Advisory;
          p.Callout(W(application::AlertLevelName(a.level)) +
                        wxString::FromUTF8(" · ") + W(a.title),
                    body, w, height, attention, critical);
        });
    panel->SetBackgroundColour(CalloutBacking(Theme(light_), a.level));
    auto *inspect = new XNavButton(panel, wxID_ANY, InspectLabel(a.area),
                                   "Inspect " + W(a.title));
    inspect->Enable(bool(actions_.inspect) && !replay_);
    auto *ack = new XNavButton(panel, wxID_ANY, "Acknowledge",
                               "Acknowledge " + W(a.title));
    ack->Enable(bool(actions_.acknowledge) && !a.acknowledged && !replay_);
    inspect->SetLightMode(light_);
    ack->SetLightMode(light_);
    const auto arrange = [panel, inspect, ack, button_y] {
      const int w = panel->ToDIP(panel->GetClientSize().x);
      const int first = (w - 34 - 8) / 2;
      inspect->SetSize(wxRect(panel->FromDIP(wxPoint(17, button_y)),
                              panel->FromDIP(wxSize(first, 48))));
      ack->SetSize(wxRect(panel->FromDIP(wxPoint(17 + first + 8, button_y)),
                          panel->FromDIP(wxSize(w - 34 - first - 8, 48))));
    };
    panel->Bind(wxEVT_SIZE, [arrange](wxSizeEvent &e) {
      arrange();
      e.Skip();
    });
    arrange();
    inspect->Bind(wxEVT_BUTTON, [this, area = a.area](wxCommandEvent &) {
      if (queued_ || replay_ || !actions_.inspect)
        return;
      queued_ = true;
      CallAfter([this, area] {
        queued_ = false;
        if (!replay_)
          actions_.inspect(area);
      });
    });
    ack->Bind(wxEVT_BUTTON,
              [this, id = a.id, episode = a.episode](wxCommandEvent &) {
                if (queued_ || replay_ || !actions_.acknowledge)
                  return;
                queued_ = true;
                CallAfter([this, id, episode] {
                  queued_ = false;
                  // Identity and episode are the ones displayed when pressed.
                  // Backend rejects a recovered/replaced episode; never
                  // retarget a delayed tap.
                  if (!replay_)
                    actions_.acknowledge(id, episode);
                });
              });
  }
  content_->AddSpacer(FromDIP(20));
  visual(244, "Alert meanings", [](XNavPainter &p, wxDC &, int w) {
    const std::array<const char *, 4> labels = {"Info", "Advisory", "Warning",
                                                "Critical"};
    const std::array<const char *, 4> notes = {
        "Routine status", "Awareness, no immediate action",
        "Review and act as appropriate", "Persistent until resolved"};
    for (int i = 0; i < 4; ++i) {
      p.Text(labels[i], 0, i * 45 + 13, 12, p.c.secondary, false, 65);
      p.TextWeight(notes[i], 70, i * 45 + 13, 12, p.c.primary, 500, w - 70,
                   true);
      p.Rule(0, (i + 1) * 45 - 1, w);
    }
    p.Wrapped("An acknowledged critical alert stays visible while its "
              "underlying condition remains unresolved.",
              0, 197, 11, 18, w, p.c.muted, 3);
  });
  body_->Layout();
  body_->FitInside();
  body_->Scroll(0, scroll);
}
} // namespace opennav::ui

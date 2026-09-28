#include "ui/AisDrawer.h"
#include "ui/Sheet.h"
#include "vessel/AisSelection.h"
#include <algorithm>
#include <cmath>
#include <wx/dcbuffer.h>
#include <wx/dialog.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
std::optional<double> Current(const vessel::Sample &s, vessel::Time now) {
  const auto a = vessel::Assess(s, now);
  return a.quality == vessel::Quality::Live ||
                 a.quality == vessel::Quality::Aging
             ? a.value
             : std::nullopt;
}
wxString Number(const vessel::Sample &s, vessel::Time now, int places,
                const char *unit = "") {
  const auto value = Current(s, now);
  return value ? wxString::Format("%.*f", places, *value) + W(unit)
               : wxString::FromUTF8("—");
}
wxString Name(const vessel::AisTarget &t) {
  return W(t.name.empty() ? std::to_string(t.mmsi) : t.name);
}
wxString Connection(ais::Connection value) {
  switch (value) {
  case ais::Connection::Disabled:
    return "Off";
  case ais::Connection::CredentialMissing:
    return "Key needed";
  case ais::Connection::Connecting:
  case ais::Connection::Subscribing:
    return "Connecting";
  case ais::Connection::Connected:
    return "Connected";
  case ais::Connection::Backoff:
    return "Reconnecting";
  default:
    return "Offline";
  }
}
wxString Age(const vessel::AisTarget &t, vessel::Time now) {
  if (t.observed_at == vessel::Time{} || now < t.observed_at)
    return "Time unavailable";
  const double seconds =
      std::chrono::duration<double>(now.time_since_epoch()).count() -
      std::chrono::duration<double>(t.observed_at.time_since_epoch()).count();
  return wxString::Format("%.0f s", seconds);
}
} // namespace
XNavAisDrawer::XNavAisDrawer(wxWindow &owner,
                             application::OnlineAisActions actions,
                             std::function<void(int)> show_on_chart)
    : XNavDrawer(owner, "OpenNav vessel traffic"), actions_(std::move(actions)),
      show_on_chart_(std::move(show_on_chart)) {
  on_back = [this] { List(); };
  Build();
}
void XNavAisDrawer::List() {
  view_ = View::List;
  mmsi_ = 0;
  Build();
}
void XNavAisDrawer::Target(int mmsi) {
  mmsi_ = mmsi;
  view_ = View::Target;
  Build();
}
std::optional<vessel::AisTarget> XNavAisDrawer::Selected() const {
  std::optional<vessel::AisTarget> found;
  for (const auto &t : display_.targets)
    if (t.mmsi == mmsi_) {
      if (found)
        return {};
      found = t;
    }
  return found;
}
void XNavAisDrawer::Update(const vessel::AisState &display,
                           const application::OnlineAisState &online,
                           vessel::Time now, LightMode light) {
  display_ = display;
  online_ = online;
  now_ = now;
  if (light != light_) {
    SetLight(light);
    Build();
  }
  RefreshValues();
}
void XNavAisDrawer::AddText(const wxString &text, int size) {
  auto *label = new wxStaticText(body_, wxID_ANY, text);
  label->SetFont(UiFont(*label, size));
  label->SetForegroundColour(Colour(Theme(light_).secondary));
  label->Wrap(FromDIP(352));
  EnableScrollGesture(*label);
  content_->Add(label, 0, wxEXPAND | wxBOTTOM, FromDIP(16));
}
XNavButton *XNavAisDrawer::Button(const wxString &label,
                                  std::function<void()> action, bool enabled) {
  auto *button = new XNavButton(body_, wxID_ANY, label, label);
  button->SetLightMode(light_);
  button->SetMinSize(FromDIP(wxSize(120, 48)));
  button->Enable(enabled);
  button->Bind(wxEVT_BUTTON, [this, action = std::move(action)](
                                 wxCommandEvent &) { CallAfter(action); });
  content_->Add(button, 0, wxEXPAND | wxBOTTOM, FromDIP(12));
  buttons_.push_back(button);
  return button;
}
void XNavAisDrawer::AddVisual(int height,
                              std::function<void(XNavPainter &, int)> draw) {
  auto *panel = new wxPanel(body_, wxID_ANY);
  panel->SetMinSize(FromDIP(wxSize(300, height)));
  panel->SetBackgroundStyle(wxBG_STYLE_PAINT);
  EnableScrollGesture(*panel);
  panel->Bind(wxEVT_PAINT, [this, panel, draw](wxPaintEvent &) {
    wxAutoBufferedPaintDC dc(panel);
    XNavPainter p(*panel, dc, light_);
    dc.SetBackground(wxBrush(Colour(p.c.background)));
    dc.Clear();
    draw(p, panel->ToDIP(panel->GetClientSize().x));
  });
  content_->Add(panel, 0, wxEXPAND | wxBOTTOM, FromDIP(16));
  visuals_.push_back(panel);
}
void XNavAisDrawer::Build() {
  ClearBody();
  list_ = nullptr;
  range_ = cpa_ = show_ = nullptr;
  visuals_.clear();
  buttons_.clear();
  if (view_ == View::List) {
    auto *segment = new wxPanel(body_, wxID_ANY);
    segment->SetBackgroundColour(Colour(Theme(light_).surface));
    segment->SetBackgroundStyle(wxBG_STYLE_PAINT);
    segment->Bind(wxEVT_PAINT, [this, segment](wxPaintEvent &) {
      wxAutoBufferedPaintDC dc(segment);
      const auto c = Theme(light_);
      dc.SetBackground(wxBrush(Colour(c.background))); dc.Clear();
      dc.SetPen(*wxTRANSPARENT_PEN); dc.SetBrush(wxBrush(Colour(c.surface)));
      const auto size = segment->GetClientSize();
      dc.DrawRoundedRectangle(0, 0, size.x, size.y, FromDIP(9));
    });
    auto *row = new wxBoxSizer(wxHORIZONTAL);
    cpa_ = new XNavButton(segment, wxID_ANY, "Closest approach",
                          "Sort AIS by closest approach");
    range_ = new XNavButton(segment, wxID_ANY, "Range", "Sort AIS by range");
    for (auto *b : {cpa_, range_}) {
      b->SetRole(ButtonRole::Segment);
      b->SetLightMode(light_);
      b->SetMinSize(FromDIP(wxSize(140, 40)));
    }
    row->Add(cpa_, 1, wxTOP | wxBOTTOM | wxLEFT, FromDIP(4));
    row->AddSpacer(FromDIP(4));
    row->Add(range_, 1, wxTOP | wxBOTTOM | wxRIGHT, FromDIP(4));
    cpa_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
      sort_range_ = false;
      RefreshValues();
    });
    range_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
      sort_range_ = true;
      RefreshValues();
    });
    segment->SetSizer(row);
    content_->Add(segment, 0, wxEXPAND | wxBOTTOM, FromDIP(20));
    list_ = new XNavListView(body_);
    list_->SetMinSize(FromDIP(wxSize(300, 300)));
    list_->on_select = [this](const std::string &identity) {
      Target(std::stoi(identity));
    };
    content_->Add(list_, 0, wxEXPAND | wxBOTTOM, FromDIP(20));
    AddVisual(60, [this](XNavPainter &p, int width) {
      p.Text("Online AIS", 0, 0, 12, p.c.secondary);
      p.Text(Connection(online_.feed.health.connection), width - 110, 0, 12,
             p.c.primary, false, 110);
      p.Text("Select a vessel for its position, age and source.", 0, 28, 11,
             p.c.secondary, false, width);
    });
    Button("Online AIS settings", [this] {
      view_ = View::Settings;
      Build();
    });
  } else if (view_ == View::Target) {
    AddVisual(40, [this](XNavPainter &p, int width) {
      const auto target = Selected();
      p.Text(target ? W(target->status) : wxString("Target unavailable"), 0, 4,
             11, p.c.secondary, false, width);
      p.Text(target && target->origin == vessel::AisOrigin::AisStreamOnline
                 ? "Internet AIS"
                 : "Onboard AIS",
             0, 22, 10, p.c.accent, false, width);
    });
    AddVisual(122, [this](XNavPainter &p, int width) {
      const auto t = Selected();
      const auto empty = vessel::Sample{};
      const auto stat = [&](const wxString &label, const vessel::Sample &s,
                            int x, int y, int decimals, const char *unit) {
        p.Text(label, x, y, 9, p.c.muted, false, width / 2 - 12);
        p.Text(Number(s, now_, decimals, unit), x, y + 20, 24, p.c.primary,
               false, width / 2 - 12);
      };
      stat("CLOSEST APPROACH", t ? t->cpa_nm : empty, 0, 0, 2, " NM");
      stat("TIME TO CPA", t ? t->tcpa_minutes : empty, width / 2, 0, 0, " min");
      stat("SPEED OVER GROUND", t ? t->sog_kn : empty, 0, 66, 1, " kn");
      stat("COURSE OVER GROUND", t ? t->cog_deg : empty, width / 2, 66, 0, "°");
    });
    AddVisual(76, [this](XNavPainter &p, int width) {
      const auto t = Selected();
      const bool online = t && t->origin == vessel::AisOrigin::AisStreamOnline;
      p.Text(online                   ? "Supplemental internet traffic"
             : t && t->upstream_alarm ? "OpenCPN AIS alert"
                                      : "Onboard AIS report",
             0, 5, 12, online ? p.c.attention : p.c.primary, true, width);
      p.Text(online ? "Reception may be delayed. CPA/TCPA unavailable."
                    : "Maintain a visual watch and review the chart.",
             0, 31, 11, p.c.secondary, false, width);
      p.Text(t && vessel::AisSelection::CurrentPosition(*t, now_)
                 ? "Position current in this source"
                 : "Position stale or unavailable",
             0, 53, 11, p.c.secondary, false, width);
    });
    AddVisual(224, [this](XNavPainter &p, int width) {
      const auto t = Selected();
      const auto empty = vessel::Sample{};
      const auto line = [&](int y, const char *label, const wxString &value) {
        p.Text(W(label), 0, y, 12, p.c.secondary, false, 140);
        p.Text(value, width / 2, y, 11, p.c.primary, true, width / 2);
        p.Rule(0, y + 31, width);
      };
      line(0, "Range / bearing",
           t ? Number(t->range_nm, now_, 1, " NM") + " / " +
                   Number(t->bearing_true_deg, now_, 0, "°")
             : wxString::FromUTF8("—"));
      line(44, "MMSI",
           t ? wxString::Format("%d", t->mmsi) : wxString::FromUTF8("—"));
      line(88, "Length / beam",
           Number(t ? t->length_m : empty, now_, 0, " m") + " / " +
               Number(t ? t->beam_m : empty, now_, 0, " m"));
      line(132, "Source",
           t && t->origin == vessel::AisOrigin::AisStreamOnline
               ? "AISStream online"
               : "OpenCPN onboard AIS");
      line(176, "Position age", t ? Age(*t, now_) : wxString("Unavailable"));
    });
    show_ = Button("Show on chart", [this] {
      const auto t = Selected();
      if (t && vessel::AisSelection::CurrentPosition(*t, now_) &&
          show_on_chart_)
        show_on_chart_(t->mmsi);
    });
    show_->SetRole(ButtonRole::Primary);
  } else {
    AddText("Optional internet traffic for the current chart area. Onboard AIS "
            "remains the navigation source.");
    AddVisual(62, [this](XNavPainter &p, int width) {
      p.Text("Online AIS", 0, 0, 13, p.c.secondary);
      p.Text(Connection(online_.feed.health.connection), width - 120, 0, 13,
             p.c.primary, false, 120);
      p.Text("AISStream key", 0, 30, 12, p.c.secondary);
      p.Text(online_.credential_present ? "Stored securely" : "Not configured",
             width - 120, 30, 11, p.c.primary, false, 120);
    });
    auto update = [this](application::CommandResult result) {
      message_ = W(result.message);
      if (actions_.read)
        online_ = actions_.read(now_);
      Build();
    };
    auto *off = Button(
        "Off",
        [this, update] {
          if (actions_.enable)
            update(actions_.enable(false));
        },
        bool(actions_.enable));
    off->SetSelected(!online_.enabled);
    auto *on = Button(
        "Enabled",
        [this, update] {
          if (actions_.enable)
            update(actions_.enable(true));
        },
        bool(actions_.enable));
    on->SetSelected(online_.enabled);
    if (online_.credential_writable) {
      Button(
          "Set AISStream key", [this] { StoreKey(); },
          bool(actions_.store_key));
      Button(
          "Remove key",
          [this, update] {
            if (actions_.remove_key &&
                ConfirmSheet(*this, light_, "Remove AISStream key",
                             "Online AIS will stop. The saved key will be "
                             "removed from this Windows account.",
                             "Remove key"))
              update(actions_.remove_key());
          },
          online_.credential_present && bool(actions_.remove_key));
    } else
      AddText("Development key: AISSTREAM_API_KEY environment variable.");
    if (!message_.empty())
      AddText(message_);
  }
  RefreshValues();
  body_->FitInside();
  body_->Layout();
}
void XNavAisDrawer::RefreshValues() {
  if (view_ == View::List) {
    SetHeading(wxString::Format("AIS · %zu TARGETS", display_.targets.size()),
               "Vessel traffic", false);
    auto targets = display_.targets;
    if (targets.size() > 2000)
      targets.clear();
    bool has_cpa = false, has_range = false;
    for (const auto &t : targets) {
      has_cpa |= Current(t.cpa_nm, now_).has_value();
      has_range |= Current(t.range_nm, now_).has_value();
    }
    if (cpa_) {
      cpa_->Enable(has_cpa);
      cpa_->SetSelected(!sort_range_ && has_cpa);
    }
    if (range_) {
      range_->Enable(has_range);
      range_->SetSelected(sort_range_ && has_range);
    }
    std::sort(
        targets.begin(), targets.end(), [this](const auto &a, const auto &b) {
          const auto av = Current(sort_range_ ? a.range_nm : a.cpa_nm, now_),
                     bv = Current(sort_range_ ? b.range_nm : b.cpa_nm, now_);
          if (av && bv && *av != *bv)
            return *av < *bv;
          if (bool(av) != bool(bv))
            return bool(av);
          return a.mmsi < b.mmsi;
        });
    std::vector<XNavListRowData> rows;
    for (const auto &t : targets) {
      rows.push_back(
          {std::to_string(t.mmsi), Name(t),
           Number(t.sog_kn, now_, 1, " kn") + " · " +
               (t.origin == vessel::AisOrigin::AisStreamOnline ? "Internet AIS"
                                                               : "Onboard AIS"),
           Number(sort_range_ ? t.range_nm : t.cpa_nm, now_, 2, " NM"),
           sort_range_ ? "Range" : "CPA", t.upstream_alarm,
           !vessel::AisSelection::CurrentPosition(t, now_)});
    }
    if (list_)
      list_->Update(std::move(rows), light_);
  } else if (view_ == View::Target) {
    const auto t = Selected();
    SetHeading("AIS TARGET", t ? Name(*t) : "Target unavailable", true);
    if (show_)
      show_->Enable(t && vessel::AisSelection::CurrentPosition(*t, now_));
  } else
    SetHeading("AIS", "Online AIS", true);
  for (auto *p : visuals_)
    p->Refresh(false);
}
void XNavAisDrawer::StoreKey() {
  // A masked entry never participates in ordinary settings serialization,
  // diagnostic text or field export. Only the bounded Secret crosses to
  // storage.
  wxDialog prompt(this, wxID_ANY, "AISStream key", wxDefaultPosition,
                  FromDIP(wxSize(430, 210)), wxBORDER_NONE);
  prompt.SetBackgroundColour(Colour(Theme(light_).background));
  auto *layout = new wxBoxSizer(wxVERTICAL);
  auto *title = new wxStaticText(&prompt, wxID_ANY, "AISStream key");
  title->SetFont(UiFont(prompt, 22));
  title->SetForegroundColour(Colour(Theme(light_).primary));
  layout->Add(title, 0, wxALL, FromDIP(20));
  auto *entry = new wxTextCtrl(&prompt, wxID_ANY, "", wxDefaultPosition,
                               wxDefaultSize, wxTE_PASSWORD | wxBORDER_NONE);
  entry->SetName("Protected AISStream key");
  entry->SetMaxLength(512);
  entry->SetFont(UiFont(prompt, 18));
  entry->SetBackgroundColour(Colour(Theme(light_).surface));
  entry->SetForegroundColour(Colour(Theme(light_).primary));
  layout->Add(entry, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(20));
  auto *row = new wxBoxSizer(wxHORIZONTAL);
  for (auto choice : {std::pair<wxString, int>{"Cancel", wxID_CANCEL},
                      {"Save key", wxID_OK}}) {
    auto *button =
        new XNavButton(&prompt, wxID_ANY, choice.first, choice.first);
    button->SetLightMode(light_);
    button->Bind(wxEVT_BUTTON, [&prompt, id = choice.second](wxCommandEvent &) {
      prompt.EndModal(id);
    });
    row->Add(button, 1, wxALL, FromDIP(4));
  }
  layout->Add(row, 0, wxEXPAND | wxALL, FromDIP(16));
  prompt.SetSizerAndFit(layout);
  prompt.CentreOnParent();
  prompt.Bind(wxEVT_CHAR_HOOK, [&prompt](wxKeyEvent &e) {
    if (e.GetKeyCode() == WXK_ESCAPE)
      prompt.EndModal(wxID_CANCEL);
    else
      e.Skip();
  });
  entry->SetFocus();
  if (prompt.ShowModal() == wxID_OK && actions_.store_key) {
    wxString entered = entry->GetValue();
    wxCharBuffer encoded(entered.utf8_str().data());
    ais::Secret key;
    const bool valid =
        key.Assign(std::string_view(encoded.data(), encoded.length()));
    volatile char *bytes = encoded.data();
    for (std::size_t i = 0; i < encoded.length(); ++i)
      bytes[i] = 0;
    for (std::size_t i = 0; i < entered.length(); ++i)
      entered[i] = wxUniChar(0);
    entry->ChangeValue("");
    message_ =
        valid ? W(actions_.store_key(key).message) : "Key is empty or invalid";
  }
  entry->ChangeValue("");
  if (actions_.read)
    online_ = actions_.read(now_);
  Build();
}
} // namespace opennav::ui

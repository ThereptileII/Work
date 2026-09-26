#include "ui/ProductPanel.h"
#include "ui/Sheet.h"
#include "integration/BuildFeatures.h"
#include <wx/dcbuffer.h>
#include <wx/textctrl.h>
#include "vessel/DataItems.h"
#include "vessel/DisplayItems.h"
#include <algorithm>
#include <cmath>
#include <wx/wrapsizer.h>
namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
wxString Name(const std::string &name, const std::string &id) {
  (void)id;
  return W(name.empty() ? "Unnamed" : name);
}
const vessel::AisTarget *Target(const ProductState &s, int mmsi) {
  for (const auto &t : s.ais.targets)
    if (t.mmsi == mmsi)
      return &t;
  return nullptr;
}
void Metric(XNavPainter &p, const vessel::Sample &sample, vessel::Time now,
            int x, int y, int width, const wxString &title, const wxString &unit,
            int decimals = 1, int size = 36) {
  const auto a = vessel::Assess(sample, now);
  const bool stale = a.quality == vessel::Quality::Stale;
  p.Text(title, x, y, 11, p.c.secondary, false, width);
  p.Text(a.value && !stale ? wxString::Format("%.*f", decimals, *a.value) : wxString::FromUTF8("—"),
         x, y + 24, size, stale ? p.c.muted : p.c.primary, false, width);
  p.Text(unit, x, y + size + 30, 12, p.c.secondary, false, width);
  if (a.quality != vessel::Quality::Live)
    p.Text(a.quality == vessel::Quality::Unavailable ? "NO DATA" : W(vessel::QualityName(a.quality)),
           x, y + size + 50, 11, stale ? p.c.attention : p.c.muted, false, width);
}
} // namespace
ProductPanel::ProductPanel(wxWindow *parent, ProductActions actions)
    : XNavScroll(parent),
      actions_(std::move(actions)) {
  SetScrollRate(0, FromDIP(24));
  Bind(wxEVT_CHAR_HOOK, [this](wxKeyEvent &event) {
    auto *focus = wxWindow::FindFocus();
    if (event.GetKeyCode() == WXK_ESCAPE &&
        (!focus || (!dynamic_cast<wxTextCtrl *>(focus) &&
                     wxGetTopLevelParent(focus) == wxGetTopLevelParent(this)))) {
      CallAfter([this] { Back(); });
      return;
    }
    event.Skip();
  });
  Bind(wxEVT_SIZE, [this](wxSizeEvent &e) {
    const int width = GetClientSize().x;
    if (width > 0 && width != layout_width_) {
      layout_width_ = width;
      for (auto &t : static_text_) {
        t.first->SetLabel(t.second);
        t.first->Wrap(std::max(200, width - FromDIP(64)));
      }
      for (auto &g : action_grids_)
        g.sizer->SetCols(std::max(1, std::min(g.columns, width / FromDIP(g.minimum_width + 12))));
      Layout();
      FitInside();
    }
    if (grid_) {
      const int cols = GetClientSize().x >= FromDIP(920)   ? 4
                       : GetClientSize().x >= FromDIP(600) ? 3
                                                           : 2;
      if (grid_->GetCols() != cols) {
        grid_->SetCols(cols);
        Layout();
        FitInside();
      }
    }
    e.Skip();
  });
}
void ProductPanel::Back() {
  ProductPage parent = ProductPage::Home;
  switch (page_) {
  case ProductPage::Home: if (actions_.chart) actions_.chart(); return;
  case ProductPage::RouteDetail: parent = ProductPage::Routes; break;
  case ProductPage::WaypointDetail: parent = ProductPage::Waypoints; break;
  case ProductPage::AisDetail: parent = ProductPage::Ais; break;
  case ProductPage::SourceDetail: case ProductPage::BoatMapping: case ProductPage::SourcesAdvanced: parent = ProductPage::Sources; break;
  case ProductPage::PilotSettings: parent = ProductPage::Pilot; break;
  case ProductPage::RailLayout: case ProductPage::InstrumentLayout: parent = ProductPage::Display; break;
  case ProductPage::EnergySettings: case ProductPage::VesselSettings:
  case ProductPage::NavigationSettings: case ProductPage::Sources:
  case ProductPage::Display: case ProductPage::Radar: parent = ProductPage::Settings; break;
  case ProductPage::Commissioning: case ProductPage::FieldReport: parent = ProductPage::System; break;
  default: break;
  }
  ShowPage(parent, mode_);
}
void ProductPanel::Heading(const wxString &title, const wxString &subtitle) {
  if (first_heading_) {
    first_heading_ = false;
    auto *row = new wxBoxSizer(wxHORIZONTAL);
    auto *back = new XNavIconButton(this, wxID_ANY, XNavIcon::Back, "Back", "Back");
    back->SetMinSize(FromDIP(wxSize(64, 48)));
    back->SetLightMode(mode_);
    back->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { CallAfter([this] { Back(); }); });
    row->Add(back, 0, wxRIGHT, FromDIP(16));
    auto *label = new wxStaticText(this, wxID_ANY, title);
    label->SetFont(UiFont(*this, 28));
    label->SetForegroundColour(Colour(Theme(mode_).primary));
    EnableScrollGesture(*label);
    row->Add(label, 1, wxALIGN_CENTER_VERTICAL);
    body_->Add(row, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(16));
    if (!subtitle.empty()) Text(subtitle, 13);
  } else {
    Text(title, 22);
    if (!subtitle.empty()) Text(subtitle, 13);
  }
}
void ProductPanel::Visual(const wxString &name, int height,
                          std::function<void(XNavPainter &, wxDC &, int)> draw) {
  auto *panel = new wxPanel(this);
  panel->SetName(name);
  panel->SetMinSize(FromDIP(wxSize(280, height)));
  panel->SetBackgroundStyle(wxBG_STYLE_PAINT);
  EnableScrollGesture(*panel);
  panel->Bind(wxEVT_PAINT, [this, panel, draw](wxPaintEvent &) {
    wxAutoBufferedPaintDC dc(panel);
    dc.SetBackground(wxBrush(Colour(Theme(mode_).background)));
    dc.Clear();
    XNavPainter painter(*panel, dc, mode_);
    draw(painter, dc, panel->ToDIP(panel->GetClientSize().x));
  });
  body_->Add(panel, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(16));
  visuals_.push_back(panel);
}
void ProductPanel::Text(const wxString &text, int size) {
  auto *label = new wxStaticText(this, wxID_ANY, text);
  EnableScrollGesture(*label);
  label->SetFont(UiFont(*this, size));
  label->SetForegroundColour(
      Colour(size > 18 ? Theme(mode_).primary : Theme(mode_).secondary));
  static_text_.push_back({label, text});
  label->Wrap(std::max(200, GetClientSize().x - FromDIP(64)));
  body_->Add(label, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(16));
}
void ProductPanel::LiveText(
    std::function<wxString(const ProductState &)> text) {
  auto *label = new wxStaticText(this, wxID_ANY, text(state_));
  EnableScrollGesture(*label);
  label->SetFont(UiFont(*this, 14));
  label->Wrap(std::max(200, GetClientSize().x - FromDIP(64)));
  label->SetForegroundColour(Colour(Theme(mode_).secondary));
  body_->Add(label, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(16));
  text_.push_back({label, std::move(text)});
}
XNavButton *ProductPanel::StatusAction(const wxString &title,
    std::function<wxString(const ProductState &)> status, std::function<void()> action) {
  auto text = [title, status](const ProductState &state) { return title + "  ·  " + status(state); };
  auto *button = Action(text(state_), std::move(action));
  button->SetRole(ButtonRole::Quiet);
  button->SetMinSize(FromDIP(wxSize(200, 64)));
  button_text_.push_back({button, std::move(text)});
  return button;
}
XNavButton *ProductPanel::Action(const wxString &label,
                                 std::function<void()> action, bool enabled) {
  auto *button = new XNavButton(this, wxID_ANY, label, label);
  button->SetMinSize(FromDIP(wxSize(actions_grid_ ? action_width_ : 200, 52)));
  button->SetLightMode(mode_);
  button->Enable(enabled && static_cast<bool>(action));
  button->Bind(wxEVT_BUTTON,
               [this, action = std::move(action)](wxCommandEvent &) {
                 CallAfter([action] {
                   if (action)
                     action();
                 });
               });
  if (actions_grid_)
    actions_grid_->Add(button, 1, wxEXPAND);
  else
    body_->Add(button, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(8));
  return button;
}
void ProductPanel::Value(
    const wxString &title, const wxString &unit,
    std::function<vessel::Sample(const ProductState &)> value, int decimals) {
  if (!grid_) {
    grid_ = new wxGridSizer(3, FromDIP(12), FromDIP(12));
    body_->Add(grid_, 0, wxEXPAND | wxALL, FromDIP(16));
  }
  auto *v = new XNavDataValue(this, title, unit, decimals);
  v->SetLightMode(mode_);
  v->SetMinSize(FromDIP(wxSize(180, 126)));
  v->SetReading(value(state_), state_.now);
  grid_->Add(v, 1, wxEXPAND);
  values_.push_back({v, std::move(value)});
}
void ProductPanel::BeginActions(int columns, int minimum_width) {
  action_width_ = minimum_width;
  actions_grid_ = new wxGridSizer(
      std::max(1, std::min(columns, GetClientSize().x / FromDIP(minimum_width + 12))),
      FromDIP(8), FromDIP(8));
  action_grids_.push_back({actions_grid_, columns, minimum_width});
  body_->Add(actions_grid_, 0, wxEXPAND | wxALL, FromDIP(16));
}
void ProductPanel::Result(application::CommandResult result) {
  if (result.ok && !result.identity.empty()) {
    if (page_ == ProductPage::RouteDetail)
      ShowObject(result.identity, true, mode_);
    else if (page_ == ProductPage::WaypointDetail ||
             page_ == ProductPage::Waypoints)
      ShowObject(result.identity, false, mode_);
  }
  notice_->Show();
  notice_->SetLabel(W(result.message));
  notice_->SetForegroundColour(
      Colour(result.ok ? Theme(mode_).healthy : Theme(mode_).attention));
  notice_->Wrap(std::max(200, GetClientSize().x - FromDIP(48)));
  Layout();
  FitInside();
}
std::string ProductPanel::PageTitle() const {
  switch (page_) {
  case ProductPage::Commissioning:
    return "Commissioning & recordings";
  case ProductPage::Alerts:
    return "Alerts";
  case ProductPage::System:
    return "System";
  case ProductPage::NavigationSettings: return "Navigation settings";
  case ProductPage::BoatMapping: return "Motor & battery setup";
  case ProductPage::SourcesAdvanced: return "Advanced source details";
  case ProductPage::FieldReport:
    return "Field diagnostic bundle";
  case ProductPage::Home:
    return "Menu";
  case ProductPage::Routes:
    return "Routes";
  case ProductPage::Waypoints:
    return "Waypoints";
  case ProductPage::RouteDetail:
    return "Route detail";
  case ProductPage::WaypointDetail:
    return "Waypoint detail";
  case ProductPage::Ais:
    return "AIS targets";
  case ProductPage::AisDetail:
    return "AIS target";
  case ProductPage::Instruments:
    return "Vessel instruments";
  case ProductPage::Advice:
    return "SmartNav";
  case ProductPage::Pilot:
    return "Manual autopilot";
  case ProductPage::PilotSettings:
    return "Autopilot configuration";
  case ProductPage::Anchor:
    return "Anchor watch";
  case ProductPage::Display:
    return "Display";
  case ProductPage::RailLayout:
    return "Data rail layout";
  case ProductPage::InstrumentLayout:
    return "Instrument layout";
  case ProductPage::Settings:
    return "Settings";
  case ProductPage::EnergySettings:
    return "Energy configuration";
  case ProductPage::Sources:
    return "Data Sources";
  case ProductPage::SourceDetail:
    return "Source selection";
  case ProductPage::VesselSettings:
    return "Vessel safety settings";
  case ProductPage::Radar:
    return "Radar status";
  }
  return "Unknown";
}
void ProductPanel::ShowPage(ProductPage page, LightMode mode) {
  if (page != page_) pilot_advanced_ = false;
  page_ = page;
  mode_ = mode;
  Scroll(0, 0);
  Build();
  if (IsShownOnScreen()) SetFocus();
}
int ProductPanel::MinimumValueHeight() const {
  int minimum = 0;
  for (const auto &v : values_) {
    const int height = ToDIP(v.first->GetClientSize().y);
    if (!minimum || height < minimum) minimum = height;
  }
  for (auto *panel : visuals_) {
    const int height = ToDIP(panel->GetClientSize().y) - 48;
    if (height > 0 && (!minimum || height < minimum)) minimum = height;
  }
  return minimum;
}
std::vector<ProductGeometry> ProductPanel::ControlGeometry() const {
  std::vector<ProductGeometry> out;
  for (auto *child : GetChildren()) {
    if (!dynamic_cast<XNavButton *>(child)) continue;
    const auto rectangle = child->GetScreenRect();
    out.push_back({child->GetLabel().ToStdString(wxConvUTF8), rectangle,
                   child->IsEnabled(), IsShownOnScreen() && child->IsShownOnScreen() &&
                   GetScreenRect().Contains(rectangle)});
  }
  return out;
}
std::vector<ProductGeometry> ProductPanel::RegionGeometry() const {
  std::vector<ProductGeometry> out;
  for (auto *panel : visuals_) {
    const auto rectangle = panel->GetScreenRect();
    out.push_back({panel->GetName().ToStdString(wxConvUTF8), rectangle, true,
                   IsShownOnScreen() && panel->IsShownOnScreen() &&
                   GetScreenRect().Contains(rectangle)});
  }
  return out;
}
void ProductPanel::ShowAis(int mmsi, LightMode mode) {
  mmsi_ = mmsi;
  ShowPage(ProductPage::AisDetail, mode);
}
void ProductPanel::ShowObject(const std::string &id, bool route,
                              LightMode mode) {
  if (actions_.navigation.catalog) {
    const auto catalog = actions_.navigation.catalog();
    if (route) {
      for (const auto &r : catalog.routes)
        if (r.id == id) {
          route_ = r;
          ShowPage(ProductPage::RouteDetail, mode);
          return;
        }
    } else {
      for (const auto &p : catalog.waypoints)
        if (p.id == id) {
          point_ = p;
          ShowPage(ProductPage::WaypointDetail, mode);
          return;
        }
    }
  }
  ShowPage(route ? ProductPage::Routes : ProductPage::Waypoints, mode);
  Result({false, "Selected object no longer available", {}});
}
void ProductPanel::Update(const ProductState &state, LightMode mode) {
  const bool mode_changed = state_.vessel.replayed != state.vessel.replayed ||
                            state_.vessel.simulated != state.vessel.simulated;
  bool alerts_changed = state_.alerts.size() != state.alerts.size();
  if (!alerts_changed)
    for (std::size_t i = 0; i < state.alerts.size(); ++i)
      alerts_changed |= state_.alerts[i].episode != state.alerts[i].episode ||
                        state_.alerts[i].acknowledged != state.alerts[i].acknowledged;
  state_ = state;
  if (mode != mode_ || mode_changed || (page_ == ProductPage::Alerts && alerts_changed)) {
    auto *focus=wxWindow::FindFocus();
    const bool restore_focus=focus && (focus==this || IsDescendant(focus));
    mode_ = mode;
    Build();
    if((restore_focus || mode_changed) && IsShownOnScreen())SetFocus();
  }
  for (auto &button : button_text_) button.first->SetLabel(button.second(state));
  for (auto *visual : visuals_) visual->Refresh(false);
  for (auto &v : values_)
    v.first->SetReading(v.second(state), state.now);
  const auto &caps = state.pilot.capabilities;
  for (auto &button : pilot_buttons_) {
    const auto action = button.second;
    const bool supported =
        action == adapters::PilotAction::Standby ? caps.standby
        : action == adapters::PilotAction::Auto ? caps.auto_mode
        : action == adapters::PilotAction::Track ? caps.track
        : action == adapters::PilotAction::Wind ? caps.wind
                                               : caps.alter_course;
    const bool command_ready = state.pilot.enabled &&
        (action == adapters::PilotAction::Standby || state.pilot.fresh) &&
        (action != adapters::PilotAction::AlterCourse ||
         state.pilot.feedback.mode == adapters::PilotMode::Auto);
    button.first->Enable(supported && command_ready && !state.vessel.replayed);
  }
  bool changed = false;
  for (auto &t : text_) {
    auto value = t.second(state);
    if (t.first->GetLabel() != value) {
      t.first->SetLabel(value);
      t.first->Wrap(std::max(200, GetClientSize().x - FromDIP(64)));
      changed = true;
    }
  }
  if (changed) {
    Layout();
    FitInside();
  }
}
void ProductPanel::AlertsPanel() {
  Heading("Alerts", state_.vessel.replayed ? "REPLAY / Historical conditions" : "Conditions needing your attention");
  if (state_.alerts.empty()) Text("No current XNav alerts. Continue to monitor the chart, instruments and surroundings.");
  for (const auto &a : state_.alerts) {
    Visual("Alert " + W(a.title), 160, [this, id = a.id](XNavPainter &p, wxDC &dc, int width) {
      for (const auto &alert : state_.alerts) if (alert.id == id) {
        const auto color = alert.level == application::AlertLevel::Critical ? p.c.alarm
                           : alert.level == application::AlertLevel::Warning ? p.c.attention
                                                                            : p.c.accent;
        p.Card(0, 0, width, 156, W(application::AlertLevelName(alert.level)));
        dc.SetPen(*wxTRANSPARENT_PEN); dc.SetBrush(wxBrush(Colour(color)));
        dc.DrawRoundedRectangle(p.D(0), p.D(12), p.D(4), p.D(132), p.D(2));
        p.Text(W(alert.title), 24, 46, 23, p.c.primary, false, width - 48);
        p.Text(W(alert.action), 24, 84, 14, p.c.secondary, false, width - 48);
        p.Text(alert.acknowledged ? "Acknowledged / condition remains active" : "Needs attention",
               24, 122, 11, color, false, width - 48);
      }
    });
    BeginActions(2);
    Action("Inspect condition", [this, area = a.area] {
      switch (area) {
      case application::AlertArea::Sources: ShowPage(ProductPage::Sources, mode_); break;
      case application::AlertArea::Ais: ShowPage(ProductPage::Ais, mode_); break;
      case application::AlertArea::Anchor: ShowPage(ProductPage::Anchor, mode_); break;
      case application::AlertArea::Pilot: ShowPage(ProductPage::Pilot, mode_); break;
      case application::AlertArea::Energy: if(actions_.energy) actions_.energy(); break;
      }
    });
    Action("Acknowledge " + W(a.id), [this, id = a.id, episode = a.episode] {
      if (actions_.acknowledge_alert) actions_.acknowledge_alert(id, episode);
    }, !a.acknowledged);
    EndActions();
  }
  Text("Acknowledgement does not clear an active condition or acknowledge alarms on other equipment.", 12);
}
void ProductPanel::CreateMark() {
  if (!actions_.navigation.chart_position)
    return;
  auto location = actions_.navigation.chart_position();
  if (!location) {
    Result({false, "Chart position unavailable", {}});
    return;
  }
  auto fields = EditSheet(
      *this, mode_, "Create waypoint",
      wxString::Format("At chart center %.6f, %.6f. Saves a real OpenCPN mark.",
                       location->latitude_deg, location->longitude_deg),
      {{"Name", "New waypoint", 128}, {"Description", "", 2048}},
      "Create waypoint");
  if (fields && actions_.navigation.create_waypoint)
    Result(actions_.navigation.create_waypoint(*location, (*fields)[0],
                                               (*fields)[1]));
}
void ProductPanel::RouteActions() {
  Heading(Name(route_.name, route_.id),
          route_.active
              ? "Active passage"
              : "Saved route");
  BeginActions(2);
  Action("Back to routes", [this] { ShowPage(ProductPage::Routes, mode_); });
  Action("View first point on chart", [this] {
    if (actions_.chart)
      actions_.chart();
    if (actions_.navigation.view_route)
      actions_.navigation.view_route(route_.id);
  });
  Action(
      route_.active ? "Stop navigation" : "Activate route",
      [this] {
        if (ConfirmSheet(
                *this, mode_,
                route_.active ? "Stop navigation" : "Activate route",
                "This changes OpenCPN navigation. Existing configured OpenCPN "
                "output connections retain their normal behavior.",
                route_.active ? "Stop navigation" : "Activate"))
          Result(route_.active ? actions_.navigation.deactivate()
                               : actions_.navigation.activate(route_));
      },
      !state_.vessel.simulated && !state_.vessel.replayed &&
          (route_.active || route_.editable));
  Action(
      "Edit route name / description",
      [this] {
        auto f = EditSheet(*this, mode_, "Edit route",
                           "Changes use the existing OpenCPN database.",
                           {{"Name", W(route_.name), 128},
                            {"Description", W(route_.description), 2048}});
        if (f)
          Result(actions_.navigation.edit_route(route_, (*f)[0], (*f)[1]));
      },
      route_.editable);
  Action(
      "Edit route points on chart",
      [this] {
        if (ConfirmSheet(
                *this, mode_, "Edit route geometry",
                "Drag the route's points on the chart using OpenCPN's "
                "normal editing behavior. Shared points can affect more "
                "than one route. Inspect the planned legs before activation.",
                "Open chart")) {
          if (actions_.chart)
            actions_.chart();
          if (actions_.navigation.view_route)
            actions_.navigation.view_route(route_.id);
        }
      },
      route_.editable);
  Action(
      "Reverse route",
      [this] {
        if (ConfirmSheet(*this, mode_, "Reverse route",
                         "Reverse planned leg order; waypoint names are "
                         "retained. Navigation must be inactive.",
                         "Reverse"))
          Result(actions_.navigation.reverse(route_));
      },
      route_.editable);
  EndActions();
  Text("PLANNED LEGS", 18);
  for (std::size_t i = 0; i < route_.points.size(); ++i) {
    const auto &p = route_.points[i];
    Text(
        wxString::Format("%u  ", static_cast<unsigned>(i + 1)) +
        Name(p.name, p.id) +
        (p.incoming_nm ? wxString::Format("   %.2f NM", *p.incoming_nm)
                       : "   Start") +
        (p.incoming_course_true_deg
             ? wxString::Format(W("   %.0f° true"), *p.incoming_course_true_deg)
             : ""));
  }
}
void ProductPanel::PointActions() {
  Heading(Name(point_.name, point_.id),
          wxString::Format("%.5f, %.5f", point_.latitude_deg,
                           point_.longitude_deg));
  Text(W(point_.description));
  BeginActions(2);
  Action("Back to waypoints",
         [this] { ShowPage(ProductPage::Waypoints, mode_); });
  Action("View on chart", [this] {
    if (actions_.chart)
      actions_.chart();
    if (actions_.navigation.view_waypoint)
      actions_.navigation.view_waypoint(point_.id);
  });
  Action("GO TO", [this] {
    if (!actions_.navigation.go_to_waypoint) return;
    if (ConfirmSheet(*this, mode_, "Go to " + Name(point_.name, point_.id),
          "Start a passage to this waypoint? Check the chart and passage before starting.", "START")) {
      const auto result = actions_.navigation.go_to_waypoint(point_);
      if (result.ok && actions_.chart) actions_.chart();
      else Result(result);
    }
  }, !state_.vessel.simulated && !state_.vessel.replayed)->SetRole(ButtonRole::Primary);
  Action(
      "Edit waypoint",
      [this] {
        auto f = EditSheet(*this, mode_, "Edit waypoint",
                           "Active-route, anchor-watch and protected points "
                           "are read-only here.",
                           {{"Name", W(point_.name), 128},
                            {"Description", W(point_.description), 2048}});
        if (f)
          Result(actions_.navigation.edit_waypoint(point_, (*f)[0], (*f)[1]));
      },
      point_.editable);
  Action(
      "Delete waypoint",
      [this] {
        if (ConfirmSheet(*this, mode_, "Delete waypoint",
                         Name(point_.name, point_.id) +
                             " will be removed from the shared OpenCPN "
                             "database. This cannot be undone from XNav.",
                         "Delete waypoint"))
          Result(actions_.navigation.delete_waypoint(point_));
      },
      point_.removable);
}
void ProductPanel::Instruments() {
  Heading("Vessel instruments", state_.vessel.replayed
      ? "REPLAY / Historical vessel readings" : "Navigation, wind and conditions");
  const auto config = actions_.settings ? actions_.settings() : state_.settings;
  const std::vector<std::pair<wxString, std::vector<std::string>>> groups{
      {"NAVIGATION", {"sog", "cog", "heading", "stw"}},
      {"WIND", {"aws", "awa", "tws", "twa"}},
      {"CONDITIONS", {"depth", "water_temp", "pressure", "rudder", "heel"}},
      {"ENERGY", {"soc", "voltage", "current", "pack_power", "motor_power", "rpm", "motor_temp"}},
      {"TANKS", {"fresh_water", "fuel", "waste"}}};
  for (const auto &group : groups) {
    std::vector<std::string> chosen;
    for (const auto &key : group.second)
      if (std::find(config.instruments.begin(), config.instruments.end(), key) != config.instruments.end())
        chosen.push_back(key);
    if (chosen.empty()) continue;
    // Each numeric region has a full readable 132 DIP below the common heading.
    // More than four configured values in a family occupy a second grouped row.
    for (std::size_t start = 0; start < chosen.size(); start += 4) {
      const std::vector<std::string> row(chosen.begin() + start,
          chosen.begin() + std::min(chosen.size(), start + 4));
      Visual("Instruments " + group.first, 188,
          [this, row, title = group.first](XNavPainter &p, wxDC &dc, int width) {
        p.Card(0, 0, width, 184, title);
        const int cell = (width - 48) / static_cast<int>(row.size());
        const auto items = vessel::DisplayItems(state_.vessel);
        for (std::size_t i = 0; i < row.size(); ++i)
          for (const auto &item : items)
            if (row[i] == item.key) {
              const int decimals = row[i] == "cog" || row[i] == "heading" || row[i] == "awa" || row[i] == "twa" ? 0 : 1;
              Metric(p, *item.sample, state_.now, 24 + static_cast<int>(i) * cell,
                     52, cell - 16, W(item.title), W(item.unit), decimals);
            }
        if (title == "WIND") {
          const auto angle = vessel::Assess(state_.vessel.wind.apparent_angle_deg, state_.now);
          if (angle.value && (angle.quality == vessel::Quality::Live || angle.quality == vessel::Quality::Aging)) {
            const double radians = *angle.value * 3.14159265358979323846 / 180.0;
            const int cx = width - 42, cy = 26;
            const int dx = static_cast<int>(std::sin(radians) * 14);
            const int dy = static_cast<int>(-std::cos(radians) * 14);
            dc.SetPen(wxPen(Colour(p.c.accent), p.D(2)));
            dc.DrawLine(p.D(cx - dx), p.D(cy - dy), p.D(cx + dx), p.D(cy + dy));
            dc.SetBrush(wxBrush(Colour(p.c.accent)));
            dc.DrawCircle(p.D(cx + dx), p.D(cy + dy), p.D(3));
          }
        }
      });
    }
  }
  Action("Configure instruments", [this] { ShowPage(ProductPage::InstrumentLayout, mode_); });
}
void ProductPanel::PilotActions() {
  Heading("Manual autopilot", state_.vessel.replayed
              ? "REPLAY / All hardware controls disabled" : "Human control / Feedback confirmed");
  Visual("Autopilot heading", 192, [this](XNavPainter &p, wxDC &, int width) {
    p.Card(0, 0, width, 188, "AUTOPILOT");
    const auto &pilot = state_.pilot;
    p.Text(pilot.fresh ? W(adapters::PilotModeName(pilot.feedback.mode)) : wxString("STATUS UNAVAILABLE"),
           width / 2, 20, 15, pilot.fresh ? p.c.healthy : p.c.attention, false, width / 2 - 24);
    auto locked = pilot.fresh ? pilot.feedback.locked_heading_magnetic_deg : vessel::Sample{};
    auto heading = pilot.fresh ? pilot.feedback.heading_magnetic_deg : vessel::Sample{};
    const int cell = (width - 48) / 3;
    Metric(p, locked, state_.now, 24, 54, cell - 16, "COMMANDED HEADING", "° MAGNETIC", 0, 48);
    Metric(p, heading, state_.now, 24 + cell, 54, cell - 16, "ACTUAL HEADING", "° MAGNETIC", 0, 32);
    Metric(p, state_.vessel.rudder.angle_deg, state_.now, 24 + cell * 2, 54,
           cell - 16, "RUDDER", "°", 1, 32);
  });
  BeginActions(4, 96);
  for (int delta : {-10, -1, 1, 10}) {
    auto *button = Action(wxString::Format("%+d°", delta), [this, delta] {
      if (actions_.pilot_command)
        actions_.pilot_command(adapters::PilotAction::AlterCourse, delta);
    }, state_.pilot.enabled && state_.pilot.fresh &&
       state_.pilot.capabilities.alter_course && state_.pilot.feedback.mode == adapters::PilotMode::Auto);
    button->SetName(wxString::Format("%+d° magnetic course", delta));
    button->SetMinSize(FromDIP(wxSize(96, 56)));
    pilot_buttons_.push_back({button, adapters::PilotAction::AlterCourse});
  }
  EndActions();
  BeginActions(2, 144);
  auto *standby = Action("STANDBY", [this] {
    if (actions_.pilot_command) actions_.pilot_command(adapters::PilotAction::Standby, 0);
  }, state_.pilot.enabled && state_.pilot.capabilities.standby);
  standby->SetRole(ButtonRole::Critical);
  standby->SetMinSize(FromDIP(wxSize(144, 56)));
  pilot_buttons_.push_back({standby, adapters::PilotAction::Standby});
  for (const auto &choice : std::vector<std::pair<adapters::PilotAction, wxString>>{
           {adapters::PilotAction::Auto, "AUTO"},
           {adapters::PilotAction::Track, "TRACK"},
           {adapters::PilotAction::Wind, "WIND"}}) {
    const auto &caps = state_.pilot.capabilities;
    const bool supported = choice.first == adapters::PilotAction::Auto ? caps.auto_mode
                         : choice.first == adapters::PilotAction::Track ? caps.track : caps.wind;
    auto *button = Action(choice.second, [this, choice] {
      if (ConfirmSheet(*this, mode_, "Request " + choice.second,
            "The mode changes only after fresh pilot feedback confirms it.", "Request " + choice.second) &&
          actions_.pilot_command)
        actions_.pilot_command(choice.first, 0);
    }, supported && state_.pilot.enabled && state_.pilot.fresh);
    button->SetRole(choice.first == adapters::PilotAction::Auto ? ButtonRole::Primary : ButtonRole::Quiet);
    pilot_buttons_.push_back({button, choice.first});
  }
  EndActions();
  LiveText([](const auto &s) {
    if (s.vessel.replayed) return wxString("Historical replay / control OFF");
    if (!s.pilot.fresh) return wxString("Communication lost or unavailable. Check the pilot locally.");
    if (s.pilot.command.state == adapters::CommandState::Pending ||
        s.pilot.command.state == adapters::CommandState::Requested)
      return wxString("Waiting for pilot confirmation");
    if (s.pilot.command.state == adapters::CommandState::TimedOut)
      return wxString("No confirmation received. Check the pilot locally; no command was retried.");
    return wxString(s.pilot.enabled ? "Manual control enabled for this session" : "Control OFF / status only");
  });
  wxString enable_label = "Enable / disable manual control";
#if XNAV_ENABLE_TEST_FIXTURES
  if (state_.vessel.simulated) enable_label = "Enable / disable DEMO manual control";
#endif
  Action(enable_label, [this] {
    const bool enable = !state_.pilot.enabled;
    wxString title = "Enable physical pilot control?";
    wxString detail = "Manual buttons can move the vessel's rudder. Confirm the correct pilot, a clear drive area and immediate physical STANDBY access. Enable lasts only for this session.";
    wxString accept = "Enable manual control";
#if XNAV_ENABLE_TEST_FIXTURES
    if (state_.vessel.simulated) { title = "Enable manual simulator"; detail = "Commands affect only the labelled test simulator."; accept = "Enable DEMO"; }
#endif
    if ((!enable || ConfirmSheet(*this, mode_, title, detail, accept)) && actions_.pilot_enable)
      actions_.pilot_enable(enable);
  }, !state_.vessel.replayed && (state_.vessel.simulated || state_.settings.pilot.permit_control));
  if (!state_.vessel.simulated && !state_.vessel.replayed)
    Action("Autopilot setup & diagnostics", [this] { ShowPage(ProductPage::PilotSettings, mode_); });
}
void ProductPanel::Build() {
  Freeze();
  first_heading_ = true;
  visuals_.clear();
  button_text_.clear();
  text_.clear();
  static_text_.clear();
  action_grids_.clear();
  pilot_buttons_.clear();
  values_.clear();
  grid_ = nullptr;
  actions_grid_ = nullptr;
  DestroyChildren();
  if (GetSizer())
    SetSizer(nullptr);
  body_ = new wxBoxSizer(wxVERTICAL);
  SetSizer(body_);
  SetBackgroundColour(Colour(Theme(mode_).background));
  body_->AddSpacer(FromDIP(20));
  notice_ = new wxStaticText(this, wxID_ANY, "");
  notice_->SetFont(UiFont(*this, 14, true));
  body_->Add(notice_, 0, wxEXPAND | wxALL, FromDIP(12));
  notice_->Hide();
  SetName("OpenNav product page");
  SetLabel("OpenNav product page: " + W(PageTitle()));
  if (page_ == ProductPage::Alerts) {
    AlertsPanel();
  } else if (page_ == ProductPage::System) {
    Heading("System", "Interface, recovery and diagnostics");
    BeginActions(2);
    Action("Open Legacy OpenCPN",actions_.legacy);
    Action("Restart XNav",actions_.restart_xnav);
    Action("Safe Mode",actions_.safe);
    Action("Diagnostics",actions_.diagnostics);
    Action("Open diagnostics folder",actions_.diagnostics_folder);
    Action("Commissioning & recordings",[this]{ShowPage(ProductPage::Commissioning,mode_);});
    Action("Export diagnostic bundle",[this]{ShowPage(ProductPage::FieldReport,mode_);});
    Action("Advanced / Legacy Settings",actions_.navigation.legacy_settings);
    EndActions();
    Text("Legacy and Safe use the same navigation data and charts. Switching interface saves your work and restarts the application.");
  } else if (page_ == ProductPage::FieldReport) {
    FieldReportPanel();
  } else if (page_ == ProductPage::Commissioning) {
    CommissioningPanel();
  } else if (page_ == ProductPage::Home) {
    Heading("Navigate with OpenNav X", "Beta / Chart, vessel and passage");
    BeginActions(3);
    for (const auto &p : std::vector<std::pair<wxString, ProductPage>>{
             {"Routes", ProductPage::Routes},
             {"Waypoints", ProductPage::Waypoints},
             {"AIS targets", ProductPage::Ais},
             {"Vessel instruments", ProductPage::Instruments},
             {"SmartNav advisories", ProductPage::Advice},
             {"Manual autopilot", ProductPage::Pilot},
             {"Anchor watch", ProductPage::Anchor},
             {"Alerts", ProductPage::Alerts},
             {"Settings", ProductPage::Settings}})
      Action(p.first, [this, p] { ShowPage(p.second, mode_); });
    Action("Propulsion & energy", actions_.energy);
    Action("System & diagnostics",
           [this] { ShowPage(ProductPage::System, mode_); });
  } else if (page_ == ProductPage::Routes || page_ == ProductPage::Waypoints) {
    const bool routes = page_ == ProductPage::Routes;
    wxString source_note = state_.vessel.replayed ? "REPLAY / Saved routes and waypoints are read-only" : "Saved passages and places";
#if XNAV_ENABLE_TEST_FIXTURES
    if (state_.vessel.simulated && !state_.vessel.replayed) source_note = "DEMO telemetry / REAL saved routes and waypoints";
#endif
    Heading(routes ? "Routes" : "Waypoints", source_note);
    BeginActions(2);
    Action("Refresh catalog", [this] { Build(); });
    Action(routes ? "Waypoints" : "Routes", [this, routes] {
      ShowPage(routes ? ProductPage::Waypoints : ProductPage::Routes, mode_);
    });
    if (routes) {
      Action("Current passage", actions_.route_summary);
      Action("Create route on chart", [this] {
        if (actions_.chart) actions_.chart();
        if (actions_.navigation.start_route) actions_.navigation.start_route();
      });
    } else
      Action("Create waypoint at chart center", [this] { CreateMark(); });
    EndActions();
    if (actions_.navigation.catalog) {
      auto catalog = actions_.navigation.catalog();
      if (catalog.truncated)
        Text("Catalog capped; use Advanced route manager for remaining "
             "objects.");
      if (routes) {
        if (catalog.routes.empty())
          Text("No routes saved. Create a route or import GPX using Advanced "
               "route manager.");
        for (const auto &r : catalog.routes)
          Action(Name(r.name, r.id) +
                     wxString::Format(" / %u points",
                                      static_cast<unsigned>(r.points.size())) +
                     (r.active ? " / ACTIVE" : ""),
                 [this, r] {
                   route_ = r;
                   ShowPage(ProductPage::RouteDetail, mode_);
                 });
      } else {
        if (catalog.waypoints.empty())
          Text("No waypoints saved.");
        for (const auto &p : catalog.waypoints)
          Action(Name(p.name, p.id) + (p.in_route ? " / in route" : " / mark"),
                 [this, p] {
                   point_ = p;
                   ShowPage(ProductPage::WaypointDetail, mode_);
                 });
      }
    }
    Action("Advanced / Legacy route manager",
           actions_.navigation.legacy_route_manager);
  } else if (page_ == ProductPage::RouteDetail)
    RouteActions();
  else if (page_ == ProductPage::WaypointDetail)
    PointActions();
  else if (page_ == ProductPage::Instruments) {
    Instruments();
  } else if (page_ == ProductPage::Ais) {
    wxString traffic_note = "Traffic, closest approach and vessel information";
#if XNAV_ENABLE_TEST_FIXTURES
    if (state_.ais.simulated) traffic_note = "DEMO targets / not chart traffic";
#endif
    Heading("AIS targets", traffic_note);
    Action("Refresh target list", [this] { Build(); });
    Action("Show / hide AIS on chart", actions_.navigation.toggle_ais);
    if (state_.ais.targets.empty())
      Text("No AIS targets received. Check the AIS connection in Sensors.");
    for (const auto &t : state_.ais.targets)
      Action(Name(t.name, std::to_string(t.mmsi)) + " / " + W(t.status) +
                 (t.upstream_alarm ? " / ALARM" : ""),
             [this, id = t.mmsi] { ShowAis(id, mode_); });
  } else if (page_ == ProductPage::AisDetail) {
    const auto *selected_target = Target(state_, mmsi_);
    Heading(selected_target ? Name(selected_target->name, std::to_string(mmsi_)) : wxString("AIS target"),
            wxString::Format("MMSI %d", mmsi_));
    Visual("AIS encounter", 220, [this](XNavPainter &p, wxDC &, int width) {
      const auto *target = Target(state_, mmsi_);
      p.Card(0, 0, width, 216, target && target->upstream_alarm ? "AIS ALARM" : "VESSEL MOTION & APPROACH");
      if (!target) { p.Text("Target no longer available", 24, 64, 23, p.c.attention); return; }
      const int cell = (width - 48) / 4;
      Metric(p, target->sog_kn, state_.now, 24, 58, cell - 16, "SPEED", "kn");
      Metric(p, target->cog_deg, state_.now, 24 + cell, 58, cell - 16, "COURSE", "° TRUE", 0);
      Metric(p, target->cpa_nm, state_.now, 24 + cell * 2, 58, cell - 16, "CPA", "NM", 2);
      Metric(p, target->tcpa_minutes, state_.now, 24 + cell * 3, 58, cell - 16, "TCPA", "min", 0);
      p.Text(W(target->status), 24, 184, 12, target->upstream_alarm ? p.c.attention : p.c.secondary, false, width - 48);
    });
    BeginActions(2);
    Action("Select target on chart", [this] {
      if (actions_.navigation.view_ais) {
        const auto result = actions_.navigation.view_ais(mmsi_);
        if (!result.ok) Result(result);
        else if (actions_.chart) actions_.chart();
      }
    }, !state_.vessel.simulated && !state_.vessel.replayed)->SetRole(ButtonRole::Primary);
    Action("Back to targets", [this] { ShowPage(ProductPage::Ais, mode_); });
    EndActions();
    Visual("AIS position", 180, [this](XNavPainter &p, wxDC &, int width) {
      const auto *target = Target(state_, mmsi_);
      p.Card(0, 0, width, 176, "POSITION RELATIVE TO VESSEL");
      const int cell = (width - 48) / 3;
      Metric(p, target ? target->range_nm : vessel::Sample{}, state_.now, 24, 48, cell - 16, "RANGE", "NM");
      Metric(p, target ? target->bearing_true_deg : vessel::Sample{}, state_.now, 24 + cell, 48, cell - 16, "BEARING", "° TRUE", 0);
      Metric(p, target ? target->heading_true_deg : vessel::Sample{}, state_.now, 24 + cell * 2, 48, cell - 16, "HEADING", "° TRUE", 0);
    });
  } else if (page_ == ProductPage::Advice) {
    Heading("SmartNav", "Passage timeline / Advice only");
    auto next = [](const ProductState &s, int field) {
      vessel::Sample sample;
      for (const auto &e : s.advice.events)
        if (e.kind == smartnav::EventKind::Turn) {
          sample.value = field == 0   ? e.course_true_deg
                         : field == 1 ? e.course_change_deg
                         : field == 2 ? e.distance_nm
                                      : e.seconds_from_now;
          if (field == 3 && sample.value)
            *sample.value /= 60;
          sample.source = e.source;
          sample.observed_at = e.observed_at;
          sample.validity = vessel::Validity::Estimated;
          break;
        }
      return sample;
    };
    Value(
        "NEXT COURSE", "deg true", [next](const auto &s) { return next(s, 0); },
        0);
    Value(
        "COURSE CHANGE", "deg / advisory",
        [next](const auto &s) { return next(s, 1); }, 0);
    Value("DISTANCE TO TURN", "NM",
          [next](const auto &s) { return next(s, 2); });
    Value(
        "TIME TO TURN", "minutes / estimated",
        [next](const auto &s) { return next(s, 3); }, 0);

    LiveText([](const auto &s) {
      wxString text = W(s.advice.reason);
      for (const auto &e : s.advice.events) {
        text += "\n\n";
        text += e.seconds_from_now ? wxString::Format("+ %.0f min   ",
                                                      *e.seconds_from_now / 60)
                                   : "Time unavailable   ";
        text += W(e.title) + "\n" + W(e.detail);
      }
      return text;
    });
    Text("CHART LOOK-AHEAD / Unavailable", 18);
    Text("Chart hazard look-ahead is unavailable. Depth is measured at the "
         "boat, not ahead. Always inspect the chart and surroundings.");
  } else if (page_ == ProductPage::PilotSettings)
    PilotSettings();
  else if (page_ == ProductPage::Pilot)
    PilotActions();
  else if (page_ == ProductPage::Anchor) {
    Heading("Anchor watch", "Distance, movement and watch radius");
    Visual("Anchor watch", 224, [this](XNavPainter &p, wxDC &dc, int width) {
      p.Card(0, 0, width, 220, "ANCHOR WATCH");
      const auto &watch = state_.anchor;
      const auto distance = vessel::Assess(watch.distance_m, state_.now);
      const bool current = distance.value &&
                           (distance.quality == vessel::Quality::Live ||
                            distance.quality == vessel::Quality::Aging);
      p.Text(watch.alarm ? "ANCHOR ALARM" : !watch.anchor ? "WATCH OFF" : current ? "WATCH ACTIVE" : "POSITION UNAVAILABLE",
             24, 48, 20, watch.alarm ? p.c.alarm : current ? p.c.healthy : p.c.attention, false, width - 48);
      const int half = (width - 48) / 2;
      Metric(p, watch.distance_m, state_.now, 24, 86, half - 24, "DISTANCE FROM ANCHOR", "m", 0, 42);
      p.Text("ALARM RADIUS", 24 + half, 86, 11, p.c.secondary);
      p.Text(watch.radius_m ? wxString::Format("%.0f m", *watch.radius_m) : wxString::FromUTF8("—"),
             24 + half, 110, 42, p.c.primary, false, half - 24);
      if (current && watch.radius_m && std::abs(*watch.radius_m) > 0) {
        dc.SetPen(*wxTRANSPARENT_PEN); dc.SetBrush(wxBrush(Colour(p.c.border)));
        dc.DrawRoundedRectangle(p.D(24), p.D(200), p.D(width - 48), p.D(4), p.D(2));
        dc.SetBrush(wxBrush(Colour(watch.alarm ? p.c.alarm : p.c.accent)));
        dc.DrawRoundedRectangle(p.D(24), p.D(200),
          p.D(static_cast<int>((width - 48) * std::clamp(*distance.value / std::abs(*watch.radius_m), 0.0, 1.0))), p.D(4), p.D(2));
      }
    });
    BeginActions(2);
    Action(
        "Set anchor at vessel position",
        [this] {
          auto f =
              EditSheet(*this, mode_, "Set anchor watch",
                        "Requires fresh selected OpenCPN position. This "
                        "creates a real anchor mark.",
                        {{"Alarm radius / metres", "50", 8}}, "Set anchor");
          if (f) {
            double radius;
            const auto text = W((*f)[0]);
            if (!text.ToCDouble(&radius)) {
              Result({false, "Invalid radius", {}});
              return;
            }
            Result(actions_.navigation.start_anchor(radius));
          }
        },
        !state_.vessel.simulated && !state_.vessel.replayed);
    Action(
        "Clear anchor watch",
        [this] {
          if (ConfirmSheet(*this, mode_, "Clear anchor watch",
                           "Stops the anchor watch. A mark created only for this watch "
                           "will be removed; your existing waypoints are preserved.",
                           "Clear watch"))
            Result(actions_.navigation.clear_anchor(state_.anchor.waypoint_id));
        },
        !state_.vessel.simulated && !state_.vessel.replayed);
    EndActions();
    Visual("Anchor conditions", 180, [this](XNavPainter &p, wxDC &, int width) {
      p.Card(0, 0, width, 176, "CONDITIONS AT THE BOAT");
      const int cell = (width - 48) / 3;
      Metric(p, state_.vessel.environment.depth_below_transducer_m, state_.now, 24, 48, cell - 16, "DEPTH", "m / TRANSDUCER");
      Metric(p, state_.vessel.wind.apparent_speed_kn, state_.now, 24 + cell, 48, cell - 16, "WIND", "kn APPARENT");
      Metric(p, state_.vessel.battery.soc_percent, state_.now, 24 + cell * 2, 48, cell - 16, "BATTERY", "%", 0);
    });
    Action(anchor_history_ ? "Hide recorded positions" : "Recorded positions", [this] { anchor_history_ = !anchor_history_; Build(); });
    if (anchor_history_) {
      LiveText([](const auto &s) {
        wxString text = s.anchor.anchor ? wxString::Format("Anchor %.5f, %.5f", s.anchor.anchor->latitude_deg, s.anchor.anchor->longitude_deg) : wxString("No anchor position");
        const auto &history = s.anchor.recent_positions;
        const auto first = history.size() > 8 ? history.size() - 8 : 0;
        for (std::size_t i = first; i < history.size(); ++i)
          text += wxString::Format("\n%.0f s ago  %.5f, %.5f", std::chrono::duration<double>(s.now - history[i].observed_at).count(), history[i].position.latitude_deg, history[i].position.longitude_deg);
        return text;
      });
    }
  } else if (page_ == ProductPage::Settings) {
    Heading("Settings", "Your vessel, navigation and display");
    BeginActions(2);
    for (const auto &entry : std::vector<std::pair<wxString, ProductPage>>{
        {"VESSEL", ProductPage::VesselSettings},
        {"NAVIGATION", ProductPage::NavigationSettings},
        {"SENSORS", ProductPage::Sources},
        {"AUTOPILOT", ProductPage::PilotSettings},
        {"RADAR", ProductPage::Radar},
        {"DISPLAY", ProductPage::Display},
        {"SYSTEM", ProductPage::System}})
      Action(entry.first, [this, entry] { ShowPage(entry.second, mode_); });
    EndActions();
  } else if (page_ == ProductPage::NavigationSettings) {
    Heading("Navigation", "Passages, chart presentation and alarms");
    BeginActions(2);
    Action("Routes", [this] { ShowPage(ProductPage::Routes, mode_); });
    Action("Waypoints", [this] { ShowPage(ProductPage::Waypoints, mode_); });
    Action("Chart orientation: North / Course up", [this] {
      if (actions_.chart) actions_.chart();
      if (actions_.navigation.orientation) actions_.navigation.orientation();
    });
    Action("Advanced / Legacy Settings", actions_.navigation.legacy_settings);
    EndActions();
    Text("Units, chart presentation and navigation alarm settings remain available in Advanced / Legacy Settings.");
  } else if (page_ == ProductPage::Display) {
    DisplaySettings();
  } else if (page_ == ProductPage::RailLayout ||
             page_ == ProductPage::InstrumentLayout) {
    InstrumentSelection(page_ == ProductPage::RailLayout);
  } else if (page_ == ProductPage::EnergySettings) {
    EnergySettings();
  } else if (page_ == ProductPage::Sources) {
    Sources();
  } else if (page_ == ProductPage::SourcesAdvanced) {
    Heading("Advanced source details", "Values, source selection and freshness");
    BeginActions(2);
    for (const auto &q : vessel::Quantities())
      Action(W(q.name), [this, q] { source_quantity_ = q.quantity; ShowPage(ProductPage::SourceDetail, mode_); });
    EndActions();
  } else if (page_ == ProductPage::BoatMapping) {
    BoatMapping();
  } else if (page_ == ProductPage::SourceDetail) {
    SourceDetail();
  } else if (page_ == ProductPage::Radar) {
    Heading("Radar",
            "Radar connection and presentation");
    Action("Back to Settings",
           [this] { ShowPage(ProductPage::Settings, mode_); });
    LiveText([](const auto &s) { return W(s.radar.status); });
    LiveText([](const auto &s) {
      return "Source: " +
             W(s.radar.source.empty() ? "Unavailable" : s.radar.source) +
             " / " + (s.radar.available ? "AVAILABLE" : "NO DATA");
    });
    LiveText([](const auto &s) {
      return wxString("Overlay: ") +
             (s.radar.capabilities.overlay ? "supported" : "unavailable") +
             " / Radar Focus: " +
             (s.radar.capabilities.focus ? "supported" : "unavailable") +
             " / Receive: " +
             (s.radar.capabilities.receive ? "supported" : "unavailable");
    });
    Text("No validated radar display adapter is integrated in this Beta. "
         "Presentation remains Off. Existing compatible plugin interfaces "
         "remain accessible through Legacy; no synthetic radar is used in live "
         "mode.");
    Action("OpenCPN plugins", actions_.navigation.plugin_settings);
  } else if (page_ == ProductPage::VesselSettings) {
    Heading("Vessel safety settings",
            "Dimensions and energy");
    Action("Back to Settings",
           [this] { ShowPage(ProductPage::Settings, mode_); });
    Text("Draft and margin describe your vessel for future hazard advice. "
         "They do not change chart safety contours or depth alarms. "
         "Live chart hazard look-ahead is unavailable.");
    LiveText([](const auto &s) {
      auto value = [](double n) {
        return std::isfinite(n) ? wxString::Format("%.2f m", n)
                               : wxString("Unconfigured");
      };
      return "Draft: " + value(s.settings.hazard.draft_m) +
             " / Margin: " + value(s.settings.hazard.safety_margin_m) +
             " / Corridor half width: " + value(s.settings.hazard.corridor_half_width_m);
    });
    Action("Energy configuration", [this] { ShowPage(ProductPage::EnergySettings, mode_); });
    Action("Configure draft & margin", [this] {
      auto s = actions_.settings();
      auto n = [](double v) {
        return std::isfinite(v) ? wxString::Format("%.6g", v) : wxString{};
      };
      auto f = EditSheet(
          *this, mode_, "Vessel corridor assumptions",
          "Metres. Blank leaves a field unconfigured. This does not enable a "
          "live ENC hazard service.",
          {{"Vessel draft / m", n(s.hazard.draft_m), 64},
           {"Safety margin / m", n(s.hazard.safety_margin_m), 64},
           {"Corridor half width / m", n(s.hazard.corridor_half_width_m), 64}});
      if (!f)
        return;
      try {
        s.hazard.draft_m = application::ParseSettingNumber((*f)[0]);
        s.hazard.safety_margin_m = application::ParseSettingNumber((*f)[1]);
        s.hazard.corridor_half_width_m =
            application::ParseSettingNumber((*f)[2]);
        SaveSettings(std::move(s));
      } catch (const std::exception &e) {
        Result({false, e.what()});
      }
    });
    Action("Chart / alarm settings", actions_.navigation.legacy_settings);
  }
  body_->AddSpacer(FromDIP(24));
  Layout();
  FitInside();
  Thaw();
}
} // namespace opennav::ui

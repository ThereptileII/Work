#include "ui/ProductPanel.h"
#include "ui/Sheet.h"
#include "vessel/DataItems.h"
#include <algorithm>
#include <cmath>
#include <wx/wrapsizer.h>
namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
wxString Name(const std::string &name, const std::string &id) {
  return W(name.empty() ? id : name);
}
const vessel::AisTarget *Target(const ProductState &s, int mmsi) {
  for (const auto &t : s.ais.targets)
    if (t.mmsi == mmsi)
      return &t;
  return nullptr;
}
wxString Reading(const vessel::Sample &sample, vessel::Time now,
                 const wxString &unit) {
  auto a = vessel::Assess(sample, now);
  return (a.value ? wxString::Format("%.1f ", *a.value) + unit
                  : "Unavailable") +
         " / " + W(vessel::QualityName(a.quality));
}
} // namespace
ProductPanel::ProductPanel(wxWindow *parent, ProductActions actions)
    : wxScrolledWindow(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                       wxVSCROLL | wxBORDER_NONE),
      actions_(std::move(actions)) {
  SetScrollRate(0, FromDIP(24));
  Bind(wxEVT_SIZE, [this](wxSizeEvent &e) {
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
void ProductPanel::Heading(const wxString &title, const wxString &subtitle) {
  Text(title, 26);
  Text(subtitle, 13);
}
void ProductPanel::Text(const wxString &text, int size) {
  auto *label = new wxStaticText(this, wxID_ANY, text);
  label->SetFont(UiFont(*this, size, size > 18));
  label->SetForegroundColour(
      Colour(size > 18 ? Theme(mode_).primary : Theme(mode_).secondary));
  label->Wrap(std::max(200, GetClientSize().x - FromDIP(64)));
  body_->Add(label, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(16));
}
void ProductPanel::LiveText(
    std::function<wxString(const ProductState &)> text) {
  auto *label = new wxStaticText(this, wxID_ANY, text(state_));
  label->SetFont(UiFont(*this, 14));
  label->SetForegroundColour(Colour(Theme(mode_).secondary));
  body_->Add(label, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(16));
  text_.push_back({label, std::move(text)});
}
XNavButton *ProductPanel::Action(const wxString &label,
                                 std::function<void()> action, bool enabled) {
  auto *button = new XNavButton(this, wxID_ANY, label, label);
  button->SetMinSize(FromDIP(wxSize(200, 52)));
  button->SetLightMode(mode_);
  button->Enable(enabled);
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
void ProductPanel::BeginActions(int columns) {
  actions_grid_ = new wxGridSizer(
      std::max(1, std::min(columns, GetClientSize().x / FromDIP(212))),
      FromDIP(8), FromDIP(8));
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
  case ProductPage::Anchor:
    return "Anchor watch";
  case ProductPage::Settings:
    return "Settings";
  }
  return "Unknown";
}
void ProductPanel::ShowPage(ProductPage page, LightMode mode) {
  page_ = page;
  mode_ = mode;
  Scroll(0, 0);
  Build();
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
  state_ = state;
  if (mode != mode_) {
    mode_ = mode;
    Build();
  }
  for (auto &v : values_)
    v.first->SetReading(v.second(state), state.now);
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
              ? "ACTIVE / OpenCPN route"
              : "OpenCPN route / copied selection; actions revalidate changes");
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
      !state_.vessel.simulated && (route_.active || route_.editable));
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
  Text("PLANNED LEGS / OpenCPN stored distances and courses", 18);
  for (std::size_t i = 0; i < route_.points.size(); ++i) {
    const auto &p = route_.points[i];
    Text(wxString::Format("%u  ", static_cast<unsigned>(i + 1)) +
         Name(p.name, p.id) +
         (p.incoming_nm ? wxString::Format("   %.2f NM", *p.incoming_nm)
                        : "   Start") +
         (p.incoming_course_true_deg
              ? wxString::Format("   %.0f° true", *p.incoming_course_true_deg)
              : ""));
  }
}
void ProductPanel::PointActions() {
  Heading(Name(point_.name, point_.id),
          wxString::Format("%.6f, %.6f / OpenCPN waypoint", point_.latitude_deg,
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
      "Delete isolated waypoint",
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
void ProductPanel::PilotActions() {
  Heading("Manual autopilot", state_.vessel.simulated
                                  ? "DEMO adapter / no vessel commands"
                                  : "Live hardware output unavailable in Alpha "
                                    "/ physical validation pending");
  LiveText([](const auto &s) {
    return W(adapters::PilotModeName(s.pilot.feedback.mode)) + " / " +
           (s.pilot.fresh ? "Feedback current"
                          : "Feedback unavailable or stale") +
           " / " +
           (s.pilot.enabled ? "Manual control enabled" : "Control disabled");
  });
  LiveText([](const auto &s) {
    return "Command: " + W(adapters::CommandStateName(s.pilot.command.state)) +
           " / " + W(s.pilot.command.detail);
  });
  Action(
      "Enable / disable DEMO manual control",
      [this] {
        const bool enable = !state_.pilot.enabled;
        if (!enable ||
            ConfirmSheet(*this, mode_, "Enable manual simulator",
                         "Commands affect only the labelled autopilot "
                         "simulator. SmartNav has no command path.",
                         "Enable DEMO"))
          actions_.pilot_enable(enable);
      },
      state_.vessel.simulated);
  const auto caps = state_.pilot.capabilities;
  BeginActions(4);
  Action(
      "STANDBY",
      [this] { actions_.pilot_command(adapters::PilotAction::Standby, 0); },
      caps.standby);
  for (const auto &p : std::vector<std::pair<adapters::PilotAction, wxString>>{
           {adapters::PilotAction::Auto, "AUTO"},
           {adapters::PilotAction::Track, "TRACK"},
           {adapters::PilotAction::Wind, "WIND"}}) {
    const bool supported =
        p.first == adapters::PilotAction::Auto    ? caps.auto_mode
        : p.first == adapters::PilotAction::Track ? caps.track
                                                  : caps.wind;
    Action(
        p.second,
        [this, p] {
          if (ConfirmSheet(*this, mode_, "Request " + p.second,
                           "A pending request is not confirmation. Mode "
                           "changes require new adapter feedback.",
                           "Request " + p.second))
            actions_.pilot_command(p.first, 0);
        },
        supported);
  }
  for (int delta : {-10, -1, 1, 10})
    Action(
        wxString::Format("%+d° magnetic course", delta),
        [this, delta] {
          actions_.pilot_command(adapters::PilotAction::AlterCourse, delta);
        },
        caps.alter_course);
  Value(
      "LOCKED HEADING", "deg magnetic",
      [](const auto &s) {
        return s.pilot.feedback.locked_heading_magnetic_deg;
      },
      0);
  LiveText([](const auto &s) {
    wxString log = "RECENT COMMAND LOG";
    const auto start = s.pilot_log.size() > 8 ? s.pilot_log.size() - 8 : 0;
    for (std::size_t i = start; i < s.pilot_log.size(); ++i)
      log += "\n" +
             wxString::Format("#%llu ", static_cast<unsigned long long>(
                                            s.pilot_log[i].request.id)) +
             W(adapters::CommandStateName(s.pilot_log[i].state)) + " / " +
             W(s.pilot_log[i].detail);
    return log;
  });
}
void ProductPanel::Build() {
  Freeze();
  text_.clear();
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
  SetName("OpenNav Alpha product page");
  SetLabel("OpenNav Alpha page: " + W(PageTitle()));
  if (page_ == ProductPage::Home) {
    Heading("Navigate with OpenNav X", "Alpha / Chart, vessel and passage");
    BeginActions(3);
    for (const auto &p : std::vector<std::pair<wxString, ProductPage>>{
             {"Routes", ProductPage::Routes},
             {"Waypoints", ProductPage::Waypoints},
             {"AIS targets", ProductPage::Ais},
             {"Vessel instruments", ProductPage::Instruments},
             {"SmartNav advisories", ProductPage::Advice},
             {"Manual autopilot", ProductPage::Pilot},
             {"Anchor watch", ProductPage::Anchor},
             {"Settings", ProductPage::Settings}})
      Action(p.first, [this, p] { ShowPage(p.second, mode_); });
    Action("Chart orientation: North / Course up", [this] {
      if (actions_.chart)
        actions_.chart();
      if (actions_.navigation.orientation)
        actions_.navigation.orientation();
    });
    Action("Measure on chart", [this] {
      if (actions_.chart)
        actions_.chart();
      if (actions_.navigation.measure)
        actions_.navigation.measure();
    });
    Action("Chart information at center", [this] {
      if (actions_.chart)
        actions_.chart();
      if (actions_.navigation.object_info)
        actions_.navigation.object_info();
    });
    Action("Propulsion & energy", actions_.energy);
    Action("System & diagnostics", actions_.diagnostics);
  } else if (page_ == ProductPage::Routes || page_ == ProductPage::Waypoints) {
    const bool routes = page_ == ProductPage::Routes;
    Heading(routes ? "Routes" : "Waypoints",
            state_.vessel.simulated ? "DEMO telemetry / this catalog contains "
                                      "REAL OpenCPN navigation objects"
                                    : "Shared OpenCPN navigation objects");
    BeginActions(2);
    Action("Refresh catalog", [this] { Build(); });
    Action(routes ? "Waypoints" : "Routes", [this, routes] {
      ShowPage(routes ? ProductPage::Waypoints : ProductPage::Routes, mode_);
    });
    if (routes) {
      Action("Current passage", actions_.route_summary);
      Action("Create route on chart", [this] {
        if (ConfirmSheet(
                *this, mode_, "Create route",
                "Tap chart positions to add route points. Use Finish on the "
                "left rail when done. This creates a real OpenCPN route.",
                "Create route")) {
          if (actions_.chart)
            actions_.chart();
          if (actions_.navigation.start_route)
            actions_.navigation.start_route();
        }
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
    Heading("Vessel instruments",
            state_.vessel.simulated
                ? "DEMO / synthetic instruments"
                : "Selected marine sources / stale values retain their age");
#define VAL(label, unit, field)                                                \
  Value(label, unit, [](const auto &s) { return s.vessel.field; });
    VAL("SOG", "kn", navigation.sog_kn)
    VAL("COG", "deg true", navigation.cog_deg)
    VAL("HEADING", "deg true", navigation.heading_true_deg)
    VAL("STW", "kn", navigation.stw_kn)
    VAL("APPARENT WIND", "kn", wind.apparent_speed_kn)
    VAL("APPARENT ANGLE", "deg relative", wind.apparent_angle_deg)
    VAL("TRUE WIND", "kn", wind.true_speed_kn)
    VAL("TRUE ANGLE", "deg relative", wind.true_angle_deg)
    VAL("DEPTH", "m / transducer", environment.depth_below_transducer_m)
    VAL("WATER TEMP", "°C", environment.water_temperature_c)
    VAL("PRESSURE", "hPa", environment.pressure_hpa)
    VAL("RUDDER", "deg", rudder.angle_deg)
    VAL("HEEL", "deg", rudder.heel_deg)
#undef VAL
  } else if (page_ == ProductPage::Ais) {
    Heading("AIS targets", state_.ais.simulated
                               ? "DEMO targets / not chart traffic"
                               : "OpenCPN AIS / existing CPA, TCPA and alarms");
    Action("Refresh target list", [this] { Build(); });
    Action("Show / hide AIS on chart", actions_.navigation.toggle_ais);
    if (state_.ais.targets.empty())
      Text("No AIS targets available. Live traffic is never fabricated.");
    for (const auto &t : state_.ais.targets)
      Action(Name(t.name, std::to_string(t.mmsi)) + " / " + W(t.status) +
                 (t.upstream_alarm ? " / ALARM" : ""),
             [this, id = t.mmsi] { ShowAis(id, mode_); });
  } else if (page_ == ProductPage::AisDetail) {
    Heading(
        "AIS target",
        wxString::Format(
            "MMSI %d / OpenCPN calculations; advisory presentation", mmsi_));
    Action("Back to targets", [this] { ShowPage(ProductPage::Ais, mode_); });
    LiveText([id = mmsi_](const auto &s) {
      auto *t = Target(s, id);
      return t ? Name(t->name, std::to_string(id)) + " / " + W(t->status) +
                     (t->upstream_alarm ? " / OPENCPN ALARM" : "")
               : "Target no longer available";
    });
#define AISVAL(label, unit, field)                                             \
  Value(label, unit, [id = mmsi_](const auto &s) {                             \
    auto *t = Target(s, id);                                                   \
    return t ? t->field : vessel::Sample{};                                    \
  });
    AISVAL("SOG", "kn", sog_kn)
    AISVAL("COG", "deg true", cog_deg)
    AISVAL("HEADING", "deg true", heading_true_deg)
    AISVAL("RANGE", "NM", range_nm)
    AISVAL("BEARING", "deg true", bearing_true_deg)
    AISVAL("CPA", "NM", cpa_nm)
    AISVAL("TCPA", "min", tcpa_minutes)
#undef AISVAL
  } else if (page_ == ProductPage::Advice) {
    Heading("SmartNav", "ADVISORY ONLY / no command path to autopilot");
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
    Text("The chart-corridor query adapter is not yet connected. Measured "
         "depth is not a forecast. Absence of a detected hazard is not proof "
         "of safe water.");
  } else if (page_ == ProductPage::Pilot)
    PilotActions();
  else if (page_ == ProductPage::Anchor) {
    Heading("Anchor watch", "Uses OpenCPN anchor radius and alarm semantics / "
                            "no intelligent drag detection");
    LiveText([](const auto &s) {
      return W(s.anchor.state) + "\n" +
             (s.anchor.anchor ? wxString::Format("Anchor %.6f, %.6f",
                                                 s.anchor.anchor->latitude_deg,
                                                 s.anchor.anchor->longitude_deg)
                              : "No anchor position") +
             "\n" +
             (s.anchor.radius_m
                  ? wxString::Format("Radius %.0f m", *s.anchor.radius_m)
                  : "Radius unavailable") +
             "\n" +
             wxString::Format(
                 "%u observed movement positions",
                 static_cast<unsigned>(s.anchor.recent_positions.size()));
    });
    LiveText([](const auto &s) {
      return "VESSEL  " +
             Reading(s.vessel.navigation.latitude_deg, s.now, "lat") + " / " +
             Reading(s.vessel.navigation.longitude_deg, s.now, "lon");
    });
    LiveText([](const auto &s) {
      wxString history = "RECENT OBSERVED MOVEMENT";
      const auto &h = s.anchor.recent_positions;
      const auto first = h.size() > 5 ? h.size() - 5 : 0;
      for (std::size_t i = first; i < h.size(); ++i)
        history += wxString::Format(
            "\n%.0f s ago   %.6f, %.6f",
            std::chrono::duration<double>(s.now - h[i].observed_at).count(),
            h[i].position.latitude_deg, h[i].position.longitude_deg);
      return history;
    });
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
        !state_.vessel.simulated);
    Action(
        "Clear anchor watch",
        [this] {
          if (ConfirmSheet(*this, mode_, "Clear anchor watch",
                           "Stops this OpenCPN anchor watch. The mark remains "
                           "in your navigation database.",
                           "Clear watch"))
            Result(actions_.navigation.clear_anchor(state_.anchor.waypoint_id));
        },
        !state_.vessel.simulated);
    Value("DISTANCE FROM ANCHOR", "m",
          [](const auto &s) { return s.anchor.distance_m; });
    Value("DEPTH", "m / transducer", [](const auto &s) {
      return s.vessel.environment.depth_below_transducer_m;
    });
    Value("WIND", "kn apparent",
          [](const auto &s) { return s.vessel.wind.apparent_speed_kn; });
  } else if (page_ == ProductPage::Settings) {
    Heading("Settings", "Vessel / Navigation / Sources / Display / System");
    Action("Vessel instruments",
           [this] { ShowPage(ProductPage::Instruments, mode_); });
    Action("Energy assumptions", actions_.energy);
    Action("Data source diagnostics", actions_.diagnostics);
    Action("Autopilot permissions & status",
           [this] { ShowPage(ProductPage::Pilot, mode_); });
    EndActions();
    Text("RADAR / Unavailable", 18);
    Text("No validated radar adapter is connected. Off / Overlay / Radar Focus "
         "capabilities will be supplied by a real adapter; no synthetic live "
         "radar.");
    Action("Fullscreen / window", actions_.navigation.fullscreen);
    Action("Advanced / Legacy Settings", actions_.navigation.legacy_settings);
    Action("OpenCPN plugins", actions_.navigation.plugin_settings);
  }
  body_->AddSpacer(FromDIP(24));
  Layout();
  FitInside();
  Thaw();
}
} // namespace opennav::ui

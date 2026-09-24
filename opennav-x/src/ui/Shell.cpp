#include "ui/Shell.h"

#include "smartnav/Advisories.h"
#include <wx/accel.h>
#include <wx/datetime.h>
#include <wx/popupwin.h>
#include <wx/sizer.h>

#include <utility>

namespace opennav::ui {
namespace {
class SystemPopup final : public wxPopupTransientWindow {
public:
  explicit SystemPopup(wxWindow *parent)
      : wxPopupTransientWindow(parent, wxBORDER_NONE) {}

protected:
  void OnDismiss() override { Destroy(); }
};
} // namespace

wxPanel *Shell::MakePane(const wxString &name, wxAuiPaneInfo placement) {
  auto *panel = new wxPanel(&frame_, wxID_ANY);
  panel->SetName(name);
  manager_.AddPane(panel, placement.Name(name)
                              .CaptionVisible(false)
                              .CloseButton(false)
                              .PaneBorder(false)
                              .Resizable(true)
                              .DockFixed());
  panes_.push_back(panel);
  return panel;
}

XNavButton *Shell::Button(wxWindow *parent, const wxString &text,
                          const wxString &name,
                          std::function<void()> callback) {
  auto *button = new XNavButton(parent, wxID_ANY, text, name);
  button->Bind(wxEVT_BUTTON,
               [callback = std::move(callback)](wxCommandEvent &) {
                 if (callback)
                   callback();
               });
  buttons_.push_back(button);
  return button;
}

wxStaticText *Shell::Text(wxWindow *parent, const wxString &text, int size,
                          bool bold) {
  auto *label = new wxStaticText(parent, wxID_ANY, text);
  label->SetFont(UiFont(*parent, size, bold));
  labels_.push_back(label);
  return label;
}

Shell::Shell(wxFrame &frame, wxAuiManager &manager, ShellActions actions,
             LightMode mode, bool simulation)
    : frame_(frame), manager_(manager), actions_(std::move(actions)),
      mode_(mode), simulation_(simulation), timer_(this) {
  const int gap = frame_.FromDIP(spacing::base);
  auto *top = MakePane("OpenNavTop", wxAuiPaneInfo().Top().Layer(10).BestSize(
                                         -1, frame_.FromDIP(56)));
  auto *row = new wxBoxSizer(wxHORIZONTAL);
  row->Add(Text(top, "OpenNav X", 22, true), 0,
           wxALIGN_CENTER_VERTICAL | wxLEFT, gap * 2);
  row->AddSpacer(gap * 3);
  clock_ = Text(top, "", 15);
  clock_->SetMinSize(frame_.FromDIP(wxSize(56, 24)));
  row->Add(clock_, 0, wxALIGN_CENTER_VERTICAL);
  row->AddStretchSpacer();
  source_ = Text(top, "No vessel input", 13, true);
  row->Add(source_, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, gap * 2);
  auto *theme =
      Button(top, "Light", "Cycle day, dusk and night palettes", [this] {
        mode_ = mode_ == LightMode::Day    ? LightMode::Dusk
                : mode_ == LightMode::Dusk ? LightMode::Night
                                           : LightMode::Day;
        if (actions_.theme)
          actions_.theme(mode_);
        ApplyTheme();
      });
  theme->SetMinSize(frame_.FromDIP(wxSize(72, 48)));
  row->Add(theme, 0, wxALL, frame_.FromDIP(4));
  top->SetSizer(row);

  auto *left =
      MakePane("OpenNavTools", wxAuiPaneInfo().Left().Layer(1).BestSize(
                                   frame_.FromDIP(spacing::left_rail), -1));
  auto *tools = new wxBoxSizer(wxVERTICAL);
  tools->Add(Button(left, "+", "Zoom chart in", actions_.zoom_in), 0, wxALL,
             frame_.FromDIP(4));
  tools->Add(Button(left, wxString::FromUTF8("−"), "Zoom chart out",
                    actions_.zoom_out),
             0, wxALL, frame_.FromDIP(4));
  tools->Add(
      Button(left, "GPS", "Follow own ship using OpenCPN", actions_.follow), 0,
      wxALL, frame_.FromDIP(4));
  finish_route_ = Button(left, "Done", "Finish creating OpenCPN route",
                         actions_.navigation.finish_route);
  tools->Add(finish_route_, 0, wxALL, frame_.FromDIP(4));
  finish_route_->Hide();
  tools->AddStretchSpacer();
  left->SetSizer(tools);

  auto *right =
      MakePane("OpenNavData", wxAuiPaneInfo().Right().Layer(1).BestSize(
                                  frame_.FromDIP(spacing::right_rail), -1));
  auto *scroll = new wxScrolledWindow(right, wxID_ANY, wxDefaultPosition,
                                      wxDefaultSize, wxVSCROLL | wxBORDER_NONE);
  scroll->SetScrollRate(0, frame_.FromDIP(24));
  auto *rail = new wxBoxSizer(wxVERTICAL);
  wind_ = new XNavDataValue(scroll, "APPARENT WIND", "kn");
  depth_ = new XNavDataValue(scroll, "DEPTH", "m / transducer");
  speed_ = new XNavDataValue(scroll, "SOG", "kn");
  course_ = new XNavDataValue(scroll, "COG", "deg true", 0);
  heading_ = new XNavDataValue(scroll, "HEADING", "deg true", 0);
  for (auto *value : {wind_, depth_, speed_, course_, heading_})
    rail->Add(value, 0, wxEXPAND);
  rail->AddStretchSpacer();
  scroll->SetSizer(rail);
  auto *rail_container = new wxBoxSizer(wxVERTICAL);
  rail_container->Add(scroll, 1, wxEXPAND);
  right->SetSizer(rail_container);

  auto *bottom = MakePane("OpenNavActions",
                          wxAuiPaneInfo().Bottom().Layer(10).BestSize(
                              -1, frame_.FromDIP(spacing::action_height)));
  auto *actions_row = new wxBoxSizer(wxHORIZONTAL);
  for (const auto &entry :
       std::vector<std::pair<wxString, std::function<void()>>>{
           {"Navigation", [this] { ShowNavigation(); }},
           {"Route", [this] { ShowPage(PreviewPage::Route); }},
           {"Energy", [this] { ShowPage(PreviewPage::Energy); }},
           {"Demo", [this] { ShowDemo(); }},
           {"Menu", [this] { ShowProduct(ProductPage::Home); }}}) {
    auto *b = Button(bottom, entry.first, entry.first, entry.second);
    b->SetMinSize(
        frame_.FromDIP(wxSize(entry.first == "Navigation" ? 112 : 88, 48)));
    actions_row->Add(b, 0, wxALL, frame_.FromDIP(4));
  }
  route_summary_ = Text(bottom, "Route unavailable", 12);
  actions_row->Add(route_summary_, 0, wxALIGN_CENTER_VERTICAL | wxLEFT, gap);
  actions_row->AddStretchSpacer();
  auto *system = Button(bottom, "System", "System and Open Legacy OpenCPN",
                        [this] { ShowSystem(); });
  system->SetMinSize(frame_.FromDIP(wxSize(112, 48)));
  actions_row->Add(system, 0, wxALL, frame_.FromDIP(4));
  bottom->SetSizer(actions_row);
  page_ = new PreviewPanel(&frame_);
  // An unmanaged overlay is reordered behind ChartCanvas by the native AUI
  // resize path. Use the same layout manager for the alternate center page.
  manager_.AddPane(page_, wxAuiPaneInfo()
                              .Name("OpenNavPage")
                              .CenterPane()
                              .PaneBorder(false)
                              .Hide());
  ProductActions product_actions;
  product_actions.navigation = actions_.navigation;
  product_actions.settings = actions_.settings;
  product_actions.save_settings = actions_.save_settings;
  product_actions.chart = [this] { ShowNavigation(); };
  product_actions.route_summary = [this] { ShowPage(PreviewPage::Route); };
  product_actions.energy = [this] { ShowPage(PreviewPage::Energy); };
  product_actions.diagnostics = [this] { ShowPage(PreviewPage::Diagnostics); };
  product_actions.pilot_command = [this](auto action, double delta) {
    if (actions_.pilot_command)
      actions_.pilot_command(simulation_, action, delta);
  };
  product_actions.pilot_enable = [this](bool enabled) {
    if (actions_.pilot_enable)
      actions_.pilot_enable(simulation_, enabled);
  };
  product_ = new ProductPanel(&frame_, std::move(product_actions));
  manager_.AddPane(product_, wxAuiPaneInfo()
                                 .Name("OpenNavProduct")
                                 .CenterPane()
                                 .PaneBorder(false)
                                 .Hide());
  std::vector<std::pair<int, std::function<void()>>> commands = {
      {'M', [this] { ShowProduct(ProductPage::Home); }},
      {'W', [this] { ShowProduct(ProductPage::Waypoints); }},
      {'B', [this] { ShowProduct(ProductPage::Routes); }},
      {'A', [this] { ShowProduct(ProductPage::Ais); }},
      {'V', [this] { ShowProduct(ProductPage::Instruments); }},
      {'J', [this] { ShowProduct(ProductPage::Advice); }},
      {'Y', [this] { ShowProduct(ProductPage::Pilot); }},
      {'H', [this] { ShowProduct(ProductPage::Anchor); }},
      {'G', [this] { ShowProduct(ProductPage::Settings); }},
      {'K', [this] { ShowProduct(ProductPage::EnergySettings); }},
      {'O', [this] { ShowProduct(ProductPage::Sources); }},
      {'Q', [this] { ShowProduct(ProductPage::VesselSettings); }},
      {'Z', [this] { ShowProduct(ProductPage::Radar); }},
      {'D', [this] { StartDemo(); }},
      {'P',
       [this] {
         simulation_paused_ = !simulation_paused_;
         demo_.Pause(simulation_paused_, vessel::Clock::now());
       }},
      {'L', actions_.legacy},
      {'N', [this] { ShowNavigation(); }},
      {'R', [this] { ShowPage(PreviewPage::Route); }},
      {'E', [this] { ShowPage(PreviewPage::Energy); }},
      {'I', [this] { ShowPage(PreviewPage::Diagnostics); }},
      {'S', [this] { ShowSystem(); }},
      {'T', [this] { ShowDemo(); }}};
  for (int i = 0; i < 8; ++i)
    commands.push_back({WXK_F1 + i, [this, i] {
                          SelectDemo(static_cast<vessel::DemoScenario>(i));
                        }});
  std::vector<wxAcceleratorEntry> accelerators;
  for (const auto &command : commands) {
    const int id = wxWindow::NewControlId();
    accelerators.emplace_back(wxACCEL_CTRL | wxACCEL_SHIFT, command.first, id);
    frame_.Bind(wxEVT_MENU, &Shell::OnCommand, this, id);
    commands_.push_back({id, command.second});
  }
  frame_.SetAcceleratorTable(wxAcceleratorTable(
      static_cast<int>(accelerators.size()), accelerators.data()));
  ApplyTheme();
  Tick();
  manager_.Update();
  Bind(wxEVT_TIMER, [this](wxTimerEvent &) { Tick(); });
  timer_.Start(250);
}

Shell::~Shell() {
  timer_.Stop();
  for (const auto &c : commands_)
    frame_.Unbind(wxEVT_MENU, &Shell::OnCommand, this, c.first);
  frame_.SetAcceleratorTable(wxNullAcceleratorTable);
  if (page_) {
    ShowNavigation();
    manager_.DetachPane(page_);
    page_->Destroy();
  }
  if (product_) {
    manager_.DetachPane(product_);
    product_->Destroy();
  }
  for (auto *pane : panes_) {
    manager_.DetachPane(pane);
    pane->Destroy();
  }
  manager_.Update();
}

void Shell::UpdateState(const vessel::VesselState &state) {
  if (!simulation_)
    state_ = state;
}

void Shell::ApplyTheme() {
  const auto colors = Theme(mode_);
  for (auto *pane : panes_) {
    pane->SetBackgroundColour(Colour(colors.surface));
    pane->Refresh();
  }
  for (auto *label : labels_)
    label->SetForegroundColour(Colour(colors.secondary));
  for (auto *button : buttons_)
    button->SetLightMode(mode_);
  for (auto *value : {wind_, depth_, speed_, course_, heading_})
    value->SetLightMode(mode_);
  source_->SetForegroundColour(
      Colour(simulation_ ? colors.attention : colors.secondary));
}

void Shell::Tick() {
  const auto now = vessel::Clock::now();
  if (simulation_)
    state_ = demo_.Read(now);
  else {
    if (actions_.live_state)
      state_ = actions_.live_state();
    if (actions_.route)
      state_.navigation.route = actions_.route();
  }
  const auto config =
      actions_.settings ? actions_.settings() : application::Settings{};
  const auto model =
      simulation_ ? smartnav::PreviewEnergyModel(true) : config.energy.battery;
  const auto energy =
      simulation_
          ? smartnav::PredictVesselEnergy(model, state_, now)
          : smartnav::PredictConfiguredEnergy(config.energy, state_, now);
  const bool creating = actions_.route_creating && actions_.route_creating();
  if (finish_route_->IsShown() != creating) {
    finish_route_->Show(creating);
    finish_route_->GetParent()->Layout();
  }
  if (product_) {
    ProductState p;
    p.vessel = state_;
    p.now = now;
    if (simulation_)
      p.ais = vessel::DemoAis(state_);
    else if (actions_.navigation.ais)
      p.ais = actions_.navigation.ais();
    if (!simulation_ && actions_.navigation.anchor)
      p.anchor = actions_.navigation.anchor();
    else
      p.anchor.state =
          "No DEMO anchor watch / real anchor controls disabled in DEMO";
    if (actions_.pilot_tick)
      p.pilot = actions_.pilot_tick(simulation_);
    if (actions_.pilot_log)
      p.pilot_log = actions_.pilot_log(simulation_);
    p.settings = config;
    if (actions_.settings_status)
      p.settings_status = actions_.settings_status();
    if (actions_.source_health)
      p.sources = actions_.source_health();
    if (actions_.radar)
      p.radar = actions_.radar();
    p.advice = smartnav::Advise(state_, energy, p.ais, now);
    product_->Update(p, mode_);
  }
  wind_->SetReading(state_.wind.apparent_speed_kn, now);
  depth_->SetReading(state_.environment.depth_below_transducer_m, now);
  speed_->SetReading(state_.navigation.sog_kn, now);
  course_->SetReading(state_.navigation.cog_deg, now);
  heading_->SetReading(state_.navigation.heading_true_deg, now);
  clock_->SetLabel(simulation_ ? "10:42" : wxDateTime::Now().Format("%H:%M"));
  const wxString label =
      simulation_
          ? "DEMO / " + wxString(simulation_paused_
                                     ? "PAUSED"
                                     : wxString::FromUTF8(vessel::ScenarioName(
                                           demo_.Scenario())))
          : InputSummary();
  if (source_->GetLabel() != label) {
    source_->SetLabel(label);
    source_->GetParent()->Layout();
  }
  const auto route = state_.navigation.route;
  const auto distance =
      route ? vessel::AssessRoute(*route, now).remaining_distance_nm
            : std::nullopt;
  const wxString summary =
      distance ? wxString::Format("%.1f NM to destination", *distance)
               : "Route unavailable";
  const bool show_summary = frame_.GetClientSize().x >= frame_.FromDIP(1060);
  const bool summary_layout = route_summary_->GetLabel() != summary ||
                              route_summary_->IsShown() != show_summary;
  route_summary_->SetLabel(summary);
  route_summary_->Show(show_summary);
  if (summary_layout)
    route_summary_->GetParent()->Layout();
  if (page_ && page_->IsShown())
    page_->Update(current_page_, mode_, state_, now, model, energy,
                  actions_.build_info ? actions_.build_info()
                                      : std::vector<std::string>{});
  if (actions_.diagnostic_snapshot)
    actions_.diagnostic_snapshot(state_, energy, PageTitle());
}

std::string Shell::PageTitle() const {
  if (product_ && product_->IsShown())
    return product_->PageTitle();
  if (page_ && page_->IsShown())
    return current_page_ == PreviewPage::Route    ? "Route"
           : current_page_ == PreviewPage::Energy ? "Energy"
                                                  : "Diagnostics";
  return "Navigation";
}
void Shell::OnCommand(wxCommandEvent &event) {
  for (const auto &c : commands_)
    if (c.first == event.GetId()) {
      const auto action = c.second;
      if (action)
        action();
      return;
    }
}
void Shell::StartDemo() {
  SelectDemo(vessel::DemoScenario::Cruise);
  if (actions_.demo_chart)
    actions_.demo_chart();
}
void Shell::SelectDemo(vessel::DemoScenario scenario) {
  simulation_ = true;
  simulation_paused_ = false;
  demo_.Select(scenario, vessel::Clock::now());
  ApplyTheme();
  Tick();
}
void Shell::ShowNavigation() {
  manager_.GetPane(page_).Hide();
  if (product_)
    manager_.GetPane(product_).Hide();
  for (const auto &saved : navigation_visibility_) {
    auto &pane = manager_.GetPane(saved.first);
    if (pane.IsOk())
      pane.Show(saved.second);
  }
  navigation_visibility_.clear();
  manager_.Update();
  // A hidden page must not keep keyboard focus (GTK drops frame accelerators
  // in that state). Restore focus to a visible chart without reparenting it.
  for (const auto &name : actions_.navigation_panes) {
    auto &pane = manager_.GetPane(name);
    if (pane.IsOk() && pane.IsShown() && pane.window) {
      pane.window->SetFocus();
      break;
    }
  }
  frame_.Refresh();
}
void Shell::ShowProduct(ProductPage page) {
  ShowPage(PreviewPage::Route);
  manager_.GetPane(page_).Hide();
  manager_.GetPane(product_).Show();
  product_->ShowPage(page, mode_);
  manager_.Update();
  product_->SetFocus();
  Tick();
}
void Shell::ShowObject(const std::string &id, bool route) {
  ShowProduct(route ? ProductPage::Routes : ProductPage::Waypoints);
  product_->ShowObject(id, route, mode_);
  Tick();
}
void Shell::ShowAis(int mmsi) {
  ShowProduct(ProductPage::Ais);
  product_->ShowAis(mmsi, mode_);
  Tick();
}
void Shell::ShowPage(PreviewPage page) {
  if (product_)
    manager_.GetPane(product_).Hide();
  if (navigation_visibility_.empty()) {
    auto names = actions_.navigation_panes;
    names.push_back("OpenNavTools");
    names.push_back("OpenNavData");
    for (const auto &name : names) {
      auto &pane = manager_.GetPane(name);
      if (pane.IsOk()) {
        navigation_visibility_.push_back({name, pane.IsShown()});
        pane.Hide();
      }
    }
  }
  current_page_ = page;
  manager_.GetPane(page_).Show();
  manager_.Update();
  page_->SetFocus();
  Tick();
}

void Shell::ShowDemo() {
  auto *popup = new SystemPopup(&frame_);
  popup->SetBackgroundColour(Colour(Theme(mode_).elevated));
  auto *layout = new wxBoxSizer(wxVERTICAL);
  auto *title =
      new wxStaticText(popup, wxID_ANY, "DEMO / Deterministic trip at 60x");
  title->SetFont(UiFont(*popup, 18, true));
  title->SetForegroundColour(Colour(Theme(mode_).attention));
  layout->Add(title, 0, wxALL, frame_.FromDIP(16));
  auto *grid = new wxGridSizer(2, frame_.FromDIP(8), frame_.FromDIP(8));
  for (auto scenario :
       {vessel::DemoScenario::Cruise, vessel::DemoScenario::Stale,
        vessel::DemoScenario::Unavailable, vessel::DemoScenario::RouteInactive,
        vessel::DemoScenario::RouteEnding, vessel::DemoScenario::LowSoc,
        vessel::DemoScenario::HighPower, vessel::DemoScenario::Insufficient}) {
    auto *b = new XNavButton(popup, wxID_ANY,
                             wxString::FromUTF8(vessel::ScenarioName(scenario)),
                             "Explicit demo scenario");
    b->SetMinSize(frame_.FromDIP(wxSize(192, 48)));
    b->SetLightMode(mode_);
    b->Bind(wxEVT_BUTTON, [this, popup, scenario](wxCommandEvent &) {
      popup->Dismiss();
      popup->Destroy();
      SelectDemo(scenario);
    });
    grid->Add(b, 0, wxEXPAND);
  }
  layout->Add(grid, 0, wxLEFT | wxRIGHT | wxBOTTOM, frame_.FromDIP(16));
  popup->SetSizerAndFit(layout);
  popup->Position(
      frame_.ClientToScreen(wxPoint(frame_.FromDIP(80), frame_.FromDIP(100))),
      wxSize());
  popup->Popup();
}

wxString Shell::InputSummary() const {
  bool present = false, current = false;
  for (const auto *sample :
       {&state_.navigation.latitude_deg, &state_.navigation.sog_kn,
        &state_.navigation.cog_deg}) {
    const auto assessment = vessel::Assess(*sample, vessel::Clock::now());
    present = present || assessment.value.has_value();
    current = current || assessment.quality == vessel::Quality::Live ||
              assessment.quality == vessel::Quality::Aging;
  }
  return current   ? "OpenCPN navigation"
         : present ? "Navigation stale"
                   : "No vessel input";
}

void Shell::ShowSystem() {
  auto *popup = new SystemPopup(&frame_);
  popup->SetBackgroundColour(Colour(Theme(mode_).elevated));
  auto *layout = new wxBoxSizer(wxVERTICAL);
  const int gap = frame_.FromDIP(12);
  auto *heading = new wxStaticText(popup, wxID_ANY, "System");
  heading->SetFont(UiFont(*popup, 20, true));
  heading->SetForegroundColour(Colour(Theme(mode_).primary));
  layout->Add(heading, 0, wxALL, gap);
  auto *info = new wxStaticText(
      popup, wxID_ANY,
      "OpenNav X / Alpha development\nOpenCPN 5.12.4 / API 1.20\nMode: XNav\n" +
          (simulation_ ? wxString("Data: explicit simulator")
                       : "Data: " + InputSummary()) +
          "\nLive hardware output: disabled");
  info->SetFont(UiFont(*popup, 13));
  info->SetForegroundColour(Colour(Theme(mode_).secondary));
  layout->Add(info, 0, wxLEFT | wxRIGHT | wxBOTTOM, gap);
  auto *pause = new XNavButton(
      popup, wxID_ANY,
      simulation_
          ? (simulation_paused_ ? "Resume simulation" : "Pause simulation")
          : "Start labelled simulation",
      "Simulator control; never affects OpenCPN chart or device state");
  pause->SetLightMode(mode_);
  pause->Bind(wxEVT_BUTTON, [this, popup](wxCommandEvent &) {
    if (!simulation_)
      StartDemo();
    else {
      simulation_paused_ = !simulation_paused_;
      demo_.Pause(simulation_paused_, vessel::Clock::now());
    }
    ApplyTheme();
    popup->Dismiss();
    popup->Destroy();
  });
  layout->Add(pause, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
  auto *legacy = new XNavButton(popup, wxID_ANY, "Open Legacy OpenCPN",
                                "Save and restart in Legacy OpenCPN");
  legacy->SetLightMode(mode_);
  legacy->Bind(wxEVT_BUTTON, [this, popup](wxCommandEvent &) {
    // Closing the host can synchronously destroy this Shell and its actions.
    const auto restart_action = actions_.legacy;
    popup->Dismiss();
    popup->Destroy();
    if (restart_action)
      restart_action();
  });
  layout->Add(legacy, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
  for (const auto &entry :
       std::vector<std::pair<wxString, std::function<void()>>>{
           {"Diagnostics", [this] { ShowPage(PreviewPage::Diagnostics); }},
           {"Restart XNav", actions_.restart_xnav},
           {"Safe Mode", actions_.safe},
           {"Open diagnostics folder", actions_.diagnostics_folder}}) {
    auto *b = new XNavButton(popup, wxID_ANY, entry.first, entry.first);
    b->SetLightMode(mode_);
    b->Bind(wxEVT_BUTTON, [popup, action = entry.second](wxCommandEvent &) {
      popup->Dismiss();
      popup->Destroy();
      if (action)
        action();
    });
    layout->Add(b, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
  }
  popup->SetSizerAndFit(layout);
  const auto client = frame_.GetClientSize();
  const auto corner = frame_.ClientToScreen(wxPoint(client.x, client.y));
  popup->Move(corner.x - popup->GetSize().x - gap,
              corner.y - popup->GetSize().y - frame_.FromDIP(64));
  popup->Popup();
}

} // namespace opennav::ui

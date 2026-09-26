#include "ui/Shell.h"

#include "smartnav/Advisories.h"
#include "vessel/DisplayItems.h"
#include "ui/Sheet.h"
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
      : wxPopupTransientWindow(parent, wxBORDER_NONE) {
    // GTK's generic transient focus handler observes CHAR, while focused
    // custom controls receive CHAR_HOOK first. Close explicitly on Escape so
    // the popup's pointer grab cannot consume the next navigation action.
    Bind(wxEVT_CHAR_HOOK, [this](wxKeyEvent &event) {
      if (event.GetKeyCode() != WXK_ESCAPE) { event.Skip(); return; }
      auto *parent = GetParent();
      Dismiss();
      Destroy();
      if (parent) parent->SetFocus();
    });
  }

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
      mode_(mode), simulation_(simulation && integration::TestFixturesEnabled()), timer_(this) {
  const int gap = frame_.FromDIP(spacing::base);
  auto *top = MakePane("OpenNavTop", wxAuiPaneInfo().Top().Layer(10).BestSize(
                                         -1, frame_.FromDIP(56)));
  auto *row = new wxBoxSizer(wxHORIZONTAL);
  row->Add(Text(top, "OpenNav X", 18, false), 0,
           wxALIGN_CENTER_VERTICAL | wxLEFT, gap * 2);
  row->AddSpacer(gap * 2);
  clock_ = Text(top, "", 15);
  clock_->SetMinSize(frame_.FromDIP(wxSize(56, 24)));
  row->Add(clock_, 0, wxALIGN_CENTER_VERTICAL);
  source_ = new wxStaticText(top, wxID_ANY, "No vessel input", wxDefaultPosition,
                             wxDefaultSize, wxST_ELLIPSIZE_END);
  source_->SetFont(UiFont(*top, 13, true));
  source_->SetMinSize(wxSize(0, -1));
  labels_.push_back(source_);
  row->Add(source_, 1, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, gap * 2);
  page_up_ = Button(top, "Up", "Scroll page up", [this] {
    if (auto *s = CurrentScroll()) s->Step(-1);
    UpdateScrollControls();
  });
  page_down_ = Button(top, "Down", "Scroll page down", [this] {
    if (auto *s = CurrentScroll()) s->Step(1);
    UpdateScrollControls();
  });
  for (auto *b : {page_up_, page_down_}) {
    b->SetMinSize(frame_.FromDIP(wxSize(64, 48)));
    row->Add(b, 0, wxALL, frame_.FromDIP(4));
    b->Hide();
  }
  theme_button_ =
      Button(top, LightName(), "Cycle day, dusk and night palettes", [this] {
        SetLight(mode_ == LightMode::Day    ? LightMode::Dusk
                 : mode_ == LightMode::Dusk ? LightMode::Night
                                            : LightMode::Day);
      });
  theme_button_->SetMinSize(frame_.FromDIP(wxSize(72, 48)));
  theme_button_->SetRole(ButtonRole::Quiet);
  // Alerts occupy the existing status slot. They never steal chart/rail height.
  alert_pane_ = new wxPanel(top, wxID_ANY);
  alert_pane_->SetName("OpenNavAlerts");
  alert_pane_->Hide();
  auto *alert_row = new wxBoxSizer(wxHORIZONTAL);
  alert_label_ = new wxStaticText(alert_pane_, wxID_ANY, "", wxDefaultPosition,
      wxDefaultSize, wxST_ELLIPSIZE_END);
  alert_label_->SetFont(UiFont(*alert_pane_, 15, true));
  alert_label_->SetMinSize(wxSize(0, -1));
  alert_row->Add(alert_label_, 1, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, gap * 2);
  alert_button_ = Button(alert_pane_, "Alerts", "Inspect active alerts", [this] { ShowProduct(ProductPage::Alerts); });
  alert_button_->SetMinSize(frame_.FromDIP(wxSize(88, 48)));
  alert_row->Add(alert_button_, 0, wxALL, frame_.FromDIP(4));
  alert_pane_->SetSizer(alert_row);
  row->Add(alert_pane_,1,wxEXPAND);
  row->Add(theme_button_,0,wxALL,frame_.FromDIP(4));
  auto *menu=Button(top,"Menu","Open navigation menu",[this]{ShowProduct(ProductPage::Home);});
  menu->SetIcon(XNavIcon::Menu);menu->SetRole(ButtonRole::Quiet);
  menu->SetMinSize(frame_.FromDIP(wxSize(64,48)));
  row->Add(menu,0,wxALL,frame_.FromDIP(4));
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
  auto *center=Button(left,"Center","Center chart on boat and follow position",actions_.follow);
  center->SetIcon(XNavIcon::Ownship);center->SetMinSize(frame_.FromDIP(wxSize(56,64)));
  tools->Add(center,0,wxALL,frame_.FromDIP(4));
  finish_route_ = Button(left, "Done", "Name and save this route", [this] {
    const auto fields=EditSheet(frame_,mode_,"Save route",
      "Name this route. You can activate it after saving.",
      {{"Name","",128},{"Description","",2048}},"Save route");
    if(!fields || !actions_.navigation.finish_route_named) return;
    const auto result=actions_.navigation.finish_route_named((*fields)[0],(*fields)[1]);
    if(result.ok) { Tick(); ShowObject(result.identity,true); }
    else ConfirmSheet(frame_,mode_,"Route not saved",wxString::FromUTF8(result.message),"Back");
  });
  tools->Add(finish_route_, 0, wxALL, frame_.FromDIP(4));
  finish_route_->Hide();
  undo_route_=Button(left,"Undo","Undo last route point",[this]{if(actions_.navigation.undo_route_point)actions_.navigation.undo_route_point();});
  cancel_route_=Button(left,"Cancel","Cancel route creation",[this]{
    if(actions_.navigation.cancel_route && ConfirmSheet(frame_,mode_,"Cancel route?","Discard this unfinished route? Existing routes are preserved.","Discard route"))actions_.navigation.cancel_route();
  });
  for(auto *b:{undo_route_,cancel_route_}){tools->Add(b,0,wxALL,frame_.FromDIP(4));b->Hide();}
  tools->AddStretchSpacer();
  left->SetSizer(tools);

  auto *right =
      MakePane("OpenNavData", wxAuiPaneInfo().Right().Layer(1).BestSize(
                                  frame_.FromDIP(spacing::right_rail), -1));
  rail_scroll_ = new wxPanel(right,wxID_ANY);
  rail_scroll_->SetSizer(new wxBoxSizer(wxVERTICAL));
  auto *rail_container = new wxBoxSizer(wxVERTICAL);
  rail_container->Add(rail_scroll_, 1, wxEXPAND);
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
           {"Pilot", [this] { ShowProduct(ProductPage::Pilot); }},
           {"STBY", [this] {
              if (actions_.pilot_command && !state_.replayed)
                actions_.pilot_command(simulation_, adapters::PilotAction::Standby, 0);
            }},
#if XNAV_ENABLE_TEST_FIXTURES
           {"Demo", [this] { ShowDemo(); }},
#endif
           }) {
    auto *b = Button(bottom, entry.first, entry.first, entry.second);
    b->SetMinSize(
        frame_.FromDIP(wxSize(entry.first == "Navigation" ? 112 : entry.first == "STBY" ? 64 : 88, 48)));
    if (entry.first == "STBY") {
      standby_ = b;
      b->SetName("Manual STANDBY / requires enabled control");
      b->SetToolTip("Manual STANDBY / requires enabled control; physical STANDBY remains independent");
      b->Disable();
      b->SetRole(ButtonRole::Critical);
    }
    else b->SetRole(ButtonRole::Quiet);
    actions_row->Add(b, 0, wxALL, frame_.FromDIP(4));
  }
  route_summary_ = new wxStaticText(bottom,wxID_ANY,"No active route",wxDefaultPosition,wxDefaultSize,wxST_ELLIPSIZE_END);
  route_summary_->SetFont(UiFont(*bottom,14));route_summary_->SetMinSize(wxSize(0,-1));labels_.push_back(route_summary_);
  actions_row->Add(route_summary_, 1, wxALIGN_CENTER_VERTICAL | wxLEFT, gap);
  auto *system = Button(bottom, "System", "System and Open Legacy OpenCPN",
                        [this] { ShowSystem(); });
  system->SetMinSize(frame_.FromDIP(wxSize(112, 48)));
  system->SetRole(ButtonRole::Quiet);
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
  product_actions.acknowledge_alert = [this](const std::string &id, std::uint64_t episode) {
    alerts_.Acknowledge(id, episode);
    Tick();
  };
  product_actions.field_bundle = [this](const std::optional<std::string> &recording) {
    return diagnostics::BuildFieldReport(field_snapshot_,
        actions_.field_environment ? actions_.field_environment() : diagnostics::FieldEnvironment{},
        field_journal_, vessel::Clock::now(), recording);
  };
  product_actions.commissioning = actions_.commissioning;
  product_actions.navigation = actions_.navigation;
  product_actions.navigation.legacy_settings=[this]{ShowNavigation();if(actions_.navigation.legacy_settings)actions_.navigation.legacy_settings();};
  product_actions.navigation.plugin_settings=[this]{ShowNavigation();if(actions_.navigation.plugin_settings)actions_.navigation.plugin_settings();};
  product_actions.navigation.view_ais = [this](int mmsi) {
    if (state_.simulated || state_.replayed || !actions_.navigation.view_ais ||
        !ais_selection_.Select(mmsi, ais_state_, vessel::Clock::now()))
      return application::CommandResult{false, "A fresh live AIS target is required"};
    const auto result = actions_.navigation.view_ais(mmsi);
    if (result.ok) ShowNavigation(); else ais_selection_.Clear();
    return result;
  };
  product_actions.settings = actions_.settings;
  product_actions.theme = [this](LightMode mode) { SetLight(mode); };
  product_actions.save_settings = actions_.save_settings;
  product_actions.chart = [this] { ShowNavigation(); };
  product_actions.route_summary = [this] { ShowPage(PreviewPage::Route); };
  product_actions.energy = [this] { ShowPage(PreviewPage::Energy); };
  product_actions.diagnostics = [this] { ShowPage(PreviewPage::Diagnostics); };
  product_actions.legacy=actions_.legacy;
  product_actions.restart_xnav=actions_.restart_xnav;
  product_actions.safe=actions_.safe;
  product_actions.diagnostics_folder=actions_.diagnostics_folder;
  product_actions.pilot_command = [this](auto action, double delta) {
    if (actions_.pilot_command &&
        (!actions_.commissioning ||
         actions_.commissioning->AllowsHardwareControl()))
      actions_.pilot_command(simulation_, action, delta);
  };
  product_actions.pilot_enable = [this](bool enabled) {
    if (actions_.pilot_enable &&
        (!enabled || !actions_.commissioning ||
         actions_.commissioning->AllowsHardwareControl()))
      actions_.pilot_enable(simulation_, enabled);
  };
  product_actions.pilot_identity = [this] {
    if (simulation_ || !actions_.pilot_identity ||
        (actions_.commissioning && !actions_.commissioning->AllowsHardwareControl()))
      return application::CommandResult{false, "Identity refresh requires live commissioning mode"};
    return actions_.pilot_identity();
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
      {'C', [this] { ShowProduct(ProductPage::Commissioning); }},
      {'X', [this] { ShowProduct(ProductPage::FieldReport); }},
      {WXK_F9, [this] { ShowProduct(ProductPage::Alerts); }},
      {'Q', [this] { ShowProduct(ProductPage::VesselSettings); }},
      {'Z', [this] { ShowProduct(ProductPage::Radar); }},
      {'F', [this] { ShowProduct(ProductPage::Display); }},
#if XNAV_ENABLE_TEST_FIXTURES
      {'D', [this] { StartDemo(); }},
      {'P',
       [this] {
         simulation_paused_ = !simulation_paused_;
         demo_.Pause(simulation_paused_, vessel::Clock::now());
       }},
#endif
      {'L', actions_.legacy},
      {'N', [this] { ShowNavigation(); }},
      {'R', [this] { ShowPage(PreviewPage::Route); }},
      {'E', [this] { ShowPage(PreviewPage::Energy); }},
      {'I', [this] { ShowPage(PreviewPage::Diagnostics); }},
      {'S', [this] { ShowSystem(); }},
#if XNAV_ENABLE_TEST_FIXTURES
      {'T', [this] { ShowDemo(); }}};
  for (int i = 0; i < 8; ++i)
    commands.push_back({WXK_F1 + i, [this, i] {
                          SelectDemo(static_cast<vessel::DemoScenario>(i));
                        }});
#else
      };
#endif
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

std::vector<ProductGeometry> Shell::RailRegions() const {
  std::vector<ProductGeometry> result;
  if (!rail_scroll_ || !rail_scroll_->IsShownOnScreen()) return result;
  const auto bounds=rail_scroll_->GetScreenRect();
  for (const auto &entry:rail_values_) {
    const auto rect=entry.second->GetScreenRect();
    result.push_back({entry.first,rect,entry.second->IsEnabled(),
                      entry.second->IsShownOnScreen() && bounds.Contains(rect)});
  }
  return result;
}

std::vector<ProductGeometry> Shell::InteractionControls() const {
  std::vector<ProductGeometry> result;
  // Copy native bounds only. This diagnostic walk cannot invoke actions or
  // retain widget lifetimes in a consumer. Modal sheets and transient chart
  // cards are owned children too, even when they have a separate native HWND.
  const auto collect = [&](const auto &self, wxWindow *window) -> void {
    if (dynamic_cast<XNavButton *>(window)) {
      const auto rectangle = window->GetScreenRect();
      bool visible = window->IsShownOnScreen();
      for (auto *parent = window->GetParent(); parent && !parent->IsTopLevel();
           parent = parent->GetParent())
        visible = visible && parent->GetScreenRect().Contains(rectangle);
      result.push_back({window->GetLabel().ToStdString(wxConvUTF8), rectangle,
                        window->IsEnabled(), visible});
    }
    for (auto *child : window->GetChildren()) self(self, child);
  };
  collect(collect, &frame_);
  return result;
}

void Shell::UpdateState(const vessel::VesselState &state) {
  if (!simulation_ &&
      (!actions_.commissioning || !actions_.commissioning->Replaying()))
    state_ = state;
}

void Shell::ApplyTheme() {
  const auto colors = Theme(mode_);
  caption_themed_ = ThemeWindowChrome(frame_, mode_);
  rail_scroll_->SetBackgroundColour(Colour(colors.surface));
  alert_pane_->SetBackgroundColour(Colour(colors.surface));
  theme_button_->SetLabel(LightName());
  for (auto *pane : panes_) {
    pane->SetBackgroundColour(Colour(colors.surface));
    pane->Refresh();
  }
  for (auto *label : labels_)
    label->SetForegroundColour(Colour(colors.secondary));
  for (auto *button : buttons_)
    button->SetLightMode(mode_);
  for (const auto &value : rail_values_)
    value.second->SetLightMode(mode_);
  source_->SetForegroundColour(
      Colour(simulation_ ? colors.attention : colors.secondary));
}

void Shell::SetLight(LightMode mode) {
  mode_ = mode;
  if (actions_.theme)
    actions_.theme(mode);
  ApplyTheme();
}
void Shell::UpdateRail(const std::vector<std::string> &keys, vessel::Time now) {
  const auto items = vessel::DisplayItems(state_);
  // Four primary instruments always fit. Older profiles retain their complete
  // preference list; extra choices remain available in Instruments/settings.
  std::vector<std::string> visible(keys.begin(),keys.begin()+std::min<std::size_t>(4,keys.size()));
  if (visible != rail_keys_) {
    rail_scroll_->Freeze();
    rail_scroll_->GetSizer()->Clear(true);
    rail_values_.clear();
    rail_keys_ = visible;
    for (const auto &key : visible)
      for (const auto &item : items)
        if (key == item.key) {
          auto *value =
              new XNavDataValue(rail_scroll_, wxString::FromUTF8(item.title),
                                wxString::FromUTF8(item.unit));
          value->SetLightMode(mode_);
          value->SetCompact(true);
          rail_values_.push_back({key, value});
          rail_scroll_->GetSizer()->Add(value, 1, wxEXPAND);
        }
    rail_scroll_->Layout();
    rail_scroll_->Thaw();
  }
  for (const auto &value : rail_values_)
    for (const auto &item : items)
      if (value.first == item.key)
        value.second->SetReading(*item.sample, now);
}

void Shell::UpdateAlerts() {
  const auto &alerts = alerts_.Current();
  const bool visible = !alerts.empty();
  if (visible) {
    const auto &a = alerts.front();
    const auto colors = Theme(mode_);
    const auto text = (state_.replayed ? wxString("REPLAY / ") : state_.simulated ? wxString("DEMO / ") : wxString()) +
      wxString::FromUTF8(application::AlertLevelName(a.level)) + " / " + wxString::FromUTF8(a.title) +
      (a.acknowledged ? " / acknowledged" : "");
    if (alert_label_->GetLabel() != text) {
      alert_label_->SetLabel(text);
      alert_pane_->Layout();
    }
    alert_label_->SetForegroundColour(Colour(a.level == application::AlertLevel::Critical ? colors.alarm : colors.attention));
    alert_button_->SetLabel(wxString::Format("Alerts %u", static_cast<unsigned>(alerts.size())));
    alert_button_->SetRole(a.level==application::AlertLevel::Critical?ButtonRole::Critical:ButtonRole::Primary);
  }
  if (alert_pane_->IsShown() != visible) {
    alert_pane_->Show(visible);source_->Show(!visible);
    alert_pane_->GetParent()->Layout();
  }
}
void Shell::Tick() {
  const auto begin = std::chrono::steady_clock::now();
  const auto wall_now = vessel::Clock::now();
  const auto replay = actions_.commissioning
                          ? actions_.commissioning->ReadReplay(wall_now)
                          : std::optional<diagnostics::ReplayView>{};
  const auto now = replay ? replay->now : wall_now;
  if (replay)
    state_ = replay->state;
  #if XNAV_ENABLE_TEST_FIXTURES
  else if (simulation_)
    state_ = demo_.Read(now);
  #endif
  else {
    if (actions_.live_state)
      state_ = actions_.live_state();
    if (actions_.route)
      state_.navigation.route = actions_.route();
  }
  const auto config = replay ? actions_.commissioning->ReplayAssumptions()
                      : actions_.settings ? actions_.settings()
                                          : application::Settings{};
  #if XNAV_ENABLE_TEST_FIXTURES
  const auto model = simulation_ && !replay ? smartnav::PreviewEnergyModel(true)
                                            : config.energy.battery;
  const auto energy =
      simulation_ && !replay
          ? smartnav::PredictVesselEnergy(model, state_, now)
          : smartnav::PredictConfiguredEnergy(config.energy, state_, now);
  #else
  const auto model=config.energy.battery;
  const auto energy=smartnav::PredictConfiguredEnergy(config.energy,state_,now);
  #endif
  if (actions_.commissioning && !replay)
    actions_.commissioning->Capture(state_, wall_now);
  const bool creating = actions_.route_creating && actions_.route_creating();
  if (finish_route_->IsShown() != creating) {
    finish_route_->Show(creating);
    undo_route_->Show(creating);cancel_route_->Show(creating);
    finish_route_->GetParent()->Layout();
  }
  if (product_) {
    ProductState p;
    p.vessel = state_;
    p.now = now;
    if (replay)
      p.ais.source = "AIS not included in this recording";
    #if XNAV_ENABLE_TEST_FIXTURES
    else if (simulation_)
      p.ais = vessel::DemoAis(state_);
    #endif
    else if (actions_.navigation.ais)
      p.ais = actions_.navigation.ais(now);
    if (!simulation_ && !replay && actions_.navigation.anchor)
      p.anchor = actions_.navigation.anchor();
    else
      p.anchor.state = "Historical data / anchor controls unavailable";
    if (actions_.pilot_tick)
      p.pilot = actions_.pilot_tick(simulation_, wall_now);
    if (actions_.pilot_log && !replay)
      p.pilot_log = actions_.pilot_log(simulation_);
    if (actions_.pilot_sources && !replay && !simulation_)
      p.pilot_sources = actions_.pilot_sources();
    if (replay) {
      p.pilot = {};
      p.pilot.feedback.source = "Unavailable during REPLAY";
    }
    standby_->Enable(!replay && p.pilot.enabled && p.pilot.capabilities.standby);
    p.settings = config;
    if (actions_.settings_status)
      p.settings_status = actions_.settings_status();
    if (actions_.boat_bridge_status && !simulation_ && !replay)
      p.boat_bridge_status = actions_.boat_bridge_status();
    else p.boat_bridge_status = "Historical data / live boat mapping not applied";
    if (actions_.source_health && !replay)
      p.sources = actions_.source_health();
    if (actions_.radar && !replay)
      p.radar = actions_.radar();
    ais_state_ = p.ais;
    if (state_.simulated || state_.replayed) ais_selection_.Clear();
    else ais_selection_.Observe(p.ais, now);
    p.advice = smartnav::Advise(state_, energy, p.ais, now);
    alerts_.Observe({state_, p.ais, p.anchor, energy, p.pilot, now});
    p.alerts = alerts_.Current();
    UpdateAlerts();
    field_snapshot_ = {state_, config,
        simulation_ || replay ? std::vector<vessel::SourceHealth>{} : p.sources,
        energy, p.advice, p.pilot, p.radar, false, false, now};
    if(actions_.commissioning) {
      const auto r=actions_.commissioning->RecordingStatus();
      field_snapshot_.recording=r.active;
      field_snapshot_.recording_error=!r.error.empty();
    }
    field_snapshot_.alerts = p.alerts;
    field_journal_.Observe(field_snapshot_, wall_now);
    product_->Update(p, mode_);
  }
  UpdateRail(config.data_rail, now);
  clock_->SetLabel(simulation_ ? "10:42" : wxDateTime::Now().Format("%H:%M"));
  wxString label =
      replay ? wxString::Format("REPLAY / %s / %.0f s",
                                replay->paused  ? "PAUSED"
                                : replay->ended ? "ENDED"
                                                : "PLAYING",
                                replay->elapsed.count() / 1000.0)
      #if XNAV_ENABLE_TEST_FIXTURES
      : simulation_
          ? "DEMO / " + wxString(simulation_paused_
                                     ? "PAUSED"
                                     : wxString::FromUTF8(vessel::ScenarioName(
                                           demo_.Scenario())))
      #endif
          : InputSummary();
  if (actions_.commissioning) {
    const auto r = actions_.commissioning->RecordingStatus();
    if (r.active)
      label += " / REC";
    if (!r.error.empty())
      label += " / RECORD ERROR";
  }
  const auto source_color = Colour(
      replay || simulation_ ? Theme(mode_).attention : Theme(mode_).secondary);
  if (source_->GetForegroundColour() != source_color)
    source_->SetForegroundColour(source_color);
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
  const bool show_summary = true;
  const bool summary_layout = route_summary_->GetLabel() != summary ||
                              route_summary_->IsShown() != show_summary;
  route_summary_->SetLabel(summary);
  route_summary_->Show(show_summary);
  if (summary_layout)
    route_summary_->GetParent()->Layout();
  if (page_ && page_->IsShown())
    page_->Update(current_page_, mode_, state_, now, model, energy,
                  actions_.build_info ? actions_.build_info()
                                      : std::vector<std::string>{}, field_snapshot_.advice);
  UpdateScrollControls();
  if (actions_.diagnostic_snapshot)
    actions_.diagnostic_snapshot(state_, energy, PageTitle());
  metrics_.last_ms = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - begin)
                         .count();
  ++metrics_.ticks;
  metrics_.mean_ms += (metrics_.last_ms - metrics_.mean_ms) / metrics_.ticks;
  metrics_.maximum_ms = std::max(metrics_.maximum_ms, metrics_.last_ms);
}

XNavScroll *Shell::CurrentScroll() const {
  if (product_ && product_->IsShown()) return product_;
  if (page_ && page_->IsShown()) return page_;
  return nullptr;
}
int Shell::PageScrollPosition() const {
  auto *s = CurrentScroll();
  if (!s) return 0;
  int x, y, ux, uy;
  s->GetViewStart(&x, &y);
  s->GetScrollPixelsPerUnit(&ux, &uy);
  return y * uy;
}
bool Shell::CanScrollPage(int direction) const {
  auto *s = CurrentScroll();
  return s && s->CanScroll(direction);
}
const char *Shell::LightName() const {
  return mode_ == LightMode::Day ? "Day" : mode_ == LightMode::Dusk ? "Dusk" : "Night";
}
void Shell::UpdateScrollControls() {
  const bool scroll = CanScrollPage(-1) || CanScrollPage(1);
  auto *focus = wxWindow::FindFocus();
  if ((focus == page_up_ && !CanScrollPage(-1)) ||
      (focus == page_down_ && !CanScrollPage(1))) {
    // Focusing a panel normally delegates to a child and wxScrolledWindow
    // then scrolls that child into view. At an endpoint this can jump back
    // to the first action. Retain focus on the viewport itself before the
    // endpoint button is disabled, preserving both scrolling and shortcuts.
    if (auto *s = CurrentScroll()) s->SetFocusIgnoringChildren();
    else frame_.SetFocus();
  }
  bool changed = page_up_->IsShown() != scroll;
  for (auto *b : {page_up_, page_down_}) b->Show(scroll);
  page_up_->Enable(CanScrollPage(-1));
  page_down_->Enable(CanScrollPage(1));
  if (changed) page_up_->GetParent()->Layout();
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
#if XNAV_ENABLE_TEST_FIXTURES
  SelectDemo(vessel::DemoScenario::Cruise);
  if (actions_.demo_chart)
    actions_.demo_chart();
#endif
}
#if XNAV_ENABLE_TEST_FIXTURES
void Shell::SelectDemo(vessel::DemoScenario scenario) {
  if (actions_.commissioning) {
    actions_.commissioning->StopReplay();
    if (!simulation_)
      actions_.commissioning->StopRecording();
  }
  simulation_ = true;
  simulation_paused_ = false;
  demo_.Select(scenario, vessel::Clock::now());
  ApplyTheme();
  Tick();
}
#endif
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
  manager_
      .Update(); // Establish actual pane width before wrapping text/actions.
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
#if XNAV_ENABLE_TEST_FIXTURES
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
  auto *pause=new XNavButton(popup,wxID_ANY,simulation_paused_?"Resume simulation":"Pause simulation","CI-only source pause");
  pause->SetLightMode(mode_);
  pause->Bind(wxEVT_BUTTON,[this,popup](wxCommandEvent&){
    popup->Dismiss();popup->Destroy();simulation_paused_=!simulation_paused_;
    demo_.Pause(simulation_paused_,vessel::Clock::now());
  });
  layout->Add(pause,0,wxEXPAND|wxLEFT|wxRIGHT|wxBOTTOM,frame_.FromDIP(16));
  popup->SetSizerAndFit(layout);
  popup->Position(
      frame_.ClientToScreen(wxPoint(frame_.FromDIP(80), frame_.FromDIP(120))),
      wxSize());
  popup->Popup();
#endif
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
  if (!current) {
    const auto input = vessel::AssessText(state_.connectivity.status,
                                          vessel::Clock::now());
    if (input.quality == vessel::Quality::Live || input.quality == vessel::Quality::Aging)
      return "GPS unavailable / Marine input";
  }
  return current   ? "OpenCPN navigation"
         : present ? "Navigation stale"
                   : "No vessel input";
}

void Shell::ShowSystem() {
  // A normal center page keeps the alert/status slot and fixed manual controls
  // visible at every DPI; no transient popup can cover the Alerts action.
  ShowProduct(ProductPage::System);
}

void Shell::AfterCanvasLayoutChanged() {
  navigation_visibility_.clear();
  for(const auto &name : actions_.navigation_panes) {
    auto &pane=manager_.GetPane(name);
    if(pane.IsOk())pane.Show();
  }
  for(const auto &name : {"OpenNavTools","OpenNavData"}) {
    auto &pane=manager_.GetPane(name);if(pane.IsOk())pane.Show();
  }
  ShowNavigation();
}

void Shell::ShowChartContext(application::Coordinate position) {
  ShowNavigation();
  auto *popup=new SystemPopup(&frame_);
  popup->SetName("Chart position actions");
  popup->SetBackgroundColour(Colour(Theme(mode_).elevated));
  auto *layout=new wxBoxSizer(wxVERTICAL);
  auto *location=new wxStaticText(popup,wxID_ANY,
    wxString::Format("%.5f°   %.5f°",position.latitude_deg,position.longitude_deg));
  location->SetFont(UiFont(*popup,18));
  location->SetForegroundColour(Colour(Theme(mode_).primary));
  layout->Add(location,0,wxALL,frame_.FromDIP(16));
  auto *grid=new wxGridSizer(2,frame_.FromDIP(8),frame_.FromDIP(8));
  const auto result=[this](const application::CommandResult &r) {
    if(!r.ok)ConfirmSheet(frame_,mode_,"Unable to continue",wxString::FromUTF8(r.message),"Back");
  };
  const auto go=[this,position,result] {
    if(!actions_.navigation.go_to)return;
    if(ConfirmSheet(frame_,mode_,"Go to this position",
      wxString::Format("Destination %.5f°  %.5f°. Start navigating to this position?",position.latitude_deg,position.longitude_deg),"Start"))
      result(actions_.navigation.go_to(position,"Go To"));
  };
  const auto mark=[this,position,result] {
    auto f=EditSheet(frame_,mode_,"Create waypoint","Save this chart position.",{{"Name","Waypoint",128}},"Save");
    if(f && actions_.navigation.create_waypoint)result(actions_.navigation.create_waypoint(position,(*f)[0],""));
  };
  for(const auto &entry : std::vector<std::pair<wxString,std::function<void()>>>{
    {"Go To",go},{"Waypoint",mark},{"Measure",actions_.navigation.measure},
    {"Info",[this,position]{if(actions_.navigation.object_info_at)actions_.navigation.object_info_at(position);}}}) {
    auto *b=new XNavButton(popup,wxID_ANY,entry.first,entry.first);
    b->SetMinSize(frame_.FromDIP(wxSize(152,56)));b->SetLightMode(mode_);
    if(entry.first=="Go To")b->SetRole(ButtonRole::Primary);
    b->Bind(wxEVT_BUTTON,[popup,action=entry.second](wxCommandEvent&){popup->Dismiss();popup->Destroy();if(action)action();});
    grid->Add(b,1,wxEXPAND);
  }
  layout->Add(grid,0,wxLEFT|wxRIGHT|wxBOTTOM,frame_.FromDIP(16));
  popup->SetSizerAndFit(layout);
  popup->Position(frame_.ClientToScreen(frame_.FromDIP(wxPoint(80,80))),wxSize());
  popup->Popup();
}

} // namespace opennav::ui

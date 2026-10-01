#include "ui/Shell.h"

#include "smartnav/Advisories.h"
#include "vessel/DisplayItems.h"
#include "ui/Sheet.h"
#include "ui/PrototypeGeometry.h"
#include "diagnostics/TestUiTrace.h"
#include <wx/accel.h>
#include <wx/datetime.h>
#include <wx/dcbuffer.h>
#include <wx/dialog.h>
#include <wx/popupwin.h>
#include <wx/sizer.h>
#include <wx/textctrl.h>
#ifdef __WXMSW__
#include <windows.h>
#endif

#include <algorithm>
#include <utility>

namespace opennav::ui {
namespace {
bool CurrentMeasuredPosition(const vessel::VesselState &state, vessel::Time now) {
  const auto &latitude = state.navigation.latitude_deg;
  const auto &longitude = state.navigation.longitude_deg;
  const auto current = [now](const vessel::Sample &sample) {
    const auto a = vessel::Assess(sample, now);
    return sample.validity == vessel::Validity::Measured && a.value &&
        (a.quality == vessel::Quality::Live || a.quality == vessel::Quality::Aging);
  };
  return !state.simulated && !state.replayed && current(latitude) && current(longitude) &&
      latitude.observed_at == longitude.observed_at && !latitude.source.empty() &&
      latitude.source == longitude.source;
}
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
  original_pane_border_ = manager_.GetArtProvider()->GetMetric(wxAUI_DOCKART_PANE_BORDER_SIZE);
  original_sash_size_ = manager_.GetArtProvider()->GetMetric(wxAUI_DOCKART_SASH_SIZE);
  manager_.GetArtProvider()->SetMetric(wxAUI_DOCKART_PANE_BORDER_SIZE, 0);
  manager_.GetArtProvider()->SetMetric(wxAUI_DOCKART_SASH_SIZE, 0);
  const int gap = frame_.FromDIP(spacing::base);
  auto *top = MakePane("OpenNavTop", wxAuiPaneInfo().Top().Layer(10).BestSize(
                                         -1, frame_.FromDIP(prototype::top)));
  auto *row = new wxBoxSizer(wxHORIZONTAL);
  auto *brand = new wxPanel(top, wxID_ANY);
  brand_panel_ = brand;
  brand->SetMinSize(frame_.FromDIP(wxSize(180,68)));
  brand->SetBackgroundStyle(wxBG_STYLE_PAINT);
  brand->Bind(wxEVT_PAINT,[this,brand](wxPaintEvent &) {
    wxAutoBufferedPaintDC dc(brand);
    const auto c=Theme(mode_);
    dc.SetBackground(wxBrush(Colour(c.background))); dc.Clear();
    dc.SetDeviceOrigin(0, (brand->GetClientSize().y-brand->FromDIP(68))/2);
    // Original 32-unit prototype brand path; a design mark, not ownship data.
    const auto d=[brand](int x){return brand->FromDIP(x);};
    const wxPoint hull[]={{d(25),d(43)},{d(35),d(23)},{d(45),d(43)},{d(35),d(37)}};
    dc.SetBrush(wxBrush(Colour(c.accent)));dc.SetPen(*wxTRANSPARENT_PEN);dc.DrawPolygon(4,hull);
    dc.SetPen(wxPen(Colour(c.background),d(2)));dc.DrawLine(d(35),d(23),d(35),d(37));dc.DrawLine(d(35),d(37),d(45),d(43));
    dc.SetFont(UiFontWeight(*brand,23,650));dc.SetTextForeground(Colour(c.primary));
    dc.DrawText("opennav",d(58),d(20));
    const int x=d(58)+dc.GetTextExtent("opennav").x;
    dc.SetFont(UiFontWeight(*brand,23,350));dc.SetTextForeground(Colour(c.accent));dc.DrawText("x",x,d(20));
    dc.SetPen(wxPen(Colour(c.border)));dc.DrawLine(d(179),d(22),d(179),d(46));
  });
  brand->Bind(wxEVT_LEFT_UP,[this](wxMouseEvent&){ShowNavigation();});
  row->Add(brand,0,wxEXPAND);
  clock_ = Text(top, "", 15);
  clock_->SetMinSize(frame_.FromDIP(wxSize(56, 24)));

  source_ = Button(top,"No vessel input","Inspect source health",[this]{ShowHealth();});
  source_->SetTextSize(10);source_->SetRole(ButtonRole::Quiet);
  source_->SetMinSize(frame_.FromDIP(wxSize(120,44)));
  route_summary_ = new wxStaticText(top,wxID_ANY,"No active route",wxDefaultPosition,wxDefaultSize,wxST_ELLIPSIZE_END);
  route_summary_->SetFont(UiFont(*top,12)); route_summary_->SetMinSize(wxSize(0,-1)); labels_.push_back(route_summary_);
  row->Add(route_summary_, 1, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, gap * 2);
  row->Add(source_, 0, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, gap * 2);
  row->Add(clock_, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, gap);
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
  theme_button_->SetMinSize(frame_.FromDIP(wxSize(44, 44)));
  theme_button_->SetIconOnly();
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
  alert_button_ = Button(top, "Alerts", "Inspect active alerts", [this] { ShowProduct(ProductPage::Alerts); });
  alert_button_->SetMinSize(frame_.FromDIP(wxSize(44, 44)));
  alert_button_->SetIcon(XNavIcon::Bell); alert_button_->SetIconOnly();

  alert_pane_->SetSizer(alert_row);
  row->Add(alert_pane_,1,wxEXPAND);
  row->Add(theme_button_,0,wxALIGN_CENTER_VERTICAL|wxLEFT|wxRIGHT,frame_.FromDIP(4));
  row->Add(alert_button_,0,wxALIGN_CENTER_VERTICAL|wxLEFT|wxRIGHT,frame_.FromDIP(4));
  top->SetSizer(row);

  auto *left = MakePane("OpenNavTools", wxAuiPaneInfo().Left().Layer(5).BestSize(
      frame_.FromDIP(prototype::navigation), -1));
  auto *tools = new wxBoxSizer(wxVERTICAL);
  tools->AddSpacer(frame_.FromDIP(14));
  const auto nav = [&](const wxString &label, const wxString &name, XNavIcon icon, std::function<void()> action) {
    auto *b = Button(left,label,name,std::move(action)); b->SetNavigationItem(); b->SetIcon(icon);
    b->SetMinSize(frame_.FromDIP(wxSize(61,61)));
    tools->Add(b,0,wxLEFT|wxRIGHT,frame_.FromDIP(9));
    tools->AddSpacer(frame_.FromDIP(5));
    navigation_page_buttons_.push_back(b);
    return b;
  };
  nav("Chart", "Navigation", XNavIcon::Chart, [this]{ShowNavigation();});
  nav("Passage", "Route", XNavIcon::Route, [this]{ShowPassage();});
  nav("Traffic", "AIS targets", XNavIcon::Traffic, [this]{ShowProduct(ProductPage::Ais);});
  // The prototype separates passage/traffic from vessel views: 5px gap,
  // 7px margin, a 1px rule, 7px margin and another 5px gap.
  tools->AddSpacer(frame_.FromDIP(7));
  auto *nav_divider = new wxPanel(left, wxID_ANY);
  navigation_divider_ = nav_divider;
  // wxMSW otherwise exposes its default "panel" name as native window text.
  // A decorative separator has no label; keep the rail's action identity exact.
  nav_divider->SetLabel(wxEmptyString);
  nav_divider->SetMinSize(frame_.FromDIP(wxSize(37, 1)));
  nav_divider->SetBackgroundStyle(wxBG_STYLE_PAINT);
  nav_divider->Bind(wxEVT_PAINT, [this, nav_divider](wxPaintEvent &) {
    wxAutoBufferedPaintDC dc(nav_divider);
    dc.SetBackground(wxBrush(Colour(Theme(mode_).border)));
    dc.Clear();
  });
  tools->Add(nav_divider, 0, wxLEFT, frame_.FromDIP(21));
  tools->AddSpacer(frame_.FromDIP(12));
  nav("Energy", "Energy", XNavIcon::Energy, [this]{ShowPage(PreviewPage::Energy);});
  nav("Instruments", "Vessel instruments", XNavIcon::Instruments, [this]{ShowProduct(ProductPage::Instruments);});
  nav("Anchor", "Anchor watch", XNavIcon::Anchor, [this]{ShowProduct(ProductPage::Anchor);});
  nav("Radar", "Radar availability", XNavIcon::Radar, [this]{ShowProduct(ProductPage::Radar);});
  tools->AddStretchSpacer();
  nav("Settings", "Open navigation menu", XNavIcon::Settings, [this]{ShowProduct(ProductPage::Home);});
  vessel_profile_ = Button(left,"Vessel profile","Vessel profile / name not configured",[this] {
    ShowSettings();settings_drawer_->Select(SettingsSection::Vessel);
  });
  vessel_profile_->SetVesselProfile();
  vessel_profile_->SetMinSize(frame_.FromDIP(wxSize(32,32)));
  tools->AddSpacer(frame_.FromDIP(8));
  tools->Add(vessel_profile_,0,wxALIGN_CENTER_HORIZONTAL);
  tools->AddSpacer(frame_.FromDIP(14));
  left->SetSizer(tools);

  // Owned native surfaces float over the existing canvas. They never reparent
  // it, enter the saved AUI perspective or intercept chart input outside bounds.
  const auto overlay = [this](const char *name) {
    auto *p = new XNavFloatingSurface(frame_, name);
    chart_overlays_.push_back(p); return p;
  };
  chart_tools_ = overlay("OpenNav chart tools");
  auto *map_tools = new wxBoxSizer(wxHORIZONTAL);
  map_tools->AddSpacer(frame_.FromDIP(4));
  const auto map_button = [&](XNavIcon icon,const wxString &label,const wxString &name,std::function<void()> action) {
    auto *b=Button(chart_tools_,label,name,std::move(action)); b->SetIcon(icon); b->SetIconOnly(); b->SetFloating(); b->SetRole(ButtonRole::Quiet);
    b->SetMinSize(frame_.FromDIP(wxSize(44,44))); map_tools->Add(b,0,wxTOP|wxBOTTOM,frame_.FromDIP(4));
  };
  map_button(XNavIcon::Ruler,"Measure","Measure chart distance",actions_.navigation.measure);
  map_button(XNavIcon::Pin,"Waypoint","Waypoint at chart position",[this]{
    if(actions_.navigation.chart_position) if(auto point=actions_.navigation.chart_position()) ShowChartContext(*point);
  });
  map_tools->AddSpacer(frame_.FromDIP(5));
  map_button(XNavIcon::Plus,"+","Zoom chart in",actions_.zoom_in);
  map_button(XNavIcon::Minus,wxString::FromUTF8("−"),"Zoom chart out",actions_.zoom_out);
  map_tools->AddSpacer(frame_.FromDIP(4));
  chart_tools_->SetSizerAndFit(map_tools);
  chart_orientation_ = overlay("OpenNav chart orientation");
  auto *orientation = new wxBoxSizer(wxVERTICAL);
  orientation_button_ = Button(chart_orientation_, "North", "Change chart orientation", [this] {
    if (actions_.navigation.orientation) actions_.navigation.orientation();
    Tick();
  });
  orientation_button_->SetIcon(XNavIcon::Compass); orientation_button_->SetRole(ButtonRole::Quiet);
  orientation_button_->SetFloating();
  orientation_button_->SetMinSize(frame_.FromDIP(wxSize(68,90)));
  orientation->Add(orientation_button_,1,wxEXPAND); chart_orientation_->SetSizerAndFit(orientation);
  chart_follow_ = overlay("OpenNav follow boat");
  auto *following=new wxBoxSizer(wxHORIZONTAL);
  auto *center=Button(chart_follow_,"Follow boat","Center chart on boat and follow position",actions_.follow);
  center->SetMinSize(frame_.FromDIP(wxSize(142,44)));center->SetRole(ButtonRole::Quiet);
  center->SetFloating();
  center->SetIcon(XNavIcon::Ownship);center->SetInlineIcon();
  following->Add(center,1,wxEXPAND);chart_follow_->SetSizerAndFit(following);

  auto *right =
      MakePane("OpenNavData", wxAuiPaneInfo().Right().Layer(5).BestSize(
                                  frame_.FromDIP(prototype::rail), -1));
  rail_scroll_ = new XNavDataRail(right);
  auto *rail_container = new wxBoxSizer(wxVERTICAL);
  auto *rail_header=new wxPanel(right,wxID_ANY);
  rail_header_ = rail_header;
  rail_header->SetMinSize(frame_.FromDIP(wxSize(186,42)));
  auto *rail_heading=new wxBoxSizer(wxHORIZONTAL);
  auto *rail_title=Text(rail_header,"AT A GLANCE",9);
  rail_heading->Add(rail_title,1,wxALIGN_CENTER_VERTICAL|wxLEFT,frame_.FromDIP(18));
  auto *configure_rail=Button(rail_header,"Configure instruments","Choose the four rail values",[this]{ShowProduct(ProductPage::RailLayout);});
  rail_configure_ = configure_rail;
  configure_rail->SetIcon(XNavIcon::Sliders);configure_rail->SetIconOnly();configure_rail->SetRole(ButtonRole::Quiet);
  configure_rail->SetMinSize(frame_.FromDIP(wxSize(40,40)));
  rail_heading->Add(configure_rail,0,wxALIGN_CENTER_VERTICAL|wxRIGHT,frame_.FromDIP(6));rail_header->SetSizer(rail_heading);
  rail_container->Add(rail_header,0,wxEXPAND);
  rail_container->Add(rail_scroll_, 1, wxEXPAND);
  right->SetSizer(rail_container);

  auto *bottom = MakePane("OpenNavActions", wxAuiPaneInfo().Bottom().Layer(10).BestSize(
      -1,frame_.FromDIP(prototype::footer)));
  // A fixed AUI dock adds a stretchable trailing spacer. It takes one pixel
  // from this otherwise full-width status bar. A proportional dock fills the
  // width exactly; the XNav sash metric is zero and restored for Legacy.
  manager_.GetPane(bottom).DockFixed(false);
  auto *status = new wxBoxSizer(wxVERTICAL);
  footer_=new XNavStatusFooter(bottom,[this]{ShowHealth();});
  status->Add(footer_,1,wxEXPAND);bottom->SetSizer(status);
  auto *horizon_pane=MakePane("OpenNavHorizon",wxAuiPaneInfo().Bottom().Layer(1).BestSize(-1,frame_.FromDIP(prototype::horizon)));
  auto *horizon_layout=new wxBoxSizer(wxVERTICAL);
  horizon_=new XNavHorizon(horizon_pane,[this]{ShowPassage();},
      [this](const application::HorizonAction &action){ActivateHorizon(action);});
  horizon_layout->Add(horizon_,1,wxEXPAND);
  route_actions_=new wxPanel(horizon_pane,wxID_ANY);route_actions_->Hide();
  auto *actions_row = new wxBoxSizer(wxHORIZONTAL);
  auto *route_host=route_actions_;
  finish_route_ = Button(route_host, "Done", "Name and save this route", [this] {
    const auto fields=EditSheet(frame_,mode_,"Save route",
      "Name this route. You can activate it after saving.",
      {{"Name","",128},{"Description","",2048}},"Save route");
    if(!fields || !actions_.navigation.finish_route_named) return;
    const auto result=actions_.navigation.finish_route_named((*fields)[0],(*fields)[1]);
    if(result.ok) { Tick(); ShowObject(result.identity,true); }
    else ConfirmSheet(frame_,mode_,"Route not saved",wxString::FromUTF8(result.message),"Back");
  });
  undo_route_=Button(route_host,"Undo","Undo last route point",[this]{
    if (!actions_.navigation.undo_route_point) return;
    const auto result = actions_.navigation.undo_route_point();
    if (!result.ok)
      ConfirmSheet(frame_, mode_, "Cannot undo route point", wxString::FromUTF8(result.message), "Back");
    Tick();
  });
  undo_route_->Enable(false);
  cancel_route_=Button(route_host,"Cancel","Cancel route creation",[this]{
    if(actions_.navigation.cancel_route && ConfirmSheet(frame_,mode_,"Cancel route?","Discard this unfinished route? Existing routes are preserved.","Discard route"))actions_.navigation.cancel_route();
  });
  for (auto *button : {cancel_route_, undo_route_, finish_route_}) {
    button->SetMinSize(frame_.FromDIP(wxSize(88, 48)));
    button->SetRole(button == finish_route_ ? ButtonRole::Primary : ButtonRole::Quiet);
    actions_row->Add(button, 0, wxALL, frame_.FromDIP(4));
    button->Hide();
  }
  route_actions_->SetSizer(actions_row);horizon_layout->Add(route_actions_,0,wxEXPAND);
  horizon_pane->SetSizer(horizon_layout);
  pilot_summary_ = Button(right,"Autopilot","Open autopilot controls",[this]{ShowProduct(ProductPage::Pilot);});
  pilot_summary_->SetMinSize(frame_.FromDIP(wxSize(149,87)));
  pilot_summary_->SetSummary("Unavailable","No current feedback");
  auto *pilot_row = new wxBoxSizer(wxHORIZONTAL);
  pilot_row->AddSpacer(frame_.FromDIP(19));
  pilot_row->Add(pilot_summary_,1,wxEXPAND);
  pilot_row->AddSpacer(frame_.FromDIP(18));
  rail_container->AddSpacer(frame_.FromDIP(13));
  rail_container->Add(pilot_row,0,wxEXPAND);
  rail_container->AddSpacer(frame_.FromDIP(13));
  // The existing guarded standby action remains available in the pilot panel.
  // Keep its state sink hidden until the expanded pilot workflow owns it.
  standby_=Button(right,"STBY","Manual STANDBY / requires enabled control",[this]{
    if(actions_.pilot_command&&!state_.replayed)actions_.pilot_command(simulation_,adapters::PilotAction::Standby,0);
  });
  standby_->Hide();standby_->Disable();
  page_ = new PreviewPanel(&frame_);
  page_->SetCloseAction([this]{ShowNavigation();});
  // An unmanaged overlay is reordered behind ChartCanvas by the native AUI
  // resize path. Use the same layout manager for the alternate center page.
  manager_.AddPane(page_, wxAuiPaneInfo()
                              .Name("OpenNavPage")
                              .CenterPane()
                              .PaneBorder(false)
                              .Hide());
  ProductActions product_actions;
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
  product_actions.chart_style_status = actions_.chart_style_status;
  product_actions.chart_style_requested = actions_.chart_style_requested;
  product_actions.set_chart_style = actions_.set_chart_style;
  product_actions.theme = [this](LightMode mode) { SetLight(mode); };
  product_actions.save_settings = actions_.save_settings;
  product_actions.chart = [this] { ShowNavigation(); };
  product_actions.preferences = [this] { ShowSettings(); };
  product_actions.source_health = [this] { ShowHealth(); };
  product_actions.page_changed = [this](ProductPage page) {
    auto &pane = manager_.GetPane("OpenNavHorizon");
    if (pane.IsOk()) {
      pane.Show(page == ProductPage::Instruments);
      manager_.Update();
    }
  };
  product_actions.route_summary = [this] { ShowPassage(); };
  product_actions.anchor_watch = [this] { ShowAnchor(); };
  product_actions.pilot_controls = [this] { ShowPilot(); };
  product_actions.alerts = [this] { ShowAlerts(); };
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
  pilot_actions_.command = product_actions.pilot_command;
  pilot_actions_.enable = product_actions.pilot_enable;
  pilot_actions_.settings = [this] { ShowProduct(ProductPage::PilotSettings); };
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
      {'R', [this] { ShowPassage(); }},
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
  context_lifetime_.reset();
  CloseContext();
  if (ais_drawer_) { ais_drawer_->Dismiss(); ais_drawer_->Destroy(); ais_drawer_ = nullptr; }
  if (passage_drawer_) { passage_drawer_->Dismiss(); passage_drawer_->Destroy(); passage_drawer_ = nullptr; }
  if (settings_drawer_) { settings_drawer_->Dismiss(); settings_drawer_->Destroy(); settings_drawer_ = nullptr; }
  if (anchor_drawer_) { anchor_drawer_->Dismiss(); anchor_drawer_->Destroy(); anchor_drawer_ = nullptr; }
  if (alert_drawer_) { alert_drawer_->Dismiss(); alert_drawer_->Destroy(); alert_drawer_ = nullptr; }
  if (health_drawer_) { health_drawer_->Dismiss(); health_drawer_->Destroy(); health_drawer_ = nullptr; }
  if (pilot_drawer_) { pilot_drawer_->Dismiss(); pilot_drawer_->Destroy(); pilot_drawer_ = nullptr; }
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
  manager_.GetArtProvider()->SetMetric(wxAUI_DOCKART_PANE_BORDER_SIZE, original_pane_border_);
  manager_.GetArtProvider()->SetMetric(wxAUI_DOCKART_SASH_SIZE, original_sash_size_);
  manager_.Update();
  // ShowNavigation above still updates orientation and chart placement. Keep
  // overlay children alive until that restoration is finished (MSW destroys
  // child windows immediately, unlike GTK deferred deletion).
  for (auto *overlay : chart_overlays_) overlay->Destroy();
  chart_overlays_.clear();
}

bool Shell::OwnsPane(const wxWindow *window) const {
  return window && (window == page_ || window == product_ ||
      std::find(panes_.begin(), panes_.end(), window) != panes_.end());
}

bool Shell::LoadPersistentPerspective(const wxString &perspective) {
  // wxAUI hides/docks every managed pane before loading a saved perspective.
  // Our temporary panes deliberately never enter OpenCPN's saved workspace.
  // Preserve only those owned objects; upstream still restores every chart and
  // plugin pane using its original parser and normal configuration semantics.
  std::vector<wxAuiPaneInfo> transient;
  const auto &panes = manager_.GetAllPanes();
  for (std::size_t i = 0; i < panes.GetCount(); ++i)
    if (OwnsPane(panes[i].window)) transient.push_back(panes[i]);
  const bool loaded = manager_.LoadPerspective(perspective, false);
  for (const auto &saved : transient) {
    auto &pane = manager_.GetPane(saved.window);
    if (pane.IsOk()) pane.SafeSet(saved);
  }
  // A settings/locale reload can happen while a product page covers the chart.
  // Restore the current page's chart visibility, not stale saved visibility.
  for (const auto &saved : navigation_visibility_) {
    auto &pane = manager_.GetPane(saved.first);
    if (pane.IsOk()) pane.Hide();
  }
  return loaded;  // The existing upstream Update/Notify follows this call.
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
  std::vector<wxWindow *> visited;
  // Copy native bounds only. This diagnostic walk cannot invoke actions or
  // retain widget lifetimes in a consumer. Modal sheets and transient chart
  // cards are owned children too, even when they have a separate native HWND.
  const auto collect = [&](const auto &self, wxWindow *window) -> void {
    if (std::find(visited.begin(), visited.end(), window) != visited.end()) return;
    visited.push_back(window);
    const bool field = dynamic_cast<wxTextCtrl *>(window) != nullptr && window->IsShownOnScreen();
    if (dynamic_cast<XNavButton *>(window) || dynamic_cast<XNavRange *>(window) || field) {
      const auto rectangle = window->GetScreenRect();
      bool visible = window->IsShownOnScreen();
      for (auto *parent = window->GetParent(); parent && !parent->IsTopLevel();
           parent = parent->GetParent())
        visible = visible && parent->GetScreenRect().Contains(rectangle);
      const auto label = field ? "Field: " + window->GetName() : window->GetLabel();
      result.push_back({label.ToStdString(wxConvUTF8), rectangle,
                        window->IsEnabled(), visible, window->GetName().ToStdString(wxConvUTF8)});
    }
    for (auto *child : window->GetChildren()) self(self, child);
  };
  collect(collect, &frame_);
  for (auto *window : wxTopLevelWindows) {
    for (auto *parent = window->GetParent(); parent; parent = parent->GetParent()) {
      if (parent == &frame_) { collect(collect, window); break; }
    }
  }
  return result;
}

bool Shell::HasTransientSurface() const {
  if (DrawerRegion() || (context_ && context_->IsShownOnScreen())) return true;
  for (auto *window : wxTopLevelWindows) {
    auto *dialog = dynamic_cast<wxDialog *>(window);
    if (!dialog || !dialog->IsModal()) continue;
    for (auto *owner = dialog->GetParent(); owner; owner = owner->GetParent())
      if (owner == &frame_) return true;
  }
  return false;
}

void Shell::UpdateState(const vessel::VesselState &state) {
  if (!simulation_ &&
      (!actions_.commissioning || !actions_.commissioning->Replaying()))
    state_ = state;
}

void Shell::ApplyTheme() {
  const auto colors = Theme(mode_);
  caption_themed_ = ThemeWindowChrome(frame_, mode_);
  rail_scroll_->SetBackgroundColour(Colour(colors.background));
  alert_pane_->SetBackgroundColour(Colour(colors.surface));
  theme_button_->SetLabel(LightName());
  theme_button_->SetIcon(mode_ == LightMode::Day ? XNavIcon::Sun : mode_ == LightMode::Dusk ? XNavIcon::Dusk : XNavIcon::Moon);
  for (auto *overlay : chart_overlays_) overlay->SetBackgroundColour(Colour(FloatingTheme(mode_).surface));
  for (auto *pane : panes_) {
    pane->SetBackgroundColour(Colour(colors.background));
    for (auto *child : pane->GetChildren())
      if (dynamic_cast<wxPanel *>(child))
        child->SetBackgroundColour(Colour(colors.background));
    pane->Refresh();
  }
  for (auto *label : labels_)
    label->SetForegroundColour(Colour(colors.secondary));
  for (auto *button : buttons_)
    button->SetLightMode(mode_);
  for (const auto &value : rail_values_)
    value.second->SetLightMode(mode_);
  source_->SetTextColor(simulation_ ? colors.attention : colors.secondary);
  footer_->Update(footer_->View(),mode_);
}

void Shell::SetLight(LightMode mode) {
  mode_ = mode;
  if (actions_.theme)
    actions_.theme(mode);
  // The pinned SetAndApplyColorScheme resets the AUI border and sash metrics. Keep this
  // transient XNav presentation after each scheme change; destruction restores
  // the original metric before Legacy's perspective is saved/restored.
  manager_.GetArtProvider()->SetMetric(wxAUI_DOCKART_PANE_BORDER_SIZE, 0);
  manager_.GetArtProvider()->SetMetric(wxAUI_DOCKART_SASH_SIZE, 0);
  manager_.Update();
  ApplyTheme();
  PlaceChartControls();
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
          const int decimals = key == "heading" || key == "cog" || key == "awa" ||
              key == "twa" || key == "rpm" || key == "soc" || key == "pressure" ||
              key == "fresh_water" || key == "fuel" || key == "waste" ? 0 : 1;
          auto *value =
              new XNavDataValue(rail_scroll_, wxString::FromUTF8(item.title),
                                wxString::FromUTF8(item.unit), decimals);
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
  if (!visible) {
    alert_button_->SetLabel("Alerts");
    alert_button_->SetRole(ButtonRole::Quiet);
  }
  if (alert_pane_->IsShown() != visible) {
    alert_pane_->Show(visible);source_->Show(!visible);
    alert_pane_->GetParent()->Layout();
  }
}
void Shell::ApplyResponsiveLayout() {
  const auto size = frame_.ToDIP(frame_.GetClientSize());
  const int layout_class = (size.x <= 1100 ? 1 : 0) |
      (size.x > 760 && size.y <= 740 ? 2 : 0) |
      (size.x > 760 && size.y <= 600 ? 4 : 0);
  const int dpi = frame_.GetDPI().x;
  if (layout_class == responsive_class_ && dpi == responsive_dpi_) return;
  responsive_class_ = layout_class; responsive_dpi_ = dpi;
  const auto layout = prototype::Desktop(size.x, size.y);
  const auto dip = [this](int v) { return frame_.FromDIP(v); };
  const auto pane_size = [&](const char *name, int width, int height) {
    auto &pane = manager_.GetPane(name);
    if (!pane.IsOk()) return;
    const wxSize wanted(width < 0 ? -1 : dip(width), height < 0 ? -1 : dip(height));
    pane.BestSize(wanted).MinSize(wanted);
    pane.window->SetMinSize(wanted);
  };
  brand_panel_->SetMinSize(wxSize(dip(180), dip(layout.top)));
  pane_size("OpenNavTop", -1, layout.top);
  pane_size("OpenNavTools", layout.navigation, -1);
  pane_size("OpenNavData", layout.rail, -1);
  pane_size("OpenNavHorizon", -1, layout.horizon);
  auto *left = navigation_divider_->GetParent();
  auto *tools = left->GetSizer();
  tools->Clear(false);
  tools->AddSpacer(dip(layout.nav_inset));
  navigation_divider_->SetMinSize(wxSize(dip(layout.navigation-43), dip(1)));
  for (std::size_t i = 0; i < navigation_page_buttons_.size(); ++i) {
    if (i == 7) tools->AddStretchSpacer();
    auto *button = navigation_page_buttons_[i];
    button->SetMinSize(wxSize(dip(layout.navigation-19), dip(layout.nav_height)));
    tools->Add(button, 0, wxLEFT | wxRIGHT, dip(9));
    tools->AddSpacer(dip(layout.nav_gap));
    if (i == 2) {
      tools->AddSpacer(dip(layout.divider_before));
      tools->Add(navigation_divider_, 0, wxLEFT, dip(21));
      tools->AddSpacer(dip(layout.divider_after));
    }
  }
  const bool show_profile=layout.nav_height!=43;
  vessel_profile_->Show(show_profile);
  if(show_profile) {
    vessel_profile_->SetMinSize(wxSize(dip(32),dip(32)));
    tools->AddSpacer(dip(8));
    tools->Add(vessel_profile_,0,wxALIGN_CENTER_HORIZONTAL);
    tools->AddSpacer(dip(layout.nav_inset));
  } else tools->AddSpacer(dip(layout.nav_inset-layout.nav_gap));
  rail_header_->SetMinSize(wxSize(dip(layout.rail), dip(layout.rail_header)));
  rail_configure_->SetMinSize(wxSize(dip(layout.rail == 156 ? 30 : 40),
                                    dip(layout.rail == 156 ? 30 : 40)));
  rail_header_->GetSizer()->GetItem(std::size_t(0))->SetBorder(dip(layout.rail == 156 ? 13 : 18));
  auto *container = rail_header_->GetParent()->GetSizer();
  container->GetItem(std::size_t(2))->AssignSpacer(0, dip(layout.pilot_gap));
  auto *pilot_row = container->GetItem(std::size_t(3))->GetSizer();
  const int inset = layout.rail == 156 ? 14 : 19;
  pilot_row->GetItem(std::size_t(0))->AssignSpacer(dip(inset), 0);
  pilot_row->GetItem(std::size_t(2))->AssignSpacer(dip(inset-1), 0);
  pilot_summary_->SetMinSize(wxSize(dip(layout.rail-inset*2+1), dip(layout.pilot_height)));
  left->Layout();rail_header_->GetParent()->Layout();
  // wxAUI 3.2.8 resets DockFixed sizes from pane best/min sizes in LayoutAll.
  // Only XNav-owned panes change. No detach, perspective rewrite, chart model
  // mutation or navigation processing is needed for a display-size change.
  manager_.Update();
  brand_panel_->Refresh(false);
}

void Shell::Tick() {
  XNAV_TEST_UI_TRACE("tick.begin", metrics_.ticks, timer_.IsRunning());
  const auto begin = std::chrono::steady_clock::now();
  ApplyResponsiveLayout();
  const auto wall_now = vessel::Clock::now();
  if (actions_.chart_orientation) {
    const auto orientation = wxString::FromUTF8(actions_.chart_orientation());
    orientation_button_->SetLabel(orientation);
    orientation_button_->SetName("Chart orientation: " + orientation + " up");
    if (actions_.chart_rotation) orientation_button_->SetCompassRotation(actions_.chart_rotation());
  }
  const auto replay = actions_.commissioning
                          ? actions_.commissioning->ReadReplay(wall_now)
                          : std::optional<diagnostics::ReplayView>{};
  const auto now = replay ? replay->now : wall_now;
  // Observe bounds on the application thread. Reading or opening a sheet
  // cannot rejuvenate the provider's retained target observations.
  if (actions_.online_ais_tick) actions_.online_ais_tick(!replay && !simulation_);
  online_ais_state_ = actions_.online_ais.read
      ? actions_.online_ais.read(wall_now) : application::OnlineAisState{};
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
  XNAV_TEST_UI_TRACE("tick.input", metrics_.ticks);
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
  XNAV_TEST_UI_TRACE("tick.energy-recording", metrics_.ticks);
  const bool creating = actions_.route_creating && actions_.route_creating();
  undo_route_->Enable(creating && !state_.simulated && !state_.replayed &&
      actions_.navigation.undo_route_point && actions_.navigation.can_undo_route_point &&
      actions_.navigation.can_undo_route_point());
  if (finish_route_->IsShown() != creating) {
    finish_route_->Show(creating);
    undo_route_->Show(creating);cancel_route_->Show(creating);
    route_actions_->Show(creating);
    route_actions_->GetParent()->Layout();
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
    horizon_ais_=p.ais;
    if (!simulation_ && !replay && actions_.navigation.anchor)
      p.anchor = actions_.navigation.anchor();
    else
      p.anchor.state = "Historical data / anchor controls unavailable";
    if(anchor_drawer_&&anchor_drawer_->IsShown()) {
      anchor_drawer_->Update(p.anchor,p.vessel,now,mode_);
      anchor_drawer_->Present(DrawerWorkspace());
    }
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
    wxString pilot_mode="Unavailable", pilot_detail="No current feedback";
    if(p.pilot.fresh && p.pilot.feedback.mode!=adapters::PilotMode::Unavailable) {
      pilot_mode=wxString::FromUTF8(adapters::PilotModeName(p.pilot.feedback.mode));
      pilot_mode=pilot_mode.Left(1)+pilot_mode.Mid(1).Lower();
      pilot_detail=p.pilot.feedback.mode==adapters::PilotMode::Standby
          ? "You have the helm" : p.pilot.enabled ? "Manual control enabled" : "Display only";
    } else if(p.pilot.feedback.observed_at!=vessel::Time{}) {
      pilot_mode="Stale";pilot_detail="Check pilot connection";
    }
    pilot_summary_->SetSummary(pilot_mode,pilot_detail);
    p.settings = config;
    if(actions_.vessel_name)p.vessel_name=actions_.vessel_name();
    if(actions_.chart_safety_depth_m)p.chart_safety_depth_m=actions_.chart_safety_depth_m();
    if (pilot_drawer_ && pilot_drawer_->IsShown()) {
      pilot_drawer_->Update(p.pilot,wall_now,p.vessel,now,config.pilot.permit_control,mode_);
      pilot_drawer_->Present(DrawerWorkspace());
    }
    if (actions_.settings_status)
      p.settings_status = actions_.settings_status();
    if (actions_.boat_bridge_status && !simulation_ && !replay)
      p.boat_bridge_status = actions_.boat_bridge_status();
    else p.boat_bridge_status = "Historical data / live boat mapping not applied";
    if (actions_.source_health && !replay)
      p.sources = actions_.source_health();
    if (actions_.radar && !replay)
      p.radar = actions_.radar();
    if (settings_drawer_ && settings_drawer_->IsShown()) {
      settings_drawer_->Update(p, mode_);
      settings_drawer_->Present(DrawerWorkspace());
    }
    const auto health=application::PresentSourceHealth(p.vessel,p.sources,p.ais,
        online_ais_state_,p.pilot,now);
    footer_->Update(application::PresentFooter(p.vessel,p.anchor,health,now),mode_);
    if (health_drawer_ && health_drawer_->IsShown()) {
      health_drawer_->Update(health,mode_);
      health_drawer_->Present(DrawerWorkspace());
    }
    // Online traffic is display-only. SmartNav, alarms and receiver health
    // below continue to consume OpenCPN's original onboard state.
    ais_state_ = !simulation_ && !replay
        ? ais::Aggregate(p.ais, online_ais_state_.feed).display : p.ais;
    if (state_.simulated || state_.replayed) ais_selection_.Clear();
    else ais_selection_.Observe(ais_state_, now);
    p.advice = smartnav::Advise(state_, energy, p.ais, now);
    alerts_.Observe({state_, p.ais, p.anchor, energy, p.pilot, now});
    p.alerts = alerts_.Current();
    UpdateAlerts();
    if(alert_drawer_ && alert_drawer_->IsShown()) {
      alert_drawer_->Update(p.alerts,state_.replayed,mode_);
      alert_drawer_->Present(DrawerWorkspace());
    }
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
    XNAV_TEST_UI_TRACE("tick.services", metrics_.ticks);
    product_->Update(p, mode_);
  }
  XNAV_TEST_UI_TRACE("tick.product", metrics_.ticks);
  if (ais_drawer_ && ais_drawer_->IsShown()) {
    ais_drawer_->Update(ais_state_, online_ais_state_, now, mode_);
    ais_drawer_->Present(DrawerWorkspace());
  }
  if (passage_drawer_ && passage_drawer_->IsShown()) {
    passage_drawer_->Update(state_, field_snapshot_.advice, energy, now, mode_);
    passage_drawer_->Present(DrawerWorkspace());
  }
  UpdateRail(config.data_rail, now);
  horizon_->Update(application::PresentHorizon(state_,field_snapshot_.advice,horizon_ais_,now),mode_);
  PlaceChartControls();
  UpdateContext(wall_now);
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
  source_->SetTextColor(replay || simulation_ ? Theme(mode_).attention : Theme(mode_).secondary);
  if (source_->GetLabel() != label) {
    source_->SetLabel(label);
    source_->GetParent()->Layout();
  }
  const auto route = state_.navigation.route;
  const auto distance =
      route ? vessel::AssessRoute(*route, now).remaining_distance_nm
            : std::nullopt;
  wxString summary = "Route unavailable";
  if (distance) {
    const auto destination = !route->remaining_steps.empty()
        ? route->remaining_steps.back().name : route->route_name;
    summary = wxString::Format("%.1f NM", *distance) + "  /  " +
        (destination.empty() ? wxString("Destination") : wxString::FromUTF8(destination));
  } else if (route && route->state == vessel::RouteState::NoActiveRoute) {
    summary = "No active route";
  }
  if (creating) summary = "Build route / tap chart to add points";
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
  const auto current_title=PageTitle();
  for(auto *button:navigation_page_buttons_) {
    const auto label=button->GetLabel();
    const bool selected=(label=="Chart"&&current_title=="Navigation") ||
      (label=="Passage"&&current_title=="Route") ||
      (label=="Traffic"&&(current_title=="AIS targets"||current_title=="AIS target")) ||
      (label=="Energy"&&current_title=="Energy") ||
      (label=="Instruments"&&current_title=="Vessel instruments") ||
      (label=="Anchor"&&current_title=="Anchor watch") ||
      (label=="Radar"&&current_title=="Radar status") ||
      (label=="Settings"&&(current_title=="Settings"||current_title=="Menu"));
    button->SetSelected(selected);
  }
  XNAV_TEST_UI_TRACE("tick.before-publication", metrics_.ticks);
  if (actions_.diagnostic_snapshot)
    actions_.diagnostic_snapshot(state_, energy, PageTitle());
  XNAV_TEST_UI_TRACE("tick.published", metrics_.ticks);
  metrics_.last_ms = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - begin)
                         .count();
  ++metrics_.ticks;
  metrics_.mean_ms += (metrics_.last_ms - metrics_.mean_ms) / metrics_.ticks;
  metrics_.maximum_ms = std::max(metrics_.maximum_ms, metrics_.last_ms);
  XNAV_TEST_UI_TRACE("tick.end", metrics_.ticks);
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
  // The prototype Instruments view scrolls directly by wheel/touch. Keep the
  // earlier explicit buttons for pages which have not yet been migrated.
  const bool prototype_instruments = product_ && product_->IsShown() &&
      product_->PageTitle() == "Vessel instruments";
  const bool scroll = !prototype_instruments &&
      (CanScrollPage(-1) || CanScrollPage(1));
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
  if (health_drawer_ && health_drawer_->IsShown()) return "Source health";
  if (alert_drawer_ && alert_drawer_->IsShown()) return "Alerts";
  if (pilot_drawer_ && pilot_drawer_->IsShown()) return "Manual autopilot";
  if (anchor_drawer_ && anchor_drawer_->IsShown()) return "Anchor watch";
  if (settings_drawer_ && settings_drawer_->IsShown()) return "Settings";
  if (passage_drawer_ && passage_drawer_->IsShown()) return "Route";
  if (ais_drawer_ && ais_drawer_->IsShown()) return ais_drawer_->PageTitle();
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
  CloseContext();
  if (health_drawer_) health_drawer_->Dismiss();
  if (alert_drawer_) alert_drawer_->Dismiss();
  if (pilot_drawer_) pilot_drawer_->Dismiss();
  if (anchor_drawer_) anchor_drawer_->Dismiss();
  if (settings_drawer_) settings_drawer_->Dismiss();
  if (passage_drawer_) passage_drawer_->Dismiss();
  if (ais_drawer_) ais_drawer_->Dismiss();
  // Re-entering the already-visible chart needs no pane layout. A needless
  // canvas resize schedules OpenCPN's delayed frame-focus recapture and also
  // redraws the chart below the newly opened context card.
  const bool layout = !navigation_visibility_.empty() || manager_.GetPane(page_).IsShown() ||
      (product_ && manager_.GetPane(product_).IsShown());
  if (!layout) return;
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
  PlaceChartControls();
  frame_.Refresh();
}
void Shell::ShowProduct(ProductPage page) {
  if (page == ProductPage::SourceHealth) { ShowHealth(); return; }
  if (page == ProductPage::Home || page == ProductPage::Settings) { ShowSettings(); return; }
  if (page == ProductPage::Ais) { ShowTraffic(); return; }
  if (page == ProductPage::Anchor) { ShowAnchor(); return; }
  if (page == ProductPage::Pilot) { ShowPilot(); return; }
  if (page == ProductPage::Alerts) { ShowAlerts(); return; }
  if (ais_drawer_) ais_drawer_->Dismiss();
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
  if (route) {
    ShowProduct(ProductPage::Routes);
    product_->ShowObject(id, true, mode_);
    Tick();
    return;
  }
  ShowNavigation();
  context_waypoint_ = id;
  const std::weak_ptr<int> lifetime = context_lifetime_;
  context_ = new XNavContextCard(frame_, ContextKind::Waypoint,
      [this, lifetime, id](ContextAction action, std::optional<application::Waypoint> point) {
    if (lifetime.expired()) return;
    if (action == ContextAction::Details) {
      ShowProduct(ProductPage::Waypoints);
      product_->ShowObject(id, false, mode_); Tick();
      return;
    }
    if (!point || state_.simulated || state_.replayed) return;
    const auto result = WaypointSheet(frame_, mode_, action, *point, actions_.navigation);
    if (result && !result->ok)
      ConfirmSheet(frame_, mode_, "Unable to continue", wxString::FromUTF8(result->message), "Back");
    if (result && result->ok && (action == ContextAction::Remove || action == ContextAction::GoTo))
      ShowNavigation();
    else ShowObject(id, false);
  });
  UpdateContext(vessel::Clock::now());
  if (context_) { context_->Show(); context_->Raise(); UpdateContext(vessel::Clock::now()); }
}
void Shell::ShowAis(int mmsi) {
  ShowTraffic(mmsi);
}
wxRect Shell::DrawerWorkspace() const {
  const auto size = frame_.GetClientSize();
  const auto logical=frame_.ToDIP(size);
  const auto layout=prototype::Desktop(logical.x,logical.y);
  const int left = frame_.FromDIP(layout.navigation), top = frame_.FromDIP(layout.top);
  return {frame_.ClientToScreen({left, top}),
      wxSize(size.x-left-frame_.FromDIP(layout.rail), size.y-top-frame_.FromDIP(prototype::footer))};
}
void Shell::ShowTraffic(int mmsi) {
  ShowNavigation();
  if (!ais_drawer_) {
    const std::weak_ptr<int> lifetime = context_lifetime_;
    ais_drawer_ = new XNavAisDrawer(frame_, actions_.online_ais, [this, lifetime](int id) {
      if (lifetime.expired()) return;
      application::CommandResult result{false, "Target position unavailable or stale"};
      if (!state_.simulated && !state_.replayed &&
          ais_selection_.Select(id, ais_state_, vessel::Clock::now())) {
        for (const auto &target : ais_state_.targets) if (target.mmsi == id) {
          const auto &action = target.origin == vessel::AisOrigin::AisStreamOnline
              ? actions_.view_online_ais : actions_.navigation.view_ais;
          if (action) result = action(id);
        }
      }
      if (!result.ok) {
        ais_selection_.Clear();
        ConfirmSheet(frame_, mode_, "Unable to select target", wxString::FromUTF8(result.message), "Back");
      } else {
        // Prototype showTarget returns to the chart after a successful jump.
        // Keep the validated identity highlighted; a failed action stays here.
        ShowNavigation();
      }
    });
    ais_drawer_->on_select = [this, lifetime](int id) {
      if (lifetime.expired()) return;
      ais_selection_.Clear();
      if (!state_.simulated && !state_.replayed && id > 0)
        ais_selection_.Select(id, ais_state_, vessel::Clock::now());
    };
  }
  ais_drawer_->Update(ais_state_, online_ais_state_, vessel::Clock::now(), mode_);
  if (mmsi > 0) ais_drawer_->Target(mmsi); else ais_drawer_->List();
  ais_drawer_->Present(DrawerWorkspace());
  Tick();
}
void Shell::ActivateHorizon(const application::HorizonAction &action) {
  // A click can follow route edits, receiver loss, replay entry, or an AIS
  // replacement since the last repaint. Observe owned state again on the app
  // thread; never trigger upstream navigation processing to obtain it.
  if(simulation_ || (actions_.commissioning && actions_.commissioning->Replaying()))return;
  if(!actions_.live_state)return;
  auto current=actions_.live_state();
  if(actions_.route)current.navigation.route=actions_.route();
  const auto now=vessel::Clock::now();
  const auto onboard=actions_.navigation.ais ? actions_.navigation.ais(now) : vessel::AisState{};
  if(!application::HorizonActionAllowed(action,current,onboard,now))return;
  using Kind=application::HorizonActionKind;
  if(action.kind==Kind::Follow){if(actions_.follow)actions_.follow();}
  else if(action.kind==Kind::Passage)ShowPassage();
  else if(action.kind==Kind::Ais){Tick();ShowAis(action.mmsi);}
}
void Shell::ShowPassage() {
  ShowNavigation();
  if (!passage_drawer_) {
    passage_drawer_ = new XNavPassageDrawer(frame_, actions_.navigation);
    passage_drawer_->on_library = [this] { ShowProduct(ProductPage::Routes); };
    if (actions_.navigation.start_route)
      passage_drawer_->on_plot = [this] {
        if (state_.simulated || state_.replayed) return;
        ShowNavigation();
        actions_.navigation.start_route();
        Tick();
      };
  }
  passage_drawer_->Update(state_, field_snapshot_.advice, field_snapshot_.energy,
                          field_snapshot_.now, mode_);
  passage_drawer_->Present(DrawerWorkspace());
  Tick();
}
void Shell::ShowSettings() {
  ShowNavigation();
  if (!settings_drawer_) {
    SettingsDrawerActions actions;
    actions.page = [this](ProductPage page) { ShowProduct(page); };
    if(actions_.navigation.legacy_settings)
      actions.advanced = [this] { ShowNavigation(); actions_.navigation.legacy_settings(); };
    if(actions_.navigation.plugin_settings)
      actions.plugins = [this] { ShowNavigation(); actions_.navigation.plugin_settings(); };
    actions.diagnostics = [this] { ShowPage(PreviewPage::Diagnostics); };
    actions.fullscreen = [this] { frame_.ShowFullScreen(!frame_.IsFullScreen()); };
    actions.theme = [this](LightMode mode) { SetLight(mode); };
    actions.settings = actions_.settings;
    actions.save_vessel = actions_.save_vessel;
    actions.legacy = actions_.legacy;
    actions.safe = actions_.safe;
    settings_drawer_ = new XNavSettingsDrawer(frame_, std::move(actions));
    settings_drawer_->on_dismiss=[this]{if(settings_drawer_)settings_drawer_->ResetDraft();};
  }
  settings_drawer_->Present(DrawerWorkspace());
  Tick();
}
void Shell::ShowAnchor() {
  ShowNavigation();
  if(!anchor_drawer_)anchor_drawer_=new XNavAnchorDrawer(frame_,actions_.navigation);
  anchor_drawer_->Present(DrawerWorkspace());
  Tick();
}
void Shell::ShowPilot() {
  ShowNavigation();
  if(!pilot_drawer_)pilot_drawer_=new XNavPilotDrawer(frame_,pilot_actions_);
  pilot_drawer_->Present(DrawerWorkspace());
  Tick();
}
void Shell::ShowAlerts() {
  ShowNavigation();
  if(!alert_drawer_) {
    AlertDrawerActions callbacks;
    callbacks.acknowledge=[this](const std::string &id,std::uint64_t episode) {
      alerts_.Acknowledge(id,episode);Tick();
    };
    callbacks.inspect=[this](application::AlertArea area) {
      switch(area) {
      case application::AlertArea::Sources: ShowHealth();break;
      case application::AlertArea::Ais: ShowTraffic();break;
      case application::AlertArea::Anchor: ShowAnchor();break;
      case application::AlertArea::Energy: ShowPage(PreviewPage::Energy);break;
      case application::AlertArea::Pilot: ShowPilot();break;
      }
    };
    alert_drawer_=new XNavAlertDrawer(frame_,std::move(callbacks));
  }
  alert_drawer_->Present(DrawerWorkspace());Tick();
}
void Shell::CloseContext() {
  if (context_) context_->Dismiss();
  context_ = nullptr;
  context_waypoint_.clear();
  context_mmsi_ = 0;
  context_position_.reset();
}
void Shell::ShowHealth() {
  ShowNavigation();
  if(!health_drawer_) {
    HealthDrawerActions callbacks;
    callbacks.manage=[this]{ShowProduct(ProductPage::Sources);};
    callbacks.diagnostics=[this]{ShowProduct(ProductPage::FieldReport);};
    callbacks.configure=[this](const application::HealthSignal &s){
      if(s.id=="online") {ShowTraffic();ais_drawer_->ShowSettings();}
      else if(s.id=="ais") ShowTraffic();
      else if(s.id=="pilot") ShowProduct(ProductPage::PilotSettings);
      else {
        ShowProduct(ProductPage::Sources);
        product_->ShowSource(s.quantity.value_or(vessel::Quantity::Count),mode_);
      }
    };
    health_drawer_=new XNavHealthDrawer(frame_,std::move(callbacks));
  }
  health_drawer_->Present(DrawerWorkspace());Tick();
}
void Shell::UpdateContext(vessel::Time now) {
  if (!context_ || context_->IsBeingDeleted()) return;
  const bool live = !state_.simulated && !state_.replayed;
  if (context_mmsi_) {
    std::optional<vessel::AisTarget> selected;
    unsigned matches = 0;
    if (live && ais_state_.available)
      for (const auto &target : ais_state_.targets)
        if (target.mmsi == context_mmsi_) { selected = target; ++matches; }
    if (matches != 1) selected.reset();
    context_->UpdateAis(std::move(selected), now, live, mode_);
  } else if (!context_waypoint_.empty()) {
    application::WaypointContext point;
    if (actions_.navigation.waypoint_context)
      point = actions_.navigation.waypoint_context(context_waypoint_, now);
    if (!live) {
      point.range_nm = {}; point.bearing_true_deg = {};
      point.reason = "Live navigation unavailable during historical data";
    }
    context_->UpdateWaypoint(std::move(point), now, live, mode_);
  } else if (context_position_) {
    context_->UpdateChartPosition(*context_position_, mode_, live,
                                 CurrentMeasuredPosition(state_, now));
  }
  for (const auto &name : actions_.navigation_panes) {
    const auto &pane = manager_.GetPane(name);
    if (pane.IsOk() && pane.IsShown() && pane.window) {
      if (!context_->Place(pane.window->GetScreenRect())) CloseContext();
      return;
    }
  }
  CloseContext();
}
void Shell::ShowPage(PreviewPage page) {
  CloseContext();
  if (health_drawer_) health_drawer_->Dismiss();
  if (alert_drawer_) alert_drawer_->Dismiss();
  if (pilot_drawer_) pilot_drawer_->Dismiss();
  if (anchor_drawer_) anchor_drawer_->Dismiss();
  if (settings_drawer_) settings_drawer_->Dismiss();
  if (passage_drawer_) passage_drawer_->Dismiss();
  if (ais_drawer_) ais_drawer_->Dismiss();
  if (product_)
    manager_.GetPane(product_).Hide();
  if (navigation_visibility_.empty()) {
    auto names = actions_.navigation_panes;
    names.push_back("OpenNavHorizon");
    for (const auto &name : names) {
      auto &pane = manager_.GetPane(name);
      if (pane.IsOk()) {
        navigation_visibility_.push_back({name, pane.IsShown()});
        pane.Hide();
      }
    }
  }
  current_page_ = page;
  manager_.GetPane("OpenNavHorizon").Hide();
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

void Shell::PlaceChartControls() {
  wxRect chart;
  if (!page_->IsShown() && !product_->IsShown())
    for (const auto &name : actions_.navigation_panes) {
      const auto &pane=manager_.GetPane(name);
      if(pane.IsOk()&&pane.IsShown()&&pane.window) {
        chart=wxRect(frame_.ScreenToClient(pane.window->GetScreenPosition()),pane.window->GetSize());break;
      }
    }
  const bool available=frame_.IsShownOnScreen()&&!frame_.IsIconized()&&frame_.IsEnabled()&&
      chart.width>frame_.FromDIP(420)&&chart.height>frame_.FromDIP(240);
  for(auto *overlay:chart_overlays_) {
    if(!available){overlay->Hide();continue;}
    const auto size=overlay->GetSize();
    wxPoint position;
    if(overlay==chart_tools_)position={chart.x+chart.width-size.x-frame_.FromDIP(22),chart.y+chart.height-size.y-frame_.FromDIP(37)};
    else if(overlay==chart_orientation_)position={chart.x+chart.width-size.x-frame_.FromDIP(22),chart.y+frame_.FromDIP(22)};
    else position={chart.x+frame_.FromDIP(28),chart.y+chart.height-size.y-frame_.FromDIP(37)};
    // A separate owned surface remains above both software and GL child
    // canvases without repeatedly raising the entire chart/application.
    const auto screen = frame_.ClientToScreen(position);
    const auto drawer = DrawerRegion();
    if (drawer && drawer->Intersects(wxRect(screen,size)))
      overlay->Hide();
    else static_cast<XNavFloatingSurface *>(overlay)->Present(screen);
  }
}

void Shell::AfterCanvasLayoutChanged() {
  navigation_visibility_.clear();
  for(const auto &name : actions_.navigation_panes) {
    auto &pane=manager_.GetPane(name);
    if(pane.IsOk())pane.Show();
  }
  for(const auto &name : {"OpenNavTools","OpenNavData","OpenNavHorizon"}) {
    auto &pane=manager_.GetPane(name);if(pane.IsOk())pane.Show();
  }
  ShowNavigation();
  // This is a genuine upstream reconfiguration, including when Navigation was
  // already visible. Commit the restored pane flags even if ShowNavigation's
  // ordinary same-page fast path had no layout work of its own.
  manager_.Update();
}

void Shell::ShowChartContext(application::Coordinate position) {
  ShowNavigation();
  context_position_ = position;
  const std::weak_ptr<int> lifetime = context_lifetime_;
  context_ = new XNavContextCard(frame_, ContextKind::ChartPosition,
      [this, lifetime, position](ContextAction action, std::optional<application::Waypoint>) {
    if (lifetime.expired()) return;
    const auto live = [this] { return !state_.simulated && !state_.replayed; };
    const auto result = [this](const application::CommandResult &value) {
      if (!value.ok)
        ConfirmSheet(frame_, mode_, "Unable to continue", wxString::FromUTF8(value.message), "Back");
    };
    if (action == ContextAction::GoTo && actions_.navigation.go_to) {
      if (!live() || !CurrentMeasuredPosition(state_, vessel::Clock::now())) return;
      if (ConfirmSheet(frame_, mode_, "Go to this position",
          wxString::Format(wxString::FromUTF8("Destination %.5f° %.5f°. Check the chart before starting."),
                           position.latitude_deg, position.longitude_deg), "Start") &&
          live() && CurrentMeasuredPosition(state_, vessel::Clock::now()))
        result(actions_.navigation.go_to(position, "Go To"));
    } else if (action == ContextAction::CreateWaypoint && actions_.navigation.create_waypoint) {
      if (!live()) return;
      const auto fields = EditSheet(frame_, mode_, "Create waypoint", "Save this chart position.",
                                    {{"Name", "Waypoint", 128}}, "Save");
      if (fields && live()) result(actions_.navigation.create_waypoint(position, (*fields)[0], ""));
    } else if (action == ContextAction::Measure && actions_.navigation.measure) {
      actions_.navigation.measure();
    } else if (action == ContextAction::Info && actions_.navigation.object_info_at) {
      actions_.navigation.object_info_at(position);
    }
  });
  UpdateContext(vessel::Clock::now());
  if (context_) { context_->Show(); context_->Raise(); UpdateContext(vessel::Clock::now()); }
}

} // namespace opennav::ui

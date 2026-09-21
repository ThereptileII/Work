#include "ui/Shell.h"

#include <wx/datetime.h>
#include <wx/popupwin.h>
#include <wx/sizer.h>

#include <utility>

namespace opennav::ui {
namespace {
class SystemPopup final : public wxPopupTransientWindow {
 public:
  explicit SystemPopup(wxWindow* parent) : wxPopupTransientWindow(parent, wxBORDER_NONE) {}
 protected:
  void OnDismiss() override { Destroy(); }
};
}  // namespace

wxPanel* Shell::MakePane(const wxString& name, wxAuiPaneInfo placement) {
  auto* panel = new wxPanel(&frame_, wxID_ANY);
  panel->SetName(name);
  manager_.AddPane(panel, placement.Name(name).CaptionVisible(false)
      .CloseButton(false).PaneBorder(false).Resizable(true).DockFixed());
  panes_.push_back(panel);
  return panel;
}

XNavButton* Shell::Button(wxWindow* parent, const wxString& text,
                          const wxString& name, std::function<void()> callback) {
  auto* button = new XNavButton(parent, wxID_ANY, text, name);
  button->Bind(wxEVT_BUTTON, [callback = std::move(callback)](wxCommandEvent&) {
    if (callback) callback();
  });
  buttons_.push_back(button);
  return button;
}

wxStaticText* Shell::Text(wxWindow* parent, const wxString& text, int size, bool bold) {
  auto* label = new wxStaticText(parent, wxID_ANY, text);
  label->SetFont(UiFont(*parent, size, bold));
  labels_.push_back(label);
  return label;
}

Shell::Shell(wxFrame& frame, wxAuiManager& manager, ShellActions actions,
             LightMode mode, bool simulation)
    : frame_(frame), manager_(manager), actions_(std::move(actions)), mode_(mode),
      simulation_(simulation), timer_(this) {
  const int gap = frame_.FromDIP(spacing::base);
  auto* top = MakePane("OpenNavTop", wxAuiPaneInfo().Top().Layer(10)
      .BestSize(-1, frame_.FromDIP(56)));
  auto* row = new wxBoxSizer(wxHORIZONTAL);
  row->Add(Text(top, "OpenNav X", 22, true), 0, wxALIGN_CENTER_VERTICAL | wxLEFT, gap * 2);
  row->AddSpacer(gap * 3);
  clock_ = Text(top, "", 15);
  clock_->SetMinSize(frame_.FromDIP(wxSize(56, 24)));
  row->Add(clock_, 0, wxALIGN_CENTER_VERTICAL);
  row->AddStretchSpacer();
  source_ = Text(top, "No vessel input", 13, true);
  row->Add(source_, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, gap * 2);
  auto* theme = Button(top, "Light", "Cycle day, dusk and night palettes", [this] {
    mode_ = mode_ == LightMode::Day ? LightMode::Dusk
          : mode_ == LightMode::Dusk ? LightMode::Night : LightMode::Day;
    if (actions_.theme) actions_.theme(mode_);
    ApplyTheme();
  });
  theme->SetMinSize(frame_.FromDIP(wxSize(72, 48)));
  row->Add(theme, 0, wxALL, frame_.FromDIP(4));
  top->SetSizer(row);

  auto* left = MakePane("OpenNavTools", wxAuiPaneInfo().Left().Layer(1)
      .BestSize(frame_.FromDIP(spacing::left_rail), -1));
  auto* tools = new wxBoxSizer(wxVERTICAL);
  tools->Add(Button(left, "+", "Zoom chart in", actions_.zoom_in), 0, wxALL, frame_.FromDIP(4));
  tools->Add(Button(left, wxString::FromUTF8("−"), "Zoom chart out", actions_.zoom_out), 0, wxALL, frame_.FromDIP(4));
  tools->Add(Button(left, "GPS", "Follow own ship using OpenCPN", actions_.follow), 0, wxALL, frame_.FromDIP(4));
  tools->AddStretchSpacer();
  left->SetSizer(tools);

  auto* right = MakePane("OpenNavData", wxAuiPaneInfo().Right().Layer(1)
      .BestSize(frame_.FromDIP(spacing::right_rail), -1));
  auto* rail = new wxBoxSizer(wxVERTICAL);
  wind_ = new XNavDataValue(right, "APPARENT WIND", "kn");
  depth_ = new XNavDataValue(right, "DEPTH", "m / transducer");
  speed_ = new XNavDataValue(right, "SOG", "kn");
  course_ = new XNavDataValue(right, "COG", "deg true", 0);
  for (auto* value : {wind_, depth_, speed_, course_}) rail->Add(value, 0, wxEXPAND);
  rail->AddStretchSpacer();
  right->SetSizer(rail);

  auto* bottom = MakePane("OpenNavActions", wxAuiPaneInfo().Bottom().Layer(10)
      .BestSize(-1, frame_.FromDIP(spacing::action_height)));
  auto* actions_row = new wxBoxSizer(wxHORIZONTAL);
  actions_row->Add(Text(bottom, "AUTOPILOT", 11, true), 0, wxALIGN_CENTER_VERTICAL | wxLEFT, gap * 2);
  actions_row->Add(Text(bottom, "Unavailable", 15), 0, wxALIGN_CENTER_VERTICAL | wxLEFT, gap * 2);
  actions_row->AddStretchSpacer();
  auto* system = Button(bottom, "System", "System and Open Legacy OpenCPN", [this] { ShowSystem(); });
  system->SetMinSize(frame_.FromDIP(wxSize(112, 48)));
  actions_row->Add(system, 0, wxALL, frame_.FromDIP(4));
  bottom->SetSizer(actions_row);
  ApplyTheme();
  Tick();
  manager_.Update();
  Bind(wxEVT_TIMER, [this](wxTimerEvent&) { Tick(); });
  timer_.Start(250);
}

Shell::~Shell() {
  timer_.Stop();
  for (auto* pane : panes_) {
    manager_.DetachPane(pane);
    pane->Destroy();
  }
  manager_.Update();
}

void Shell::UpdateState(const vessel::VesselState& state) {
  if (!simulation_) state_ = state;
}

void Shell::ApplyTheme() {
  const auto colors = Theme(mode_);
  for (auto* pane : panes_) {
    pane->SetBackgroundColour(Colour(colors.surface));
    pane->Refresh();
  }
  for (auto* label : labels_) label->SetForegroundColour(Colour(colors.secondary));
  for (auto* button : buttons_) button->SetLightMode(mode_);
  for (auto* value : {wind_, depth_, speed_, course_}) value->SetLightMode(mode_);
  source_->SetForegroundColour(Colour(simulation_ ? colors.attention : colors.secondary));
}

void Shell::Tick() {
  const auto now = vessel::Clock::now();
  if (simulation_ && !simulation_paused_) state_ = vessel::SimulatorFixture(now);
  wind_->SetReading(state_.wind.apparent_speed_kn, now);
  depth_->SetReading(state_.environment.depth_below_transducer_m, now);
  speed_->SetReading(state_.navigation.sog_kn, now);
  course_->SetReading(state_.navigation.cog_deg, now);
  clock_->SetLabel(wxDateTime::Now().Format("%H:%M"));
  const wxString label = simulation_ ? wxString(simulation_paused_ ? "SIMULATION PAUSED" : "SIMULATION")
                                     : InputSummary();
  if (source_->GetLabel() != label) {
    source_->SetLabel(label);
    source_->GetParent()->Layout();
  }
}

wxString Shell::InputSummary() const {
  bool present = false, current = false;
  for (const auto* sample : {&state_.navigation.latitude_deg, &state_.navigation.sog_kn,
                            &state_.navigation.cog_deg}) {
    const auto assessment = vessel::Assess(*sample, vessel::Clock::now());
    present = present || assessment.value.has_value();
    current = current || assessment.quality == vessel::Quality::Live ||
                         assessment.quality == vessel::Quality::Aging;
  }
  return current ? "OpenCPN navigation" : present ? "Navigation stale" : "No vessel input";
}

void Shell::ShowSystem() {
  auto* popup = new SystemPopup(&frame_);
  popup->SetBackgroundColour(Colour(Theme(mode_).elevated));
  auto* layout = new wxBoxSizer(wxVERTICAL);
  const int gap = frame_.FromDIP(12);
  auto* heading = new wxStaticText(popup, wxID_ANY, "System");
  heading->SetFont(UiFont(*popup, 20, true));
  heading->SetForegroundColour(Colour(Theme(mode_).primary));
  layout->Add(heading, 0, wxALL, gap);
  auto* info = new wxStaticText(popup, wxID_ANY,
      "OpenNav X / development slice\nOpenCPN 5.12.4 / API 1.20\nMode: XNav\n" +
      (simulation_ ? wxString("Data: explicit simulator") : "Data: " + InputSummary()) +
      "\nOpenNav device controls: unavailable");
  info->SetFont(UiFont(*popup, 13));
  info->SetForegroundColour(Colour(Theme(mode_).secondary));
  layout->Add(info, 0, wxLEFT | wxRIGHT | wxBOTTOM, gap);
  auto* pause = new XNavButton(popup, wxID_ANY,
      simulation_ ? (simulation_paused_ ? "Resume simulation" : "Pause simulation") : "Start labelled simulation",
      "Simulator control; never affects OpenCPN chart or device state");
  pause->SetLightMode(mode_);
  pause->Bind(wxEVT_BUTTON, [this, popup](wxCommandEvent&) {
    if (!simulation_) { simulation_ = true; simulation_paused_ = false; }
    else simulation_paused_ = !simulation_paused_;
    ApplyTheme();
    popup->Dismiss();
    popup->Destroy();
  });
  layout->Add(pause, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
  auto* legacy = new XNavButton(popup, wxID_ANY, "Open Legacy OpenCPN", "Save and restart in Legacy OpenCPN");
  legacy->SetLightMode(mode_);
  legacy->Bind(wxEVT_BUTTON, [this, popup](wxCommandEvent&) {
    // Closing the host can synchronously destroy this Shell and its actions.
    const auto restart_action = actions_.legacy;
    popup->Dismiss();
    popup->Destroy();
    if (restart_action) restart_action();
  });
  layout->Add(legacy, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
  popup->SetSizerAndFit(layout);
  const auto client = frame_.GetClientSize();
  const auto corner = frame_.ClientToScreen(wxPoint(client.x, client.y));
  popup->Move(corner.x - popup->GetSize().x - gap,
              corner.y - popup->GetSize().y - frame_.FromDIP(64));
  popup->Popup();
}

}  // namespace opennav::ui

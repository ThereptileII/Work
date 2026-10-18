#include "ui/RailPersonaliseDrawer.h"
#include "ui/Sheet.h"
#include <algorithm>
#include <cmath>
#include <wx/dcbuffer.h>
#include <wx/weakref.h>

namespace opennav::ui {
namespace {
wxString W(const std::string &text) { return wxString::FromUTF8(text); }
const std::vector<std::string> essentials{"sog", "depth", "tws", "soc"};
const std::vector<std::pair<std::string, wxString>> quantities{
    {"sog", "Speed over ground"}, {"depth", "Depth below transducer"},
    {"tws", "True wind"}, {"soc", "Battery state of charge"},
    {"heading", "Heading (true)"}, {"water_temp", "Water temperature"},
    {"rpm", "Motor RPM"}, {"stw", "Speed through water"},
    {"cog", "Course over ground"}, {"aws", "Apparent wind"},
    {"awa", "Apparent wind angle"}, {"twa", "True wind angle"},
    {"pressure", "Air pressure"}, {"rudder", "Rudder angle"},
    {"heel", "Heel"}, {"voltage", "Battery voltage"},
    {"current", "Pack current"}, {"pack_power", "Pack discharge"},
    {"motor_power", "Motor power"}, {"motor_temp", "Motor temperature"},
    {"fresh_water", "Fresh water"}, {"fuel", "Fuel"}, {"waste", "Waste"}};

std::vector<std::string> Four(const application::Settings &settings) {
  auto values = settings.data_rail;
  if (values.size() > 4) values.resize(4);
  for (const auto &key : essentials)
    if (values.size() < 4 && std::find(values.begin(), values.end(), key) == values.end())
      values.push_back(key);
  return values;
}
void Select(application::Settings &settings, std::vector<std::string> values) {
  // Older profiles could hold six rail values. Retain their overflow in the
  // full Instruments selection when the person first chooses a four-slot rail.
  for (std::size_t i = 4; i < settings.data_rail.size(); ++i)
    if (std::find(settings.instruments.begin(), settings.instruments.end(),
                  settings.data_rail[i]) == settings.instruments.end())
      settings.instruments.push_back(settings.data_rail[i]);
  settings.data_rail = std::move(values);
}
} // namespace

XNavRailPersonaliseDrawer::XNavRailPersonaliseDrawer(
    wxWindow &owner, Read read, Save save)
    : XNavDrawer(owner, "Personalise instruments"), read_(std::move(read)),
      save_(std::move(save)) {
  SetHeading("PERSONALISE", "Your four essentials", false);
  Bind(wxEVT_SHOW, [this](wxShowEvent &event) {
    if (!event.IsShown()) ++generation_;
    event.Skip();
  });
}
void XNavRailPersonaliseDrawer::Open(const wxRect &workspace, LightMode mode,
                                    bool editable) {
  ++generation_;
  feedback_.clear();
  editable_ = editable && bool(read_) && bool(save_);
  SetLight(mode);
  Build();
  Present(workspace);
  body_->Scroll(0, 0);
}
void XNavRailPersonaliseDrawer::Update(LightMode mode, bool editable) {
  const bool enabled = editable && bool(read_) && bool(save_);
  if (editable_ != enabled) ++generation_;
  editable_ = enabled;
  if (light_ != mode) {
    SetLight(mode);
    for (auto *panel : copy_) panel->Refresh(false);
  }
  for (auto *field : positions_)
    if (field) { field->SetLightMode(mode); field->Enable(editable_); }
  if (presets_) {
    presets_->SetLightMode(mode);
    presets_->Enable(editable_ && !observed_.rail_presets.empty());
  }
  for (auto *button : buttons_) {
    button->SetLightMode(mode);
    button->Enable(editable_);
  }
  if (delete_) delete_->Enable(editable_ && !selected_preset_.empty());
}
void XNavRailPersonaliseDrawer::SetInterfaceScale(int percent) {
  const int previous = InterfaceScale();
  XNavDrawer::SetInterfaceScale(percent);
  if (previous == InterfaceScale() || !IsShownOnScreen()) return;
  const int scroll = body_->GetViewStart().y;
  Build(); body_->Scroll(0, scroll);
}
wxPanel *XNavRailPersonaliseDrawer::Copy(const wxString &text, int height, int size) {
  auto *panel = new wxPanel(body_, wxID_ANY);
  panel->SetName(text);
  const int font = static_cast<int>(std::lround(size * InterfaceScale() / 100.));
  panel->SetMinSize(FromDIP(wxSize(1,
      static_cast<int>(std::lround(height * InterfaceScale() / 100.)))));
  panel->SetBackgroundStyle(wxBG_STYLE_PAINT);
  panel->Bind(wxEVT_PAINT, [this, panel, text, font](wxPaintEvent &) {
    wxAutoBufferedPaintDC dc(panel);
    XNavPainter p(*panel, dc, light_);
    dc.SetBackground(wxBrush(Colour(p.c.background))); dc.Clear();
    p.Wrapped(text, 0, 0, font, font + 9,
              panel->ToDIP(panel->GetClientSize().x), p.c.secondary, 4);
  });
  EnableScrollGesture(*panel);
  content_->Add(panel, 0, wxEXPAND);
  copy_.push_back(panel);
  return panel;
}
void XNavRailPersonaliseDrawer::Queue(std::function<void()> action) {
  const auto generation = generation_;
  CallAfter([this, generation, action = std::move(action)] {
    if (generation == generation_ && editable_ && IsShownOnScreen()) action();
  });
}
XNavButton *XNavRailPersonaliseDrawer::Action(
    const wxString &label, std::function<void()> action) {
  auto *button = new XNavButton(body_, wxID_ANY, label, label);
  button->SetDisplayAction(48);
  button->SetInterfaceScale(InterfaceScale());
  button->Bind(wxEVT_BUTTON, [this, action](wxCommandEvent &) { Queue(action); });
  content_->Add(button, 0, wxEXPAND | wxBOTTOM, FromDIP(10));
  buttons_.push_back(button);
  return button;
}
void XNavRailPersonaliseDrawer::Build() {
  ++generation_;
  ClearBody(); positions_ = {}; buttons_.clear(); copy_.clear();
  presets_ = nullptr; delete_ = nullptr;
  try {
    observed_ = read_ ? read_() : application::Settings{};
    application::ValidateSettings(observed_);
  } catch (const std::exception &) {
    editable_ = false;
    Copy("Settings could not be read. Reopen Personalise after checking the profile.", 84);
    body_->Layout(); body_->FitInside();
    return;
  }
  Copy("Choose the values you want to read at a glance. Tap any value on the rail for more detail.", 60);
  const auto values = Four(observed_);
  std::vector<wxString> choices;
  for (const auto &quantity : quantities) choices.push_back(quantity.second);
  for (std::size_t i = 0; i < positions_.size(); ++i) {
    const auto label = wxString::Format("Position %u", static_cast<unsigned>(i + 1));
    Copy(label, 24);
    auto *field = new XNavChoiceField(body_, wxID_ANY, choices, label);
    field->SetInterfaceScale(InterfaceScale());
    for (std::size_t q = 0; q < quantities.size(); ++q)
      if (quantities[q].first == values[i]) field->SetSelection(static_cast<int>(q));
    field->Bind(wxEVT_CHOICE, [this, i](wxCommandEvent &event) {
      const int selection = event.GetInt();
      if (selection < 0 || static_cast<std::size_t>(selection) >= quantities.size()) return;
      const auto key = quantities[static_cast<std::size_t>(selection)].first;
      // Let the transient select finish before rebuilding its owner controls.
      Queue([this, i, key] { Choose(i, key); });
    });
    content_->Add(field, 0, wxEXPAND | wxBOTTOM, FromDIP(19));
    positions_[i] = field;
  }
  Action("Restore essentials", [this] {
    selected_preset_.clear();
    Commit([](application::Settings &settings) { Select(settings, essentials); });
  });
  if (!feedback_.empty()) Copy(feedback_, 63, 11);
  if (!editable_) Copy("Return to LIVE to change your rail or saved presets.", 55, 11);
  content_->AddSpacer(FromDIP(12));
  Copy("SAVED PRESETS", 26, 10);
  std::vector<wxString> preset_names{"Custom arrangement"};
  int selected = 0;
  for (std::size_t i = 0; i < observed_.rail_presets.size(); ++i) {
    const auto &preset = observed_.rail_presets[i];
    preset_names.push_back(W(preset.name));
    if (preset.name == selected_preset_ && preset.values == values)
      selected = static_cast<int>(i + 1);
  }
  if (!selected)
    for (std::size_t i = 0; i < observed_.rail_presets.size(); ++i)
      if (observed_.rail_presets[i].values == values) {
        selected = static_cast<int>(i + 1);
        selected_preset_ = observed_.rail_presets[i].name;
        break;
      }
  if (!selected) selected_preset_.clear();
  presets_ = new XNavChoiceField(body_, wxID_ANY, preset_names, "Saved rail preset");
  presets_->SetInterfaceScale(InterfaceScale()); presets_->SetSelection(selected);
  presets_->Bind(wxEVT_CHOICE, [this](wxCommandEvent &event) {
    const int selection = event.GetInt();
    if (selection > 0)
      Queue([this, selection] { ChoosePreset(static_cast<std::size_t>(selection - 1)); });
    else if (selection == 0)
      Queue([this] {
        selected_preset_.clear();
        if (delete_) delete_->Enable(false);
      });
  });
  content_->Add(presets_, 0, wxEXPAND | wxBOTTOM, FromDIP(12));
  Action("Save as a named preset", [this] { SavePreset(); });
  delete_ = Action("Delete selected preset", [this] { DeletePreset(); });
  Copy("Save up to eight arrangements, such as Anchor or Sailing. Presets change the four displayed quantities; they do not change sensor sources or alarms.", 84, 11);
  Update(light_, editable_);
  body_->Layout(); body_->FitInside();
}
void XNavRailPersonaliseDrawer::Commit(
    const std::function<void(application::Settings &)> &change) {
  if (!editable_ || !IsShownOnScreen() || !read_ || !save_) return;
  const int scroll = body_->GetViewStart().y;
  try {
    auto current = read_();
    if (current.data_rail != observed_.data_rail ||
        current.rail_presets != observed_.rail_presets) {
      feedback_ = "Your rail changed elsewhere. Review the current choices and try again.";
    } else {
      change(current);
      application::ValidateSettings(current);
      const auto result = save_(current);
      feedback_ = W(result.message.empty()
          ? (result.ok ? "Rail saved" : "Rail could not be saved") : result.message);
    }
  } catch (const std::exception &error) { feedback_ = W(error.what()); }
  Build(); // Always render the persisted readback, including rejected saves.
  body_->Scroll(0, scroll);
}
void XNavRailPersonaliseDrawer::Choose(std::size_t position, const std::string &key) {
  selected_preset_.clear();
  Commit([position, key](application::Settings &settings) {
    auto values = Four(settings);
    if (position >= values.size()) return;
    const auto existing = std::find(values.begin(), values.end(), key);
    // Choosing an already displayed quantity swaps positions, keeping four
    // distinct readings without silently deleting another chosen quantity.
    if (existing != values.end()) std::swap(values[position], *existing);
    else values[position] = key;
    Select(settings, std::move(values));
  });
}
void XNavRailPersonaliseDrawer::ChoosePreset(std::size_t index) {
  if (index >= observed_.rail_presets.size()) return;
  const auto preset = observed_.rail_presets[index];
  selected_preset_ = preset.name;
  Commit([preset](application::Settings &settings) { Select(settings, preset.values); });
}
void XNavRailPersonaliseDrawer::SavePreset() {
  if (observed_.rail_presets.size() >= 8) {
    feedback_ = "Eight presets are already saved. Delete one before adding another.";
    Build(); return;
  }
  const auto generation = generation_;
  const auto self = wxWeakRef<XNavRailPersonaliseDrawer>(this);
  const auto name = EditSheet(*this, light_, "Save rail preset",
      "Name these four quantities, for example Anchor. Existing presets are never overwritten.",
      {{"Preset name", W(draft_name_), 64}}, "Save preset", InterfaceScale());
  if (!self || !name || name->empty()) return;
  self->draft_name_ = name->front();
  if (generation != self->generation_ || !self->editable_ || !self->IsShownOnScreen()) return;
  const auto cleaned = W(self->draft_name_).Trim(true).Trim(false).ToStdString(wxConvUTF8);
  self->Commit([cleaned](application::Settings &settings) {
    Select(settings, Four(settings));
    settings.rail_presets.push_back({cleaned, settings.data_rail});
  });
}
void XNavRailPersonaliseDrawer::DeletePreset() {
  if (selected_preset_.empty()) return;
  const auto selected = selected_preset_;
  const auto generation = generation_;
  const auto self = wxWeakRef<XNavRailPersonaliseDrawer>(this);
  const bool confirmed = ConfirmSheet(*this, light_, "Delete rail preset?",
      "Delete “" + W(selected) + "”? Your current four readings remain selected.",
      "Delete preset", InterfaceScale());
  if (!self || !confirmed || generation != self->generation_ ||
      !self->editable_ || !self->IsShownOnScreen()) return;
  self->Commit([selected](application::Settings &settings) {
    auto &presets = settings.rail_presets;
    presets.erase(std::remove_if(presets.begin(), presets.end(),
        [&selected](const auto &preset) { return preset.name == selected; }), presets.end());
  });
}
} // namespace opennav::ui

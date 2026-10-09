#include "ui/BoatSetupDialog.h"
#include "ui/Controls.h"
#include "ui/ChoiceField.h"
#include "ui/Sheet.h"
#include <wx/dialog.h>
#include <wx/display.h>
#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <array>
#include <algorithm>
#include <cmath>
namespace opennav::ui {
wxRect BoatSetupDialogBounds(const wxRect& work_area, const wxRect& parent,
                             const wxSize& preferred, int margin) {
  const int width = std::max(1, work_area.width);
  const int height = std::max(1, work_area.height);
  const int inset_x = std::clamp(margin, 0, (width - 1) / 2);
  const int inset_y = std::clamp(margin, 0, (height - 1) / 2);
  const int w = std::clamp(preferred.x, 1, width - 2 * inset_x);
  const int h = std::clamp(preferred.y, 1, height - 2 * inset_y);
  return {std::clamp(parent.x + (parent.width - w) / 2,
                     work_area.x + inset_x, work_area.x + width - inset_x - w),
          std::clamp(parent.y + (parent.height - h) / 2,
                     work_area.y + inset_y, work_area.y + height - inset_y - h),
          w, h};
}
namespace {
class BoatSetupDialog final : public wxDialog {
 public:
  BoatSetupDialog(wxWindow& parent, application::BoatSetupDraft draft,
                  BoatSetupActions actions, LightMode mode)
      : wxDialog(&parent, wxID_ANY, "Boat Setup & Sensor Check", wxDefaultPosition,
          wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER),
        draft_(std::move(draft)), actions_(std::move(actions)), mode_(mode), original_safety_(draft_.safety_depth_m) {
    SetName("Boat Setup & Sensor Check");
    SetBackgroundColour(Colour(Theme(mode_).background));
    auto* layout = new wxBoxSizer(wxVERTICAL);
    body_ = new wxScrolledWindow(this, wxID_ANY);
    body_->SetScrollRate(0, FromDIP(12));
    content_ = new wxBoxSizer(wxVERTICAL);
    body_->SetSizer(content_);
    layout->Add(body_, 1, wxEXPAND | wxALL, FromDIP(24));
    error_ = new wxStaticText(this, wxID_ANY, wxEmptyString);
    error_->SetForegroundColour(Colour(Theme(mode_).alarm));
    layout->Add(error_, 0, wxEXPAND | wxLEFT | wxRIGHT, FromDIP(24));
    auto* buttons = new wxBoxSizer(wxHORIZONTAL);
    auto add = [&](const char* title, auto callback, ButtonRole role) {
      auto* button = new XNavButton(this, wxID_ANY, title, title);
      button->SetLightMode(mode_); button->SetRole(role);
      button->SetMinSize(FromDIP(wxSize(140,48)));
      button->Bind(wxEVT_BUTTON, callback);
      buttons->Add(button, 1, wxLEFT, FromDIP(8));
      return button;
    };
    add("Later", [this](wxCommandEvent&) { Dismiss(); }, ButtonRole::Quiet);
    back_ = add("Back", [this](wxCommandEvent&) { if(Capture()) { --step_; Build(); } }, ButtonRole::Normal);
    next_ = add("Continue", [this](wxCommandEvent&) {
      if (!Capture()) return;
      if (step_ < 5) { ++step_; Build(); return; }
      const auto result = actions_.save ? actions_.save(draft_) : application::CommandResult{false,"Profile storage unavailable"};
      if (!result.ok) { Error(result.message); return; }
      Destroy();
    }, ButtonRole::Primary);
    layout->Add(buttons, 0, wxEXPAND | wxALL, FromDIP(16));
    SetSizer(layout);
    const int display_index = wxDisplay::GetFromWindow(&parent);
    const wxDisplay display(display_index == wxNOT_FOUND ? 0 : display_index);
    const auto bounds = BoatSetupDialogBounds(display.GetClientArea(), parent.GetScreenRect(),
                                              FromDIP(wxSize(660,620)), FromDIP(16));
    const auto minimum = FromDIP(wxSize(520,420));
    SetMinSize({std::min(minimum.x, bounds.width), std::min(minimum.y, bounds.height)});
    SetSize(bounds);
    Bind(wxEVT_CLOSE_WINDOW, [this](wxCloseEvent&) { Dismiss(); });
    Bind(wxEVT_CHAR_HOOK, [this](wxKeyEvent& event) {
      if(event.GetKeyCode()==WXK_ESCAPE) Dismiss(); else event.Skip();
    });
    Build();
  }
 private:
  void Label(const std::string& text, int size = 13, bool bold = false) {
    auto* label = new wxStaticText(body_, wxID_ANY, wxString::FromUTF8(text));
    label->SetFont(UiFont(*this,size,bold));
    label->SetForegroundColour(Colour(bold ? Theme(mode_).primary : Theme(mode_).secondary));
    label->Wrap(FromDIP(550));
    content_->Add(label,0,wxEXPAND | wxBOTTOM,FromDIP(14));
  }
  void Field(int index, const char* title, const std::string& value) {
    Label(title,12);
    auto* field = new wxTextCtrl(body_,wxID_ANY,wxString::FromUTF8(value));
    field->SetName(wxString::FromUTF8(title));field->SetFont(UiFont(*this,15));
    field->SetForegroundColour(Colour(Theme(mode_).primary));
    field->SetBackgroundColour(Colour(Theme(mode_).surface));
    field->SetMinSize(FromDIP(wxSize(-1,44)));field->SetMaxLength(index==0?128:64);
    field->Bind(wxEVT_TEXT,[this](wxCommandEvent& event){ dirty_=true; event.Skip(); });
    content_->Add(field,0,wxEXPAND | wxBOTTOM,FromDIP(18));fields_[index]=field;
  }
  // Nothing reaches storage until the final step, so leaving early throws the
  // whole draft away. Say so once the user has actually entered something;
  // an untouched dialog still closes without ceremony (SCRUM-344).
  void Dismiss() {
    if (dismissing_) return;
    dismissing_ = true;
    if (dirty_ && !ConfirmSheet(*this, mode_, "Discard boat setup?",
            "Nothing entered in this setup has been saved yet. Closing now "
            "discards it and leaves your existing settings untouched.",
            "Discard")) { dismissing_ = false; return; }
    Destroy();
  }
  void Error(const std::string& message) { error_->SetLabel(wxString::FromUTF8(message)); error_->Wrap(FromDIP(580)); Layout(); }
  bool Capture() {
    try {
      auto next = draft_;
      if (step_ == 0) {
        next.vessel_name=fields_[0]->GetValue().ToStdString(wxConvUTF8);
        next.settings.hazard.draft_m=application::ParseSettingNumber(fields_[1]->GetValue().ToStdString(wxConvUTF8));
        next.safety_depth_m=application::ParseSettingNumber(fields_[2]->GetValue().ToStdString(wxConvUTF8));
        if(std::isnan(next.safety_depth_m)) next.safety_depth_m=original_safety_;
      } else if(step_ == 1) {
        next.display.scale_percent=100+25*scale_->GetSelection();
        next.display.layout=static_cast<application::ChartLayout>(layout_->GetSelection());
      } else if(step_ == 3) {
        next.settings.energy.battery.capacity_kwh=application::ParseSettingNumber(fields_[3]->GetValue().ToStdString(wxConvUTF8));
        next.settings.energy.battery.reserve_soc_percent=application::ParseSettingNumber(fields_[4]->GetValue().ToStdString(wxConvUTF8));
      }
      application::ValidateBoatSetup(next);draft_=std::move(next);Error("");return true;
    } catch(const std::exception& e) { Error(e.what());return false; }
  }
  void Build() {
    fields_.fill(nullptr);scale_=layout_=nullptr;
    content_->Clear(true);
    const char* steps[]{"Your vessel","Display","Sources","Energy","Helm control","System summary"};
    Label("BOAT SETUP · "+std::to_string(step_+1)+" OF 6",11,true);
    Label(steps[step_],26,true);
    if(step_==0) {
      Label("Review your boat's assumptions. Leave unknown values blank. Blank safety depth preserves the current OpenCPN setting. Changes are saved only at the final step.");
      Field(0,"Vessel name",draft_.vessel_name);
      Field(1,"Draft · metres",application::SettingNumber(draft_.settings.hazard.draft_m));
      Field(2,"Chart safety depth · metres",application::SettingNumber(draft_.safety_depth_m));
    } else if(step_==1) {
      Label("Choose a readable helm layout. Existing OpenCPN units and day/dusk/night preferences are preserved.");
      Label("Interface scale",12);
      scale_=new XNavChoiceField(body_,wxID_ANY,{"100%","125%","150%"},"Setup interface scale");
      scale_->SetSelection((draft_.display.scale_percent-100)/25);scale_->SetLightMode(mode_);
      scale_->Bind(wxEVT_CHOICE,[this](wxCommandEvent& event){ dirty_=true; event.Skip(); });
      content_->Add(scale_,0,wxEXPAND|wxBOTTOM,FromDIP(20));
      Label("Chart layout",12);
      layout_=new XNavChoiceField(body_,wxID_ANY,{"Balanced","Chart focus","Instrument focus"},"Setup chart layout");
      layout_->SetSelection(static_cast<int>(draft_.display.layout));layout_->SetLightMode(mode_);
      layout_->Bind(wxEVT_CHOICE,[this](wxCommandEvent& event){ dirty_=true; event.Skip(); });
      content_->Add(layout_,0,wxEXPAND|wxBOTTOM,FromDIP(20));
    } else if(step_==2) {
      Label("Detected onboard observations at this check. Missing, aging and stale sources need attention; detection does not validate an installation.");
      Sensors();
      auto* refresh=new XNavButton(body_,wxID_ANY,"Check again","Refresh live sensor check");
      refresh->SetLightMode(mode_);refresh->SetMinSize(FromDIP(wxSize(160,44)));
      refresh->Bind(wxEVT_BUTTON,[this](wxCommandEvent&){CallAfter([this]{Build();});});
      content_->Add(refresh,0,wxTOP,FromDIP(12));
    } else if(step_==3) {
      Label("These are configured assumptions, never measured battery values. Unknown capacity or reserve keeps the energy estimate unavailable.");
      Field(3,"Usable battery capacity · kWh",application::SettingNumber(draft_.settings.energy.battery.capacity_kwh));
      Field(4,"Minimum reserve · %",application::SettingNumber(draft_.settings.energy.battery.reserve_soc_percent));
      Label("Battery identity, current sign and consumption calibration remain in Advanced battery settings. Setup does not guess them.");
    } else if(step_==4) {
      Label("You have the helm. This setup is status only and cannot enable steering or send commands.",18,true);
      Label(draft_.settings.pilot.permit_control ? "An existing control permission is configured. It is preserved; setup grants no runtime enablement." : "Autopilot control permission: OFF.");
      Label(draft_.settings.pilot.name.empty() ? "Autopilot identity: Unconfigured" : "Configured autopilot identity: "+draft_.settings.pilot.name);
      Label("Detection is not approval. Live identity, feedback, transport and acknowledgement gates still apply outside setup.");
    } else {
      for(const auto& line:application::BoatSetupSummary(draft_))Label(line);
      Label("Sensor check",15,true);Sensors();
      Label("Save finishes setup. Unconfigured or unavailable items remain visible in the helm; this summary does not certify passage readiness.");
    }
    back_->Enable(step_>0);next_->SetLabel(step_==5?"Save & open helm":"Continue");
    body_->Layout();body_->FitInside();body_->Scroll(0,0);Layout();
  }
  void Sensors() { for(const auto& line: actions_.sensors ? actions_.sensors() : std::vector<std::string>{"Live sensor check unavailable"})Label(line); }
  application::BoatSetupDraft draft_; BoatSetupActions actions_; LightMode mode_;
  double original_safety_;
  int step_=0; wxScrolledWindow* body_; wxBoxSizer* content_; wxStaticText* error_;
  XNavButton *back_,*next_; std::array<wxTextCtrl*,5> fields_{};
  XNavChoiceField *scale_=nullptr,*layout_=nullptr;
  bool dirty_=false, dismissing_=false;
};
}
wxDialog* ShowBoatSetupDialog(wxWindow& parent, application::BoatSetupDraft draft,
                             BoatSetupActions actions, LightMode mode) {
  auto* dialog=new BoatSetupDialog(parent,std::move(draft),std::move(actions),mode);
  dialog->Show();return dialog;
}
} // namespace opennav::ui

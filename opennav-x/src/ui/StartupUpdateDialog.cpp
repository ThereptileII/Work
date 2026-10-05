#include "ui/StartupUpdateDialog.h"

#include "application/Brand.h"
#include "ui/Controls.h"

#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

namespace opennav::ui {
std::optional<application::StartupUpdateCandidate> ShowStartupUpdateDialog(
    wxWindow* parent, application::StartupUpdate& update, LightMode light) {
  if (update.State() != application::StartupUpdateState::Prompt ||
      !update.Candidate()) return {};

  // Reuses the native custom controls and supplied popup's hierarchy: badge,
  // release title, explanation, right-aligned Later / Update Now actions.
  // Release-note examples in the prototype are not real release information.
  wxDialog dialog(parent, wxID_ANY, "SKAGER software update", wxDefaultPosition,
                  wxDefaultSize, wxBORDER_NONE | wxTAB_TRAVERSAL);
  dialog.SetName("SKAGER startup update");
  const auto palette = Theme(light);
  dialog.SetBackgroundColour(Colour(palette.elevated));
  const int inset = dialog.FromDIP(20);
  const int text_width = dialog.FromDIP(430);
  auto* body = new wxBoxSizer(wxVERTICAL);
  auto label = [&](const wxString& text, int size, std::uint32_t color,
                   bool bold, int space) {
    auto* value = new wxStaticText(&dialog, wxID_ANY, text);
    value->SetFont(UiFont(dialog, size, bold));
    value->SetForegroundColour(Colour(color));
    value->Wrap(text_width);
    body->Add(value, 0, wxEXPAND | wxBOTTOM, dialog.FromDIP(space));
  };
  label("UPDATE AVAILABLE", 10, palette.accent, true, 11);
  label(wxString::FromUTF8(application::brand::Name) + " " +
            wxString::FromUTF8(update.Candidate()->version),
        24, palette.primary, true, 8);
  label("A newer release is available. Update before continuing, or start "
        "this session with your current version.",
        12, palette.secondary, false, 16);
  label("Release metadata verified. OpenCPN compatibility checked.",
        12, palette.secondary, false, 16);
  label("Package verification is required before installation.",
        10, palette.muted, false, 14);

  auto* actions = new wxBoxSizer(wxHORIZONTAL);
  actions->AddStretchSpacer();
  auto* later = new XNavButton(&dialog, wxID_ANY, "LATER",
                               "Later: start with current version");
  auto* install = new XNavButton(&dialog, wxID_ANY, "UPDATE NOW",
                                 "Update now to verified release");
  for (auto* button : {later, install}) {
    button->SetLightMode(light);
    button->SetMinSize(dialog.FromDIP(wxSize(136, 48)));
    actions->Add(button, 0, wxLEFT, dialog.FromDIP(8));
  }
  later->SetRole(ButtonRole::Quiet);
  install->SetRole(ButtonRole::Primary);
  // Native MSW can still report IsModal while queued commands drain after
  // EndModal. Latch the first explicit decision independently of that state.
  std::optional<int> first_choice;
  const auto finish = [&dialog, &first_choice](int choice) {
    if (first_choice || !dialog.IsModal()) return;
    first_choice = choice;
    dialog.EndModal(choice);
  };
  later->Bind(wxEVT_BUTTON, [finish](wxCommandEvent&) {
    finish(wxID_CANCEL);
  });
  install->Bind(wxEVT_BUTTON, [finish](wxCommandEvent&) {
    finish(wxID_OK);
  });
  dialog.Bind(wxEVT_CHAR_HOOK, [finish](wxKeyEvent& event) {
    if (event.GetKeyCode() == WXK_ESCAPE) finish(wxID_CANCEL);
    else event.Skip();
  });
  dialog.Bind(wxEVT_CLOSE_WINDOW, [finish](wxCloseEvent&) {
    finish(wxID_CANCEL);
  });
  body->Add(actions, 0, wxEXPAND);
  auto* outer = new wxBoxSizer(wxVERTICAL);
  outer->Add(body, 1, wxALL | wxEXPAND, inset);
  dialog.SetSizerAndFit(outer);
  dialog.SetMinSize(dialog.GetSize());
  dialog.Centre();
  // Enter cannot inadvertently approve installation as the dialog opens.
  later->SetFocus();
  dialog.ShowModal();
  if (first_choice && *first_choice == wxID_OK) return update.UpdateNow();
  update.Later();
  return {};
}
}  // namespace opennav::ui

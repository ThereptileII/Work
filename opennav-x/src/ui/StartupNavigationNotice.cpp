#include "ui/StartupNavigationNotice.h"

#include "ui/Controls.h"

#include <wx/dialog.h>
#include <wx/display.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include <algorithm>
#include <optional>

namespace opennav::ui {
namespace {
// Retain every warranty, licence and navigation-safety statement from the
// pinned OpenCPN ShowNavWarning notice. SKAGER's English product presentation
// keeps the instruction and its actual action labels together, independent of
// older upstream translations which can instruct the user to click "OK".
constexpr char kNavigationNotice[] =
    "OpenCPN is distributed in the hope that it will be useful, "
    "but WITHOUT ANY WARRANTY; without even the implied "
    "warranty of MERCHANTABILITY or FITNESS FOR A "
    "PARTICULAR PURPOSE.\n\n"
    "See the GNU General Public License for more details.\n\n"
    "OpenCPN must only be used in conjunction with approved "
    "paper charts and traditional methods of navigation.\n\n"
    "DO NOT rely upon OpenCPN for safety of life or property.\n\n"
    "Please click \"Agree\" and proceed, or \"Cancel\" to quit.\n";
}  // namespace

bool ShowStartupNavigationNotice(wxWindow* parent) {
  wxDialog dialog(parent, wxID_ANY, "Welcome to SKAGER", wxDefaultPosition,
                  wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
  dialog.SetName("SKAGER navigation safety notice");
  constexpr auto mode = LightMode::Day;
  const auto palette = Theme(mode);
  dialog.SetBackgroundColour(Colour(palette.background));
  ThemeWindowChrome(dialog, mode);

  auto* layout = new wxBoxSizer(wxVERTICAL);
  auto* heading = new wxStaticText(&dialog, wxID_ANY, "Welcome to SKAGER");
  heading->SetFont(UiFont(dialog, 26, true));
  heading->SetForegroundColour(Colour(palette.primary));
  layout->Add(heading, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP,
              dialog.FromDIP(24));

  auto* subtitle = new wxStaticText(
      &dialog, wxID_ANY, "Navigation safety notice · OpenCPN");
  subtitle->SetFont(UiFont(dialog, 13));
  subtitle->SetForegroundColour(Colour(palette.secondary));
  layout->Add(subtitle, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP,
              dialog.FromDIP(24));

  // A native read-only text surface retains the complete notice, selection,
  // accessibility and scrolling when display size/DPI leaves less room.
  // Actions remain outside it, so neither action can be scrolled off-screen.
  auto* notice = new wxTextCtrl(
      &dialog, wxID_ANY, wxString::FromUTF8(kNavigationNotice),
      wxDefaultPosition, wxDefaultSize,
      wxTE_MULTILINE | wxTE_READONLY | wxTE_WORDWRAP | wxBORDER_NONE);
  notice->SetName("Complete navigation safety notice");
  notice->SetFont(UiFont(dialog, 15));
  notice->SetForegroundColour(Colour(palette.primary));
  notice->SetBackgroundColour(Colour(palette.background));
  notice->SetInsertionPoint(0);
  layout->Add(notice, 1, wxEXPAND | wxALL, dialog.FromDIP(24));

  auto* actions = new wxBoxSizer(wxHORIZONTAL);
  actions->AddStretchSpacer();
  auto* cancel = new XNavButton(&dialog, wxID_CANCEL, "Cancel",
                                "Cancel: quit without accepting");
  auto* agree = new XNavButton(&dialog, wxID_OK, "Agree",
                               "Agree: accept the notice and continue");
  for (auto* button : {cancel, agree}) {
    button->SetLightMode(mode);
    button->SetMinSize(dialog.FromDIP(wxSize(120, 48)));
    actions->Add(button, 0, wxLEFT, dialog.FromDIP(8));
  }
  cancel->SetRole(ButtonRole::Quiet);
  agree->SetRole(ButtonRole::Primary);
  layout->Add(actions, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM,
              dialog.FromDIP(24));
  dialog.SetSizer(layout);

  const int display_index = parent ? wxDisplay::GetFromWindow(parent)
                                   : wxNOT_FOUND;
  const wxDisplay display(display_index == wxNOT_FOUND ? 0 : display_index);
  const auto work = display.GetClientArea();
  const int margin = dialog.FromDIP(16);
  const auto preferred = dialog.FromDIP(wxSize(680, 600));
  const wxSize size(std::min(preferred.x, std::max(1, work.width - 2 * margin)),
                    std::min(preferred.y, std::max(1, work.height - 2 * margin)));
  const auto minimum = dialog.FromDIP(wxSize(360, 320));
  dialog.SetMinSize(wxSize(std::min(minimum.x, size.x),
                           std::min(minimum.y, size.y)));
  dialog.SetSize(wxRect(work.x + (work.width - size.x) / 2,
                        work.y + (work.height - size.y) / 2, size.x, size.y));

  // EndModal alone is not consent. Latch the first explicit button decision;
  // native close, Escape, an unexpected modal return and queued later events
  // cannot turn cancellation into acceptance.
  std::optional<int> decision;
  const auto finish = [&dialog, &decision](int choice) {
    if (decision || !dialog.IsModal()) return;
    decision = choice;
    dialog.EndModal(choice);
  };
  cancel->Bind(wxEVT_BUTTON, [finish](wxCommandEvent&) {
    finish(wxID_CANCEL);
  });
  agree->Bind(wxEVT_BUTTON, [finish](wxCommandEvent&) {
    finish(wxID_OK);
  });
  dialog.Bind(wxEVT_CHAR_HOOK, [finish](wxKeyEvent& event) {
    if (event.GetKeyCode() == WXK_ESCAPE) finish(wxID_CANCEL);
    else event.Skip();
  });
  dialog.Bind(wxEVT_CLOSE_WINDOW, [finish](wxCloseEvent&) {
    finish(wxID_CANCEL);
  });
  dialog.SetEscapeId(wxID_CANCEL);
  cancel->SetFocus();
  dialog.ShowModal();
  return decision && *decision == wxID_OK;
}
}  // namespace opennav::ui

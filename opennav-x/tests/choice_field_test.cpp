#include "ui/ChoiceField.h"

#include <wx/app.h>
#include <wx/frame.h>
#include <wx/panel.h>
#include <wx/popupwin.h>
#include <wx/utils.h>

#include <cstdlib>
#include <iostream>
#include <vector>

class ChoiceTestApp final : public wxApp {
 public:
  bool OnInit() override { return true; }
};
wxIMPLEMENT_APP_NO_MAIN(ChoiceTestApp);

namespace {
using opennav::ui::XNavChoiceField;

void Require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "choice_field_test: " << message << '\n';
    std::exit(1);
  }
}

void Pump() {
  wxTheApp->ProcessPendingEvents();
  wxTheApp->Yield(true);
}

void Key(wxWindow &window, int code) {
  wxKeyEvent event(wxEVT_CHAR_HOOK);
  event.m_keyCode = code;
  window.ProcessWindowEvent(event);
  Pump();
}

wxPopupTransientWindow *PopupOf(wxWindow &top) {
  for (auto child : top.GetChildren())
    if (auto *popup = dynamic_cast<wxPopupTransientWindow *>(child)) return popup;
  return nullptr;
}

wxWindow *PopupRows(wxPopupTransientWindow &popup) {
  auto children = popup.GetChildren();
  return children.empty() ? nullptr : children.front();
}
}  // namespace

int main(int argc, char **argv) {
  if (!wxEntryStart(argc, argv) || !wxTheApp->CallOnInit()) return 2;

  auto *frame = new wxFrame(nullptr, wxID_ANY, "Choice field test");
  auto *field = new XNavChoiceField(frame, wxID_ANY, {"Day", "Dusk", "Night"},
                                    "Display theme");
  field->SetSize(320, 46);
  frame->Show();
  Pump();
  Require(field->GetMinSize().y == field->FromDIP(46),
          "100 percent interface scale uses a 46 pixel field");
  field->SetInterfaceScale(125);
  Require(field->GetMinSize().y == field->FromDIP(52),
          "125 percent interface scale uses a 52 pixel field");
  field->SetInterfaceScale(150);
  Require(field->GetMinSize().y == field->FromDIP(56),
          "150 percent interface scale uses a 56 pixel field");
  field->SetInterfaceScale(100);

  int events = 0;
  field->Bind(wxEVT_CHOICE, [&events](wxCommandEvent &event) {
    ++events;
    Require(event.GetInt() == 1, "choice event carries the committed index");
  });
  field->SetSelection(2);
  Pump();
  Require(field->GetSelection() == 2 && events == 0,
          "programmatic selection is silent");

  Key(*field, WXK_RETURN);
  auto *popup = PopupOf(*frame);
  Require(popup != nullptr, "keyboard activation opens popup");
  auto *rows = PopupRows(*popup);
  Require(rows != nullptr, "popup has a focusable choices surface");
  Key(*rows, WXK_UP);
  Key(*rows, WXK_RETURN);
  Pump();
  Require(field->GetSelection() == 1 && events == 1,
          "keyboard commit emits one choice event");

  Key(*field, WXK_RETURN);
  popup = PopupOf(*frame);
  Require(popup != nullptr, "field reopens after a keyboard commit");
  rows = PopupRows(*popup);
  Require(rows != nullptr, "reopened popup retains its choice surface");
  Key(*rows, WXK_ESCAPE);
  Pump();
  Key(*field, WXK_RETURN);
  Require(PopupOf(*frame) != nullptr, "field reopens after Escape dismissal");

  field->Enable(false);
  Pump();
  Require(PopupOf(*frame) == nullptr, "disabling dismisses the open popup");
  field->Enable(true);
  Key(*field, WXK_RETURN);
  Require(PopupOf(*frame) != nullptr, "field reopens after disable and re-enable");
  Key(*PopupRows(*PopupOf(*frame)), WXK_ESCAPE);
  Pump();

  auto *doomed_parent = new wxPanel(frame);
  auto *doomed = new XNavChoiceField(doomed_parent, wxID_ANY,
                                    {"One", "Two"}, "Doomed field");
  doomed->SetSize(280, 46);
  doomed->Show();
  Key(*doomed, WXK_RETURN);
  Require(PopupOf(*frame) != nullptr, "second popup opens before owner teardown");
  doomed_parent->Destroy();
  Pump();
  Require(PopupOf(*frame) == nullptr, "destroying an owner closes its popup");
  Require(field->GetSelection() == 1 && events == 1,
          "destroying another owner leaves the live choice field intact");

  frame->Destroy();
  Pump();
  wxTheApp->OnExit();
  wxEntryCleanup();
  return 0;
}

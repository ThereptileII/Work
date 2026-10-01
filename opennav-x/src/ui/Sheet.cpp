#include "ui/Sheet.h"
#include "ui/DisplaySizing.h"
#include "ui/Drawer.h"
#include "ui/ProductPanel.h"
#include <wx/dialog.h>
#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/weakref.h>
namespace opennav::ui {
namespace {
int OwnerScale(wxWindow &parent,int requested) {
  if(ValidInterfaceScale(requested))return requested;
  for(auto *owner=&parent;owner;owner=owner->GetParent()) {
    if(auto *product=dynamic_cast<ProductPanel *>(owner))return product->InterfaceScale();
    if(auto *drawer=dynamic_cast<XNavDrawer *>(owner))return drawer->InterfaceScale();
  }
  return 100;
}
}
std::optional<std::vector<std::string>>
EditSheet(wxWindow &parent, LightMode mode, const wxString &title,
          const wxString &detail, std::vector<SheetField> fields,
          const wxString &accept, int scale_percent) {
  scale_percent=OwnerScale(parent,scale_percent);
  wxDialog dialog(&parent, wxID_ANY, title, wxDefaultPosition, wxDefaultSize,
                  wxBORDER_NONE | wxTAB_TRAVERSAL);
  dialog.SetName(title);
  dialog.Bind(wxEVT_CHAR_HOOK,[&dialog](wxKeyEvent &event){
    if(event.GetKeyCode()==WXK_ESCAPE) dialog.EndModal(wxID_CANCEL);
    else event.Skip();
  });
  const auto c = Theme(mode);
  dialog.SetBackgroundColour(Colour(c.elevated));
  const int gap = dialog.FromDIP(16);
  // A prototype drawer is itself a top-level owned window. Use the containing
  // application frame for modal bounds, not the drawer's narrow client area.
  auto *frame = wxGetTopLevelParent(&parent);
  while (frame->GetParent()) frame = wxGetTopLevelParent(frame->GetParent());
  const auto available = frame->GetClientSize();
  const int top = dialog.FromDIP(56), bottom = dialog.FromDIP(56);
  const auto size = wxSize(
      std::min(dialog.FromDIP(520), available.x - dialog.FromDIP(24)),
      std::min(dialog.FromDIP(300 + 100 * static_cast<int>(fields.size())),
               std::max(dialog.FromDIP(160), available.y - top - bottom - dialog.FromDIP(24))));
  auto *outer = new wxBoxSizer(wxVERTICAL);
  auto *content = new XNavScroll(&dialog);
  content->SetBackgroundColour(Colour(c.elevated));
  auto *body = new wxBoxSizer(wxVERTICAL);
  auto label = [&](const wxString &text, int font_size, bool bold) {
    auto *t = new wxStaticText(content, wxID_ANY, text);
    EnableScrollGesture(*t);
    t->SetFont(UiFont(dialog, font_size, bold));
    t->SetForegroundColour(Colour(c.primary));
    t->Wrap(std::min(dialog.FromDIP(450), size.x - 2 * gap));
    body->Add(t, 0, wxEXPAND | wxALL, gap);
  };
  label(title, 24, true);
  label(detail, 14, false);
  std::vector<wxTextCtrl *> inputs;
  for (const auto &f : fields) {
    label(f.label, 12, true);
    const bool multiline = f.maximum > 512;
    auto *t =
        new wxTextCtrl(content, wxID_ANY, f.value, wxDefaultPosition,
                       dialog.FromDIP(wxSize(
                           400, multiline ? 96 : DisplayFieldHeight(scale_percent))),
                       wxBORDER_NONE | (multiline ? wxTE_MULTILINE : 0));
    t->SetName(f.label);
    t->SetFont(
        UiFont(dialog, multiline ? 18 : DisplayFieldFont(scale_percent)));
    t->SetBackgroundColour(Colour(c.background));
    t->SetForegroundColour(Colour(c.primary));
    t->SetMaxLength(f.maximum);
    body->Add(t, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, gap);
    inputs.push_back(t);
  }
  content->SetSizer(body);
  outer->Add(content, 1, wxEXPAND);
  auto *actions = new wxBoxSizer(wxHORIZONTAL);
  std::vector<XNavButton *> scroll_buttons;
  for (int direction : {-1, 1}) {
    auto *b = new XNavButton(&dialog, wxID_ANY, direction < 0 ? "Up" : "Down", "Scroll sheet fields");
    b->SetLightMode(mode);
    b->Bind(wxEVT_BUTTON, [content, direction](wxCommandEvent &) { content->Step(direction); });
    actions->Add(b, 0, wxALL, dialog.FromDIP(4));
    b->SetMinSize(dialog.FromDIP(wxSize(64, 48)));
    b->Hide();
    scroll_buttons.push_back(b);
  }
  for (const auto &pair : std::vector<std::pair<wxString, int>>{
           {"Cancel", wxID_CANCEL}, {accept, wxID_OK}}) {
    auto *b = new XNavButton(&dialog, wxID_ANY, pair.first, pair.first);
    b->SetLightMode(mode);
    b->SetRole(pair.second==wxID_OK?ButtonRole::Primary:ButtonRole::Quiet);
    if (scale_percent == 150) {
      const int height = DisplayActionHeight(scale_percent, 48);
      b->SetMinSize(dialog.FromDIP(wxSize(height, height)));
      b->SetTextSize(DisplayActionFont(scale_percent, 12));
    }
    b->Bind(wxEVT_BUTTON, [&dialog, id = pair.second](wxCommandEvent &) {
      dialog.EndModal(id);
    });
    actions->Add(b, 1, wxALL, gap);
  }
  outer->Add(actions, 0, wxEXPAND);
  dialog.SetSizer(outer);
  // Reserve both the status/alert layers and the fixed navigation/STBY row.
  // A newly arriving alert must not appear behind an already-open sheet.
  dialog.SetSize(size);
  dialog.Move(frame->ClientToScreen(wxPoint((available.x-size.x)/2,
                     top + (available.y-top-bottom-size.y)/2)));
  dialog.Layout();
  content->FitInside();
  const bool overflow = content->CanScroll(1);
  for (auto *b : scroll_buttons) b->Show(overflow);
  dialog.Layout();
  if (!inputs.empty())
    inputs.front()->SetFocus();
  const int result_code=dialog.ShowModal();
  // Owned shaped drawers can retain incompletely invalidated child surfaces
  // after a native MSW modal closes. Invalidate the actual visible UI after
  // this stack dialog is destroyed; never synthesize a capture or chart image.
  parent.CallAfter([owner=wxWeakRef<wxWindow>(&parent)] {
    if(!owner || !owner->IsShownOnScreen())return;
    const auto repaint=[](const auto &self,wxWindow *window)->void {
      if(!window->IsShownOnScreen())return;
      window->Refresh(false);
      for(auto *child:window->GetChildren())if(!child->IsTopLevel())self(self,child);
    };
    repaint(repaint,owner.get());
  });
  if (result_code != wxID_OK)
    return std::nullopt;
  std::vector<std::string> result;
  for (auto *input : inputs)
    result.push_back(input->GetValue().ToStdString(wxConvUTF8));
  return result;
}
bool ConfirmSheet(wxWindow &parent, LightMode mode, const wxString &title,
                  const wxString &detail, const wxString &accept,
                  int scale_percent) {
  return EditSheet(parent, mode, title, detail, {}, accept, scale_percent).has_value();
}
} // namespace opennav::ui

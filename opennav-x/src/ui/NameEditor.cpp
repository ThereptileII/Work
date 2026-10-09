#include "ui/NameEditor.h"
#include "ui/DisplaySizing.h"
#include <wx/dcbuffer.h>
#include <wx/graphics.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/weakref.h>

namespace opennav::ui {
XNavNameEditor::XNavNameEditor(
    wxWindow &parent, const wxString &label, std::string initial,
    LightMode light, int scale, bool editable, Save save,
    std::function<void(application::CommandResult)> result)
    : wxPanel(&parent), draft_(std::move(initial)), save_(std::move(save)),
      result_(std::move(result)), editable_(editable) {
  SetName(label + " editor");
  auto *row = new wxBoxSizer(wxHORIZONTAL);
  // Field surface: the same rounded, bordered treatment the settings fields
  // use, which the prototype specifies for every input. wxTextCtrl has no
  // radius or focus ring of its own, so the frame paints both (SCRUM-347).
  frame_ = new wxPanel(this, wxID_ANY);
  frame_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  frame_->SetMinSize(FromDIP(wxSize(120, DisplayFieldHeight(scale))));
  frame_->Bind(wxEVT_PAINT, [this](wxPaintEvent &) { PaintFrame(); });
  input_ = new wxTextCtrl(frame_, wxID_ANY, wxString::FromUTF8(draft_.Value()),
      wxDefaultPosition, wxDefaultSize, wxBORDER_NONE | wxTE_PROCESS_ENTER);
  input_->SetName(label);
  input_->SetHint(label);
  input_->SetFont(UiFont(*this, DisplayFieldFont(scale)));
  input_->SetMaxLength(128);
  auto *inset = new wxBoxSizer(wxHORIZONTAL);
  inset->Add(input_, 1, wxALIGN_CENTER_VERTICAL | wxLEFT | wxRIGHT, FromDIP(13));
  frame_->SetSizer(inset);
  row->Add(frame_, 1, wxEXPAND | wxRIGHT, FromDIP(8));
  save_button_ = new XNavButton(this, wxID_ANY, "Save", "Save " + label.Lower());
  cancel_button_ = new XNavButton(this, wxID_ANY, "Cancel", "Cancel " + label.Lower());
  for (auto *button : {save_button_, cancel_button_}) {
    button->SetMinSize(FromDIP(wxSize(84, DisplayActionHeight(scale, 48))));
    button->SetInterfaceScale(scale);
    row->Add(button, 0, wxLEFT, FromDIP(8));
  }
  save_button_->SetRole(ButtonRole::Primary);
  cancel_button_->SetRole(ButtonRole::Quiet);
  SetSizer(row);
  input_->Bind(wxEVT_TEXT, [this](wxCommandEvent &) {
    draft_.Set(input_->GetValue().ToStdString(wxConvUTF8)); Buttons();
  });
  input_->Bind(wxEVT_TEXT_ENTER, [this](wxCommandEvent &) { SaveDraft(); });
  input_->Bind(wxEVT_CHAR_HOOK, [this](wxKeyEvent &event) {
    if (event.GetKeyCode() == WXK_ESCAPE) CancelDraft(); else event.Skip();
  });
  // Focus lives on the inner control, so the frame has to be told when to
  // draw the ring; wxWidgets gives the panel no focus event of its own.
  input_->Bind(wxEVT_SET_FOCUS, [this](wxFocusEvent &event) {
    focused_ = true; if (frame_) frame_->Refresh(false); event.Skip();
  });
  input_->Bind(wxEVT_KILL_FOCUS, [this](wxFocusEvent &event) {
    focused_ = false; if (frame_) frame_->Refresh(false); event.Skip();
  });
  save_button_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { SaveDraft(); });
  cancel_button_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { CancelDraft(); });
  Present(light, true);
}
void XNavNameEditor::Buttons() {
  input_->Enable(editable_ && live_);
  save_button_->Enable(editable_ && live_ && draft_.Changed() &&
                       application::ValidNavigationName(draft_.Value()) && bool(save_));
  cancel_button_->Enable(draft_.Changed());
  if (frame_) frame_->Refresh(false);
}
void XNavNameEditor::PaintFrame() {
  if (!frame_) return;
  const auto palette = Theme(light_);
  const bool enabled = editable_ && live_;
  // Empty is not an error: the name simply is not saveable yet. Only a
  // non-empty value that the validator rejects earns the attention ink.
  const bool invalid = enabled && !draft_.Value().empty() &&
                       !application::ValidNavigationName(draft_.Value());
  wxAutoBufferedPaintDC dc(frame_);
  dc.SetBackground(wxBrush(Colour(palette.background)));
  dc.Clear();
  std::unique_ptr<wxGraphicsContext> graphics(wxGraphicsContext::Create(dc));
  if (!graphics) return;
  const auto size = frame_->GetClientSize();
  const double width = invalid || (focused_ && enabled) ? 2. : 1.;
  const auto ink = invalid ? palette.attention
                   : focused_ && enabled ? palette.accent : palette.border;
  graphics->SetBrush(wxBrush(Colour(enabled ? palette.surface : palette.background)));
  graphics->SetPen(wxPen(Colour(ink), static_cast<int>(width)));
  const double inset = width / 2.;
  graphics->DrawRoundedRectangle(inset, inset, size.x - width, size.y - width,
                                 FromDIP(8));
}
void XNavNameEditor::Present(LightMode light, bool live) {
  live_ = live;
  light_ = light;
  const auto palette = Theme(light);
  const bool enabled = editable_ && live_;
  SetBackgroundColour(Colour(palette.background));
  // The control fills the frame's interior, so it has to carry the same ink;
  // any difference shows as a rectangle inside the rounded border.
  const auto fill = enabled ? palette.surface : palette.background;
  if (frame_) frame_->SetBackgroundColour(Colour(fill));
  input_->SetBackgroundColour(Colour(fill));
  input_->SetForegroundColour(Colour(enabled ? palette.primary : palette.muted));
  save_button_->SetLightMode(light); cancel_button_->SetLightMode(light);
  Buttons();
}
void XNavNameEditor::CancelDraft() {
  draft_.Cancel(); input_->ChangeValue(wxString::FromUTF8(draft_.Value())); Buttons();
}
void XNavNameEditor::SaveDraft() {
  if (!save_button_->IsEnabled()) return;
  const auto result = draft_.Save(save_);
  Buttons();
  // A successful callback can rebuild its owning product page. Dispatch only
  // after this field's event handler has returned, never destroy it mid-save.
  if (result_) {
    const auto notify = result_;
    CallAfter([notify, result] { notify(result); });
  }
}
bool XNavNameEditor::PreserveDrafts(wxWindow &owner, LightMode light, bool live) {
  bool changed = false;
  for (auto *child : owner.GetChildren()) {
    if (auto *editor = dynamic_cast<XNavNameEditor *>(child)) {
      editor->Present(light, live);
      changed |= editor->DraftChanged();
    } else if (!child->IsTopLevel()) changed |= PreserveDrafts(*child, light, live);
  }
  return changed;
}
std::optional<std::vector<std::string>> EditNavigationNameSheet(
    wxWindow &parent, LightMode light, const wxString &title,
    const wxString &detail, const std::string &name, const std::string &description,
    const wxString &accept, int scale) {
  std::vector<SheetField> fields{{"Name", wxString::FromUTF8(name), 128},
                                {"Description", wxString::FromUTF8(description), 2048}};
  wxString message = detail;
  for (;;) {
    auto edited = EditSheet(parent, light, title, message, fields, accept, scale);
    if (!edited || application::ValidNavigationName((*edited)[0])) return edited;
    fields[0].value = wxString::FromUTF8((*edited)[0]);
    fields[1].value = wxString::FromUTF8((*edited)[1]);
    message = "Enter a non-empty, shorter name. Your draft has been kept.";
  }
}
} // namespace opennav::ui

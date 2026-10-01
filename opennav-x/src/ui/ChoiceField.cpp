#include "ui/ChoiceField.h"

#include "ui/Controls.h"
#include "ui/PrototypeIcons.h"

#include <wx/bmpbndl.h>
#include <wx/dcbuffer.h>
#include <wx/popupwin.h>
#include <wx/sizer.h>

#include <algorithm>
#include <utility>

namespace opennav::ui {

class XNavChoiceField::Popup final : public wxPopupTransientWindow {
 public:
  explicit Popup(XNavChoiceField &owner)
      : wxPopupTransientWindow(wxGetTopLevelParent(&owner), wxBORDER_NONE), owner_(&owner),
        rows_(new wxPanel(this, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                          wxBORDER_NONE)) {
    rows_->SetBackgroundStyle(wxBG_STYLE_PAINT);
    rows_->Bind(wxEVT_PAINT, &Popup::PaintRows, this);
    rows_->Bind(wxEVT_MOTION, &Popup::OnMotion, this);
    rows_->Bind(wxEVT_LEAVE_WINDOW, [this](wxMouseEvent &) {
      hovered_ = wxNOT_FOUND;
      rows_->Refresh(false);
    });
    rows_->Bind(wxEVT_LEFT_UP, &Popup::OnClick, this);
    rows_->Bind(wxEVT_CHAR_HOOK, &Popup::OnKey, this);
    Bind(wxEVT_CHAR_HOOK, &Popup::OnKey, this);
    EnableTouchEvents(wxTOUCH_VERTICAL_PAN_GESTURE);

    const auto visible = std::min<std::size_t>(owner.choices_.size(), kMaxRows);
    rows_->SetMinSize(wxSize(owner.FromDIP(260), owner.FromDIP(
        static_cast<int>(visible) * kRowHeight)));
    auto *sizer = new wxBoxSizer(wxVERTICAL);
    sizer->Add(rows_, 1, wxEXPAND);
    SetSizerAndFit(sizer);
    focus_ = owner.selection_ == wxNOT_FOUND ? 0 : owner.selection_;
  }

  void SetOwner(XNavChoiceField *owner) { owner_ = owner; }
  void FocusRows() { rows_->SetFocus(); }
  void ShowPopup() { wxPopupTransientWindow::Popup(rows_); }
  void DismissAndFinish() {
    Dismiss();
    Finish();
  }
  int VisibleRows() const {
    return static_cast<int>(std::min<std::size_t>(owner_ ? owner_->choices_.size() : 0,
                                                  kMaxRows));
  }

 protected:
  void OnDismiss() override {
    Finish();
  }

 private:
  static constexpr int kRowHeight = 44;
  static constexpr std::size_t kMaxRows = 6;

  void Finish() {
    if (finished_) return;
    finished_ = true;
    if (owner_) owner_->ClosePopup(this);
    owner_ = nullptr;
    Destroy();
  }

  void AdjustTop() {
    const int rows = VisibleRows();
    if (focus_ < top_) top_ = focus_;
    if (focus_ >= top_ + rows) top_ = focus_ - rows + 1;
    top_ = std::max(0, top_);
  }
  void PaintRows(wxPaintEvent &) {
    wxAutoBufferedPaintDC dc(rows_);
    if (!owner_) { dc.Clear(); return; }
    const auto palette = Theme(owner_ ? owner_->mode_ : LightMode::Day);
    const auto size = rows_->GetClientSize();
    dc.SetBackground(wxBrush(Colour(palette.elevated)));
    dc.Clear();
    const int row_height = rows_->FromDIP(kRowHeight);
    const int count = VisibleRows();
    const int shown = std::min(count, static_cast<int>(owner_->choices_.size()) - top_);
    for (int row = 0; row < shown; ++row) {
      const int index = top_ + row;
      const int y = row * row_height;
      if (index == focus_ || index == hovered_) {
        dc.SetPen(*wxTRANSPARENT_PEN);
        dc.SetBrush(wxBrush(Colour(palette.selected)));
        dc.DrawRoundedRectangle(rows_->FromDIP(4), y + rows_->FromDIP(2),
                                size.x - rows_->FromDIP(8),
                                row_height - rows_->FromDIP(4), rows_->FromDIP(6));
      }
      dc.SetFont(UiFont(*rows_, owner_->FontPixels()));
      dc.SetTextForeground(Colour(index == focus_ ? palette.primary : palette.secondary));
      dc.DrawText(owner_->choices_[index], rows_->FromDIP(13),
                  y + (row_height - dc.GetCharHeight()) / 2);
      if (index == owner_->selection_) {
        const int x = size.x - rows_->FromDIP(24);
        dc.SetPen(wxPen(Colour(palette.accent), rows_->FromDIP(2)));
        dc.DrawLine(x, y + row_height / 2, x + rows_->FromDIP(4), y + row_height / 2 + rows_->FromDIP(4));
        dc.DrawLine(x + rows_->FromDIP(4), y + row_height / 2 + rows_->FromDIP(4),
                    x + rows_->FromDIP(11), y + row_height / 2 - rows_->FromDIP(4));
      }
    }
    dc.SetBrush(*wxTRANSPARENT_BRUSH);
    dc.SetPen(wxPen(Colour(palette.border), rows_->FromDIP(1)));
    dc.DrawRoundedRectangle(0, 0, size.x, size.y, rows_->FromDIP(8));
  }
  void OnMotion(wxMouseEvent &event) {
    if (!owner_) return;
    const int row = top_ + event.GetY() / std::max(1, rows_->FromDIP(kRowHeight));
    if (row >= 0 && row < static_cast<int>(owner_->choices_.size()) && hovered_ != row) {
      hovered_ = row;
      focus_ = row;
      rows_->Refresh(false);
    }
  }
  void OnClick(wxMouseEvent &event) {
    if (!owner_) return;
    const int row = top_ + event.GetY() / std::max(1, rows_->FromDIP(kRowHeight));
    if (owner_->IsEnabled() && row >= 0 && row < static_cast<int>(owner_->choices_.size()))
      owner_->CommitSelection(row);
    DismissAndFinish();
  }
  void OnKey(wxKeyEvent &event) {
    if (!owner_) { event.Skip(); return; }
    const int key = event.GetKeyCode();
    if (key == WXK_ESCAPE) {
      DismissAndFinish();
    } else if (key == WXK_RETURN || key == WXK_NUMPAD_ENTER || key == WXK_SPACE) {
      if (!owner_->IsEnabled()) { DismissAndFinish(); return; }
      owner_->CommitSelection(focus_);
      DismissAndFinish();
    } else if (key == WXK_UP || key == WXK_DOWN || key == WXK_HOME || key == WXK_END) {
      if (key == WXK_UP) focus_ = std::max(0, focus_ - 1);
      if (key == WXK_DOWN) focus_ = std::min(static_cast<int>(owner_->choices_.size()) - 1, focus_ + 1);
      if (key == WXK_HOME) focus_ = 0;
      if (key == WXK_END) focus_ = static_cast<int>(owner_->choices_.size()) - 1;
      AdjustTop();
      rows_->Refresh(false);
    } else {
      event.Skip();
    }
  }

  XNavChoiceField *owner_;
  wxPanel *rows_;
  int focus_ = 0;
  int top_ = 0;
  int hovered_ = wxNOT_FOUND;
  bool finished_ = false;
};

XNavChoiceField::XNavChoiceField(wxWindow *parent, wxWindowID id,
                                 const std::vector<wxString> &choices,
                                 const wxString &accessibleName)
    : wxControl(parent, id, wxDefaultPosition, wxDefaultSize, wxBORDER_NONE),
      choices_(choices) {
  SetName(accessibleName);
  if (!choices_.empty()) wxControl::SetLabel(choices_.front());
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  if (!choices_.empty()) selection_ = 0;
  SetMinSize(FromDIP(wxSize(-1, 46)));
  Bind(wxEVT_PAINT, &XNavChoiceField::Paint, this);
  Bind(wxEVT_ENTER_WINDOW, [this](wxMouseEvent &event) {
    hovered_ = true; Refresh(false); event.Skip();
  });
  Bind(wxEVT_LEAVE_WINDOW, [this](wxMouseEvent &event) {
    hovered_ = false; Refresh(false); event.Skip();
  });
  Bind(wxEVT_SET_FOCUS, [this](wxFocusEvent &event) {
    keyboard_focus_ = true; Refresh(false); event.Skip();
  });
  Bind(wxEVT_KILL_FOCUS, [this](wxFocusEvent &event) {
    keyboard_focus_ = false; Refresh(false); event.Skip();
  });
  Bind(wxEVT_LEFT_UP, [this](wxMouseEvent &) { OpenPopup(); });
  EnableScrollGesture(*this);
  Bind(wxEVT_CHAR_HOOK, [this](wxKeyEvent &event) {
    const int key = event.GetKeyCode();
    if ((key == WXK_RETURN || key == WXK_NUMPAD_ENTER || key == WXK_SPACE ||
         key == WXK_DOWN || key == WXK_UP) && IsEnabled()) {
      OpenPopup();
    } else {
      event.Skip();
    }
  });
}

XNavChoiceField::~XNavChoiceField() {
  if (popup_) {
    auto *popup = popup_;
    popup_ = nullptr;
    popup->SetOwner(nullptr);
    popup->DismissAndFinish();
  }
}

void XNavChoiceField::SetSelection(int selection) {
  selection = ClosestSelection(selection);
  if (selection_ == selection) return;
  selection_ = selection;
  wxControl::SetLabel(selection_ == wxNOT_FOUND ? wxString{} : choices_[selection_]);
  Refresh(false);
}

bool XNavChoiceField::Enable(bool enable) {
  if (!enable && popup_) popup_->DismissAndFinish();
  const bool changed = wxControl::Enable(enable);
  Refresh(false);
  return changed;
}

int XNavChoiceField::ClosestSelection(int selection) const {
  if (choices_.empty() || selection == wxNOT_FOUND) return wxNOT_FOUND;
  return std::clamp(selection, 0, static_cast<int>(choices_.size()) - 1);
}

void XNavChoiceField::SetLightMode(LightMode mode) {
  if (mode_ == mode) return;
  mode_ = mode;
  Refresh(false);
  if (popup_) popup_->Refresh(false);
}

void XNavChoiceField::SetInterfaceScale(int percent) {
  if (percent != 100 && percent != 125 && percent != 150) return;
  if (interface_scale_ == percent) return;
  interface_scale_ = percent;
  const int height = percent == 100 ? 46 : percent == 125 ? 52 : 56;
  SetMinSize(FromDIP(wxSize(-1, height)));
  Refresh(false);
}

int XNavChoiceField::FontPixels() const {
  return interface_scale_ == 150 ? 18 : 16;
}

void XNavChoiceField::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(this);
  const auto palette = Theme(mode_);
  const auto size = GetClientSize();
  const auto parent_background = GetParent()->GetBackgroundColour();
  dc.SetBackground(wxBrush(parent_background.IsOk() ? parent_background
                                                    : Colour(palette.background)));
  dc.Clear();
  const int inset = FromDIP(1);
  dc.SetPen(wxPen(Colour(palette.border), FromDIP(1)));
  dc.SetBrush(wxBrush(Colour(palette.background)));
  dc.DrawRoundedRectangle(inset, inset, size.x - inset * 2, size.y - inset * 2,
                          FromDIP(8));
  if (hovered_ && IsEnabled()) {
    dc.SetPen(*wxTRANSPARENT_PEN);
    dc.SetBrush(wxBrush(Colour(palette.surface)));
    dc.DrawRoundedRectangle(inset + FromDIP(1), inset + FromDIP(1),
                            size.x - FromDIP(4), size.y - FromDIP(4), FromDIP(7));
  }
  if (keyboard_focus_ && IsEnabled()) {
    dc.SetBrush(*wxTRANSPARENT_BRUSH);
    dc.SetPen(wxPen(Colour(palette.accent), FromDIP(2)));
    dc.DrawRoundedRectangle(FromDIP(2), FromDIP(2), size.x - FromDIP(4),
                            size.y - FromDIP(4), FromDIP(7));
  }
  dc.SetFont(UiFont(*this, FontPixels()));
  dc.SetTextForeground(Colour(IsEnabled() ? palette.primary : palette.muted));
  if (selection_ != wxNOT_FOUND)
    dc.DrawText(choices_[selection_], FromDIP(13),
                (size.y - dc.GetCharHeight()) / 2);
  const auto icon = wxString::Format(
      "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"24\" height=\"24\">"
      "<path d=\"%s\" transform=\"rotate(90 12 12)\" fill=\"none\" "
      "stroke=\"#%06x\" stroke-width=\"1.65\" stroke-linecap=\"round\" "
      "stroke-linejoin=\"round\"/></svg>",
      wxString::FromUTF8(PrototypeIconPath(XNavIcon::Chevron)),
      IsEnabled() ? palette.secondary : palette.muted);
  const wxSize icon_size = FromDIP(wxSize(16, 16));
  const auto bitmap = wxBitmapBundle::FromSVG(icon.utf8_str(), icon_size).GetBitmap(icon_size);
  if (bitmap.IsOk())
    dc.DrawBitmap(bitmap, size.x - FromDIP(28),
                  (size.y - icon_size.y) / 2, true);
}

void XNavChoiceField::OpenPopup() {
  if (!IsEnabled() || choices_.empty()) return;
  if (popup_) {
    popup_->SetFocus();
    return;
  }
  popup_ = new Popup(*this);
  popup_->SetBackgroundColour(Colour(Theme(mode_).elevated));
  const auto position = ClientToScreen(wxPoint(0, GetClientSize().y + FromDIP(2)));
  popup_->SetSize(wxSize(GetClientSize().x,
                         popup_->GetBestSize().y));
  const wxPoint place(position.x, position.y);
  popup_->Position(place, wxSize(0, 0));
  popup_->ShowPopup();
  popup_->FocusRows();
  popup_->Refresh(false);
}

void XNavChoiceField::ClosePopup(Popup *popup) {
  if (popup_ == popup) popup_ = nullptr;
}

void XNavChoiceField::CommitSelection(int selection) {
  selection = ClosestSelection(selection);
  if (selection == wxNOT_FOUND || selection == selection_) return;
  selection_ = selection;
  wxControl::SetLabel(choices_[selection_]);
  Refresh(false);
  wxCommandEvent event(wxEVT_CHOICE, GetId());
  event.SetEventObject(this);
  event.SetInt(selection_);
  event.SetString(choices_[selection_]);
  wxPostEvent(this, event);
}

}  // namespace opennav::ui

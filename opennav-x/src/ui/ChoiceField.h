#pragma once

#include "ui/Theme.h"

#include <wx/control.h>

#include <vector>

namespace opennav::ui {

// A painted XNav select field with a themed, transient list popup. Programmatic
// selection changes are silent; wxEVT_CHOICE reports committed user changes.
class XNavChoiceField final : public wxControl {
 public:
  XNavChoiceField(wxWindow *parent, wxWindowID id,
                  const std::vector<wxString> &choices,
                  const wxString &accessibleName);
  ~XNavChoiceField() override;

  void SetSelection(int selection);
  int GetSelection() const { return selection_; }
  bool Enable(bool enable = true) override;
  void SetLightMode(LightMode mode);
  void SetInterfaceScale(int percent);

 private:
  class Popup;
  friend class Popup;

  void Paint(wxPaintEvent &event);
  void OpenPopup();
  void ClosePopup(Popup *popup);
  void CommitSelection(int selection);
  int ClosestSelection(int selection) const;
  int FontPixels() const;

  std::vector<wxString> choices_;
  int selection_ = wxNOT_FOUND;
  int interface_scale_ = 100;
  LightMode mode_ = LightMode::Day;
  bool hovered_ = false;
  bool keyboard_focus_ = false;
  Popup *popup_ = nullptr;
};

}  // namespace opennav::ui

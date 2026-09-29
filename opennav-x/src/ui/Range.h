#pragma once
#include "ui/Controls.h"
#include <functional>

namespace opennav::ui {
// Native, owner-drawn stepped range. Changing a value is never a device command.
class XNavRange final : public wxControl {
public:
  XNavRange(wxWindow *parent,const wxString &name,int minimum,int maximum,int step,int value);
  void SetValue(int value);
  int GetValue() const { return value_; }
  void SetLight(LightMode light) { light_=light;Refresh(false); }
  std::function<void(int)> on_change;
private:
  void Paint(wxPaintEvent &);
  void Choose(int x);
  int minimum_,maximum_,step_,value_;
  LightMode light_=LightMode::Day;
};
}

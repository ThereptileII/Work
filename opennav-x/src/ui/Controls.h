#pragma once

#include "ui/Theme.h"
#include "vessel/VesselState.h"

#include <wx/control.h>
#include <wx/panel.h>

namespace opennav::ui {

wxColour Colour(std::uint32_t rgb);
wxFont UiFont(wxWindow& window, int pixels, bool bold = false);

class XNavButton final : public wxControl {
 public:
  XNavButton(wxWindow* parent, wxWindowID id, const wxString& label,
             const wxString& accessible_name);
  void SetLightMode(LightMode mode);
  void SetLabel(const wxString &label) override;

 private:
  void Paint(wxPaintEvent& event);
  void Activate();
  LightMode mode_ = LightMode::Day;
  bool pressed_ = false;
};

class XNavDataValue final : public wxPanel {
 public:
  XNavDataValue(wxWindow* parent, const wxString& label, const wxString& unit,
                int decimals = 1);
  void SetLightMode(LightMode mode);
  void SetReading(const vessel::Sample& sample, vessel::Time now);

 private:
  void Paint(wxPaintEvent& event);
  LightMode mode_ = LightMode::Day;
  wxString label_, unit_;
  int decimals_;
  vessel::Sample sample_;
  vessel::Assessment reading_;
};

}  // namespace opennav::ui

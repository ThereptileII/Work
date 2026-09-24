#pragma once
#include "ui/Controls.h"
#include <optional>
#include <vector>
namespace opennav::ui {
struct SheetField {
  wxString label, value;
  int maximum = 2048;
};
// Reusable touch sheet. Native text editing is borderless and themed; primary
// actions, hierarchy and confirmation are OpenNav components.
std::optional<std::vector<std::string>>
EditSheet(wxWindow &parent, LightMode mode, const wxString &title,
          const wxString &detail, std::vector<SheetField> fields,
          const wxString &accept = "Save");
bool ConfirmSheet(wxWindow &parent, LightMode mode, const wxString &title,
                  const wxString &detail, const wxString &accept);
} // namespace opennav::ui

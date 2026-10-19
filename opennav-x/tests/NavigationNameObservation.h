#pragma once
#include "ui/NameEditor.h"
#include "ui/ProductPanel.h"
#include <vector>

namespace opennav::test {
// Fixture observation only: a name is the actual native field value in the
// selected detail page, not an arbitrary matching label elsewhere in the frame.
inline bool HasVisibleNavigationName(wxWindow &owner, ui::ProductPage kind,
                                     const wxString &expected) {
  const bool route = kind == ui::ProductPage::RouteDetail;
  if ((!route && kind != ui::ProductPage::WaypointDetail) ||
      !owner.IsShownOnScreen() || !owner.IsEnabled()) return false;
  const std::string title = route ? "Route detail" : "Waypoint detail";
  const wxString name = route ? "Route name" : "Waypoint name";
  std::vector<ui::ProductPanel *> pages;
  const auto find_page = [&](const auto &self, wxWindow &window) -> void {
    for (auto *child : window.GetChildren()) {
      if (child->IsTopLevel()) continue;
      if (auto *page = dynamic_cast<ui::ProductPanel *>(child);
          page && page->IsShownOnScreen() && page->PageTitle() == title)
        pages.push_back(page);
      self(self, *child);
    }
  };
  find_page(find_page, owner);
  if (pages.size() != 1 || !pages[0]->IsEnabled()) return false;
  std::vector<wxTextCtrl *> fields;
  const auto find_field = [&](const auto &self, wxWindow &window) -> void {
    for (auto *child : window.GetChildren()) {
      if (child->IsTopLevel()) continue;
      if (auto *field = dynamic_cast<wxTextCtrl *>(child);
          field && field->GetName() == name) fields.push_back(field);
      self(self, *child);
    }
  };
  find_field(find_field, *pages[0]);
  if (fields.size() != 1) return false;
  auto *field = fields[0];
  auto *frame = field->GetParent();
  auto *editor = frame ? dynamic_cast<ui::XNavNameEditor *>(frame->GetParent()) : nullptr;
  return editor && editor->GetName() == name + " editor" &&
         field->IsShownOnScreen() && field->IsEnabled() &&
         pages[0]->GetScreenRect().Contains(field->GetScreenRect()) &&
         field->GetValue() == expected;
}
} // namespace opennav::test

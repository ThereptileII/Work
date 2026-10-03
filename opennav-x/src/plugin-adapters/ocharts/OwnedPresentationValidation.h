#pragma once
#include <pugixml.hpp>
#include <wx/filename.h>
#include <wx/image.h>
#include <cstring>

namespace skager::ocharts {
// The compiled resource digest is the authority for contents. This second
// boundary proves that this renderer can parse the XML and decode every atlas
// before installing any tables. It performs no GL work and changes no CWD.
inline bool ValidateOwnedPresentation(const pugi::xml_document& doc,
                                      const wxString& directory) {
  const auto root = doc.child("chartsymbols");
  if (!root) return false;
  for (const auto* section : {"color-tables", "lookups", "line-styles",
                              "patterns", "symbols"})
    if (!root.child(section).first_child()) return false;
  bool day=false, dusk=false, night=false;
  for (auto table : root.child("color-tables").children("color-table")) {
    const char* name=table.attribute("name").value();
    const char* file=table.child("graphics-file").attribute("name").value();
    // Only atlases covered by the compiled digest list may be loaded.
    if (std::strcmp(file,"rastersymbols-day.png") &&
        std::strcmp(file,"rastersymbols-dusk.png") &&
        std::strcmp(file,"rastersymbols-dark.png")) return false;
    wxImage atlas;
    if (!atlas.LoadFile(wxFileName(directory,wxString::FromUTF8(file)).GetFullPath(),
                        wxBITMAP_TYPE_PNG) || !atlas.IsOk() ||
        atlas.GetWidth()<=0 || atlas.GetHeight()<=0) return false;
    day |= !std::strcmp(name,"DAY_BRIGHT");
    dusk |= !std::strcmp(name,"DUSK");
    night |= !std::strcmp(name,"NIGHT");
  }
  return day && dusk && night;
}
} // namespace skager::ocharts

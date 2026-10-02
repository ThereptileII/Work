#pragma once
#include "picosha2.h"
#include <wx/ffile.h>
#include <wx/string.h>
#include <wx/image.h>
#include <vector>
namespace opennav::integration {
// Verify the shipped input before marking its decoded icon as owned. This is
// never inferred from a key: UserIcons and plugin replacements revoke ownership.
inline bool IsPinnedRouteDiamond(const wxString &path) {
  wxFFile file(path, "rb");
  if (!file.IsOpened() || file.Length() != 493) return false;
  std::vector<unsigned char> bytes(493);
  if (file.Read(bytes.data(), bytes.size()) != bytes.size()) return false;
  return picosha2::hash256_hex_string(bytes) ==
      "ebb4c7e751b9d21db60d1bbf1041eb5434616b6b90570cd7ca478480263e8a9d";
}
// The upstream disk SVG cache is keyed by path and dimensions, not content.
// A valid source hash cannot by itself establish what was actually decoded.
inline bool SameRouteDiamondPixels(const wxImage &loaded, const wxImage &fresh) {
  if (!loaded.IsOk() || !fresh.IsOk() || loaded.GetSize()!=fresh.GetSize() ||
      loaded.HasMask() || fresh.HasMask() || loaded.HasAlpha()!=fresh.HasAlpha())
    return false;
  const auto count=static_cast<std::size_t>(loaded.GetWidth())*loaded.GetHeight();
  const auto *a=loaded.GetData(), *b=fresh.GetData();
  const auto *aa=loaded.GetAlpha(), *ba=fresh.GetAlpha();
  for (std::size_t i=0;i<count;++i) {
    if (aa && aa[i]!=ba[i]) return false;
    if ((!aa || aa[i]) && (a[i*3]!=b[i*3] || a[i*3+1]!=b[i*3+1] || a[i*3+2]!=b[i*3+2]))
      return false;
  }
  return true;
}
} // namespace opennav::integration

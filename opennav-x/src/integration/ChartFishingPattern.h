#pragma once

#include <cmath>
#include <cstring>
#include <wx/image.h>

namespace opennav::integration {

// This is an owned AP alias, never the same-name FSHFAC03 point or stock AP.
template <class Rule>
bool FishingPatternEligible(bool enabled, const Rule* rule) {
  if (!enabled || !rule || std::memcmp(rule->name.PANM, "XNFISH03", 8) ||
      rule->RCID != 60017 || rule->definition.PADF != 'V' ||
      rule->fillType.PATP != 'S' || rule->spacing.PASP != 'C') return false;
  const auto& p = rule->pos.patt;
  return p.bnbox_w.PAHL == 604 && p.bnbox_h.PAVL == 151 &&
         p.bnbox_x.PBXC == 3135 && p.bnbox_y.PBXR == 2173 &&
         p.pivot_x.PACL == 4643 && p.pivot_y.PARW == 2168 &&
         p.minDist.PAMI == 2000 && p.maxDist.PAMA == 10000;
}

struct FishingPatternPlacement {
  int width = 0, height = 0, side = 0, x = 0, y = 0, inset_x = 0, inset_y = 0;
  double old_center_x = 0, old_center_y = 0;
};

inline bool ComposeFishingPattern(double ppmm, const wxImage& source,
                                 wxImage& result,
                                 FishingPatternPlacement* observed = nullptr) {
  if (!std::isfinite(ppmm) || ppmm < 1 || ppmm > 24 ||
      !source.IsOk() || source.GetWidth() != 24 || source.GetHeight() != 24 ||
      !source.HasAlpha()) return false;
  // Exactly the original vector cell calculation: bbox expanded to pivot,
  // then minDist added on both axes, floating-point scale and truncation +1.
  const float fsf = 100 / ppmm;
  FishingPatternPlacement p;
  p.width = static_cast<int>(3508.0 / fsf) + 1;
  p.height = static_cast<int>(2156.0 / fsf) + 1;
  // Prototype logical scale at96dpi; preserve the original physical ppmm
  // pattern scaling at other displays, never squeeze the portrait diamond.
  p.side = static_cast<int>(std::lround(24 * ppmm / (96.0 / 25.4)));
  if (p.side < 1 || p.side > 153 || p.width > 1024 || p.height > 1024) return false;
  wxImage glyph = source.Scale(p.side, p.side, wxIMAGE_QUALITY_HIGH);
  if (!glyph.IsOk() || !glyph.HasAlpha()) return false;
  p.old_center_x = 302.0 / fsf + 1;
  p.old_center_y = 80.5 / fsf + 1;
  p.x = static_cast<int>(std::lround(p.old_center_x - p.side / 2.0));
  p.y = static_cast<int>(std::lround(p.old_center_y - p.side / 2.0));
  int left = p.side, top = p.side, right = -1, bottom = -1;
  for (int y = 0; y < p.side; ++y) for (int x = 0; x < p.side; ++x) {
    if (!glyph.GetAlpha(x,y)) continue;
    if (x < left) left = x;
    if (y < top) top = y;
    if (x > right) right = x;
    if (y > bottom) bottom = y;
  }
  if (right < 0) return false;
  // Keep the old motif centre, with only the minimum integer inset needed
  // to retain every antialiased pixel, bounded by one nominal96dpi pixel
  // scaled/rounded up for physical DPI. Lattice and polygon anchor stay put.
  if (p.x + left < 0) p.inset_x = -(p.x + left);
  if (p.y + top < 0) p.inset_y = -(p.y + top);
  if (observed) *observed = p;
  const int inset_limit = static_cast<int>(std::ceil(ppmm / (96.0 / 25.4)));
  if (p.inset_x > inset_limit || p.inset_y > inset_limit) return false;
  p.x += p.inset_x;
  p.y += p.inset_y;
  if (p.x + right >= p.width || p.y + bottom >= p.height) return false;
  wxImage cell(p.width, p.height);
  if (!cell.IsOk() || !cell.GetData()) return false;
  cell.InitAlpha();
  if (!cell.HasAlpha() || !cell.GetAlpha()) return false;
  std::memset(cell.GetData(), 0, p.width * p.height * 3);
  std::memset(cell.GetAlpha(), 0, p.width * p.height);
  for (int y = top; y <= bottom; ++y) for (int x = left; x <= right; ++x) {
    const unsigned char alpha = glyph.GetAlpha(x,y);
    if (!alpha) continue;
    cell.SetRGB(p.x+x,p.y+y,glyph.GetRed(x,y),glyph.GetGreen(x,y),glyph.GetBlue(x,y));
    cell.SetAlpha(p.x+x,p.y+y,alpha);
  }
  result = cell;
  if (observed) *observed = p;
  return true;
}

}  // namespace opennav::integration

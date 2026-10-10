#pragma once
#include <cstring>
#include "s52s57.h"

namespace opennav::integration {
// A light-support tower: exactly one CATLMK 17 (tower) and one FUNCTN 33
// (light support) -- what a chart calls a lighthouse (fyr). A generic tower,
// or any other or additional category/function, is never one.
inline bool LightSupportTower(const S57Obj* object) {
  if (!object || object->Primitive_type != GEO_POINT ||
      std::memcmp(object->FeatureName, "LNDMRK", 6) ||
      object->n_attr < 2 || object->n_attr > 4096 || !object->att_array ||
      !object->attVal || object->attVal->GetCount() != unsigned(object->n_attr))
    return false;
  unsigned seen = 0;
  for (int i = 0; i < object->n_attr; ++i) {
    const char* name = object->att_array + i * 6;
    const unsigned bit = !std::memcmp(name, "CATLMK", 6) ? 1
                       : !std::memcmp(name, "FUNCTN", 6) ? 2 : 0;
    if (!bit) continue;
    if (seen & bit) return false;
    seen |= bit;
    const auto* value = object->attVal->Item(i);
    if (!value || !value->value || value->valType != OGR_STR ||
        std::strncmp(static_cast<const char*>(value->value), bit == 1 ? "17" : "33", 3))
      return false;
  }
  return seen == 3;
}
// The pinned simplified LNDMRK light-support tower selectors (CATLMK17,
// FUNCTN33) paint the prototype lighthouse (XNLIT013, circle and four rays)
// instead of a tower silhouette: the owner's design shows a fyr as that one
// mark, with its sectors (boat feedback 2026-10-10, superseding SCRUM-308's
// re-inked tower). A generic tower keeps its stock structure symbol.
inline const char* PresentationLightTowerAlias(bool verified, bool simplified,
                                               int lookupRcid,
                                               const S57Obj* object,
                                               const char* symbol) {
  const bool conspicuous = lookupRcid == 31192;
  if (!verified || !simplified || !object || !symbol ||
      (!conspicuous && lookupRcid != 31199) ||
      object->Primitive_type != GEO_POINT ||
      std::memcmp(object->FeatureName, "LNDMRK", 6) ||
      std::memcmp(symbol, conspicuous ? "TOWERS03" : "TOWERS01", 8) ||
      object->n_attr < 2 || object->n_attr > 4096 || !object->att_array ||
      !object->attVal || object->attVal->GetCount() != unsigned(object->n_attr))
    return nullptr;
  unsigned seen = 0;
  for (int i = 0; i < object->n_attr; ++i) {
    const char* name = object->att_array + i * 6;
    for (const char* excluded : {"ORIENT", "STATUS", "QUAPOS", "QUASOU"})
      if (!std::memcmp(name, excluded, 6)) return nullptr;
    unsigned bit = 0;
    if (!std::memcmp(name, "CATLMK", 6)) bit = 1;
    else if (!std::memcmp(name, "FUNCTN", 6)) bit = 2;
    else if (!std::memcmp(name, "CONVIS", 6)) bit = 4;
    if (!bit) continue;
    if (seen & bit) return nullptr;
    seen |= bit;
    const auto* value = object->attVal->Item(i);
    if (!value || !value->value) return nullptr;
    if (bit == 4) {
      if (value->valType != OGR_INT ||
          *static_cast<const int*>(value->value) != (conspicuous ? 1 : 2))
        return nullptr;
    } else {
      // Both are S-57 lists encoded by the pinned core/private SENC readers as
      // owned NUL-terminated strings. Multiple categories/functions stay stock.
      if (value->valType != OGR_STR ||
          std::strncmp(static_cast<const char*>(value->value),
                       bit == 1 ? "17" : "33", 3)) return nullptr;
    }
  }
  if ((seen & 3) != 3 || (conspicuous && !(seen & 4))) return nullptr;
  return "XNLIT013";
}
}  // namespace opennav::integration

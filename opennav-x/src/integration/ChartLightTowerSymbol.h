#pragma once
#include <cstring>
#include "s52s57.h"

namespace opennav::integration {
// The pinned simplified LNDMRK light-support tower selectors (CATLMK17,
// FUNCTN33) retain their distinct effective tower raster, dimensions and pivot.
// Only owned ink changes. Never infer a lighthouse from a generic tower or
// replace a physical structure with the prototype's generic LIGHTS glyph.
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
  return conspicuous ? "XNLTWR03" : "XNLTWR01";
}
}  // namespace opennav::integration

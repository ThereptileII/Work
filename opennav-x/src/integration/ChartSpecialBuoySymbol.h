#pragma once
#include <cstring>
#include "s52s57.h"

namespace opennav::integration {
// `verified` is the existing library-owned presentation-symbol enable flag
// (historically named m_presentationLightSymbols), set only after resource
// verification. This helper does not infer the absence of a related TOPMAR.
// The pinned core and private SENC readers store enumerations as OGR_INT and
// list attributes as owned, NUL-terminated OGR_STR, not unbounded int arrays.
// Reject alternate/malformed representations rather than guessing list length.
inline const char* PresentationSpecialBuoyAlias(bool verified,
                                                bool simplified,
                                                const S57Obj* object,
                                                const char* symbol) {
  if (!verified || !simplified || !object || !symbol ||
      std::memcmp(object->FeatureName, "BOYSPP", 6) ||
      std::memcmp(symbol, "BOYSPP11", 8) ||
      object->Primitive_type != GEO_POINT || object->n_attr < 4 ||
      object->n_attr > 4096 || !object->att_array || !object->attVal ||
      object->attVal->GetCount() != static_cast<unsigned>(object->n_attr))
    return nullptr;

  unsigned matched = 0;
  for (int i = 0; i < object->n_attr; ++i) {
    const char* name = object->att_array + i * 6;
    // Never discard a supplied orientation or claim a physical topmark.
    if (!std::memcmp(name, "ORIENT", 6) || !std::memcmp(name, "TOPSHP", 6))
      return nullptr;
    unsigned bit = 0;
    const char* expected = nullptr;
    if (!std::memcmp(name, "BOYSHP", 6)) bit = 1;
    else if (!std::memcmp(name, "COLOUR", 6)) { bit = 2; expected = "1,11"; }
    else if (!std::memcmp(name, "COLPAT", 6)) { bit = 4; expected = "1"; }
    else if (!std::memcmp(name, "CATSPM", 6)) { bit = 8; expected = "27"; }
    if (!bit) continue;
    if (matched & bit) return nullptr;
    const S57attVal* value = object->attVal->Item(i);
    if (!value || !value->value) return nullptr;
    if (bit == 1) {
      if (value->valType != OGR_INT || *static_cast<const int*>(value->value) != 4)
        return nullptr;
    } else if (value->valType != OGR_STR ||
               std::strncmp(static_cast<const char*>(value->value), expected,
                            std::strlen(expected) + 1)) {
      return nullptr;
    }
    matched |= bit;
  }
  return matched == 15 ? "XNSPPW01" : nullptr;
}
}  // namespace opennav::integration

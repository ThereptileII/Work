#pragma once
#include <cmath>
#include <cstring>
#include "s52s57.h"

namespace opennav::integration {
// Pinned SENC readers own these NUL-terminated strings. Accept only canonical
// small enumeration lists; never interpret an unbounded OGR_INT_LST buffer.
inline bool YellowBuoyCategories(const char* text) {
  if (!text || !*text) return false;
  unsigned value = 0, digits = 0, count = 0;
  for (unsigned i = 0; i < 128; ++i) {
    const char c = text[i];
    if (c >= '0' && c <= '9') {
      if ((!digits && c == '0') || ++digits > 2) return false;
      value = value * 10 + unsigned(c - '0');
    } else if (c == ',' || c == 0) {
      // These two earlier Simplified lookups select BOYSUP02/BOYDEF03.
      if (!digits || value == 15 || value == 52 || ++count > 32) return false;
      if (!c) return true;
      value = digits = 0;
    } else return false;
  }
  return false;
}

inline bool YellowBuoyAttributes(const S57Obj* object, bool topmark) {
  if (!object || object->Primitive_type != GEO_POINT || object->n_attr < 2 ||
      object->n_attr > 4096 || !object->att_array || !object->attVal ||
      object->attVal->GetCount() != unsigned(object->n_attr) ||
      std::memcmp(object->FeatureName, topmark ? "TOPMAR" : "BOYSPP", 6)) return false;
  unsigned seen = 0;
  for (int i = 0; i < object->n_attr; ++i) {
    const char* name = object->att_array + i * 6;
    if (!std::memcmp(name, "ORIENT", 6) || !std::memcmp(name, "COLPAT", 6) ||
        !std::memcmp(name, topmark ? "BOYSHP" : "TOPSHP", 6)) return false;
    unsigned bit = 0;
    if (!std::memcmp(name, topmark ? "TOPSHP" : "BOYSHP", 6)) bit = 1;
    else if (!std::memcmp(name, "COLOUR", 6)) bit = 2;
    else if (!std::memcmp(name, "CATSPM", 6)) bit = 4;
    if (!bit) continue;
    if (seen & bit) return false;
    const S57attVal* value = object->attVal->Item(i);
    if (!value || !value->value) return false;
    if (bit == 1) {
      if (value->valType != OGR_INT) return false;
      const int shape = *static_cast<const int*>(value->value);
      // These known shapes share the original Simplified BOYSPP11 portrayal.
      if (topmark ? shape != 7 : !(shape == 3 || shape == 4 || shape == 5 || shape == 6 || shape == 8)) return false;
    } else {
      if (value->valType != OGR_STR) return false;
      const char* text = static_cast<const char*>(value->value);
      if (bit == 2 ? std::strncmp(text, "6", 2) != 0 : !YellowBuoyCategories(text)) return false;
    }
    seen |= bit;
  }
  return (seen & 3) == 3;
}

inline const char* PresentationYellowBuoyAlias(bool enabled, bool simplified,
                                               const S57Obj* object,
                                               const char* symbol) {
  return enabled && simplified && symbol && !std::memcmp(symbol, "BOYSPP11", 8) &&
         YellowBuoyAttributes(object, false) ? "XNSPPY01" : nullptr;
}

// Both pinned XML parsers append one unit separator to an empty instruction.
// Accept only that canonical no-op or an unparsed empty string, never a prefix.
// Core owns INST by value; pinned API-17 owns a wxString pointer.
inline bool YellowEmptyInstruction(const wxString& value) {
  return value.empty() || (value.length() == 1 && value[0] == '\037');
}
inline bool YellowEmptyInstruction(const wxString* value) {
  return value && YellowEmptyInstruction(*value);
}

inline bool PresentationYellowTopmark(bool enabled, const ObjRazRules* rz) {
  if (!enabled || !rz || !rz->LUP || !rz->obj ||
      rz->LUP->TNAM != SIMPLIFIED || rz->LUP->RCID != 31314 ||
      std::memcmp(rz->LUP->OBCL, "TOPMAR", 6) ||
      !rz->LUP->ATTArray.empty() || !YellowEmptyInstruction(rz->LUP->INST) || rz->LUP->ruleList ||
      !YellowBuoyAttributes(rz->obj, true) || !rz->obj->m_chart_context ||
      !std::isfinite(rz->obj->x) || !std::isfinite(rz->obj->y)) return false;
  const auto* platforms = rz->obj->m_chart_context->pFloatingATONArray;
  if (!platforms || platforms->GetCount() > 4096) return false;
  const S57Obj* match = nullptr;
  for (unsigned i = 0; i < platforms->GetCount(); ++i) {
    const auto* object = static_cast<const S57Obj*>(platforms->Item(i));
    if (!object) return false;
    // Exact x/y equality is the pinned TOPMAR01/_atPtPos relation. Never
    // approximate geographic proximity or retain borrowed object pointers.
    if (object->x != rz->obj->x || object->y != rz->obj->y) continue;
    if (match || object->m_chart_context != rz->obj->m_chart_context ||
        !YellowBuoyAttributes(object, false)) return false;
    match = object;
  }
  return match != nullptr;
}
}  // namespace opennav::integration

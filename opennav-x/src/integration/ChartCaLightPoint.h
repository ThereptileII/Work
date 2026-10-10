#pragma once
#include <cmath>
#include <cstring>
#include <memory>
#include <new>
#include <unordered_map>
#include <unordered_set>
#include "s52s57.h"
#include "integration/ChartLightTowerSymbol.h"

namespace opennav::integration {
// This is a presentation inventory for one synchronous point pass, not a chart
// query/cache. Exact x/y + chart identity follow the pinned TOPMAR co-location
// relation. All selected point heads are inspected before visibility filtering.
class CaLightPointInventory {
 public:
  static constexpr unsigned kMaxObjects = 32768;
  static constexpr unsigned kMaxAttributes = 262144;

  template <size_t N, size_t M>
  CaLightPointInventory(bool enabled, ObjRazRules* const (&heads)[N][M],
                        unsigned column) {
    if (!enabled || column != 0 || column >= M) return;
    try {
    std::unordered_map<Key, Group, KeyHash> groups;
    std::unordered_set<const S57Obj*> objects;
    unsigned attributes = 0, count = 0;
    for (size_t i = 0; i < N; ++i) {
      for (const ObjRazRules* node = heads[i][column]; node; node = node->next) {
        if (++count > kMaxObjects || !node->obj ||
            !objects.insert(node->obj).second) return;
        const auto* object = node->obj;
        if (MultipointSounding(object)) continue;
        if (object->Primitive_type != GEO_POINT) {
          if (!std::memcmp(object->FeatureName, "LIGHTS", 6)) return;
          continue;
        }
        if (!object->m_chart_context ||
            !std::isfinite(object->x) || !std::isfinite(object->y)) return;
        const bool light = !std::memcmp(object->FeatureName, "LIGHTS", 6);
        const bool fog = !std::memcmp(object->FeatureName, "FOGSIG", 6);
        if (light || fog) {
          if (object->n_attr < 0 || object->n_attr > 4096 ||
              unsigned(object->n_attr) > kMaxAttributes - attributes) return;
          attributes += unsigned(object->n_attr);
        }
        if (!light) continue;
        auto& group = groups[{object->m_chart_context, object->x, object->y}];
        const char* symbol = nullptr;
        unsigned color = 0;
        symbol = Classify(node, color);
        // A separate structure, topmark, short-range point or uncertain light
        // suppresses this additional point; its original painting is untouched.
        if (!symbol) group.refused = true;
        if (group.symbol && group.color != color) group.mixed = true;
        group.symbol = symbol;
        group.color = color;
        group.owner = object;  // Last actual traversal member, not first wins.
      }
    }
    // A second bounded linear walk marks independent co-located points. Only
    // LIGHTS keys allocate groups; charts without lights create no group map.
    // No callbacks or conditional/visibility processing can mutate these heads
    // between walks. The first walk proved a finite, nonduplicated chain.
    if (groups.empty()) return;
    for (size_t i = 0; i < N; ++i) {
      for (const ObjRazRules* node = heads[i][column]; node; node = node->next) {
        const auto* object = node->obj;
        if (object->Primitive_type != GEO_POINT ||
            MultipointSounding(object) ||
            !std::memcmp(object->FeatureName, "LIGHTS", 6)) continue;
        auto found = groups.find({object->m_chart_context, object->x, object->y});
        // The fyr's own tower is the light's structure, not an independent
        // platform: its lighthouse mark belongs at the centre of the sectors.
        if (found != groups.end() && !CompatibleFog(node) &&
            !LightSupportTower(object))
          found->second.refused = true;
      }
    }
    for (const auto& entry : groups) {
      const auto& group = entry.second;
      if (!group.refused && group.symbol)
        points_.emplace(group.owner, group.mixed ? "XNLIT013" : group.symbol);
    }
    } catch (const std::bad_alloc&) {
      points_.clear();  // Decorative allocation failure preserves stock drawing.
    }
  }

  const char* Take(const S57Obj* object) {
    auto it = points_.find(object);
    if (it == points_.end()) return nullptr;
    const char* symbol = it->second;
    points_.erase(it);  // At most once per object in this pass, even with two CA rules.
    return symbol;
  }
  size_t size() const { return points_.size(); }

 private:
  static bool MultipointSounding(const S57Obj* object) {
    // Pinned SetMultipointGeometry uses GEO_POINT for SOUNDG parents, stores
    // their geometry in arrays, and never initializes scalar x/y. RenderMPS
    // owns those soundings; they are not an independent co-located platform.
    // Check the initialized array pointers before npt (unset for other types).
    return !std::memcmp(object->FeatureName, "SOUNDG", 6) &&
           object->Primitive_type == GEO_POINT && !object->bIsClone &&
           object->m_chart_context && object->geoPtz && object->geoPtMulti &&
           object->npt > 0;
  }

  struct Key {
    const void* chart;
    double x, y;
    bool operator==(const Key& other) const {
      return chart == other.chart && x == other.x && y == other.y;
    }
  };
  struct KeyHash {
    size_t operator()(const Key& key) const {
      return std::hash<const void*>{}(key.chart) ^
             (std::hash<double>{}(key.x) << 1) ^
             (std::hash<double>{}(key.y) << 2);
    }
  };
  struct Group {
    const S57Obj* owner = nullptr;
    const char* symbol = nullptr;
    unsigned color = 0;
    bool refused = false;
    bool mixed = false;
  };

  static bool FogInstruction(const wxString& value) {
    return value == "SY(FOGSIG01)" || value == "SY(FOGSIG01)\037";
  }
  static bool FogInstruction(const wxString* value) {
    return value && FogInstruction(*value);
  }
  static bool CompatibleFog(const ObjRazRules* node) {
    const auto* object = node->obj;
    const auto* lookup = node->LUP;
    if (std::memcmp(object->FeatureName, "FOGSIG", 6) || !lookup ||
        lookup->RCID != 31164 || lookup->FTYP != POINT_T ||
        lookup->DPRI != PRIO_SYMB_AREA || lookup->RPRI != RAD_OVER ||
        lookup->TNAM != SIMPLIFIED || lookup->DISC != STANDARD ||
        lookup->LUCM != 27080 || std::memcmp(lookup->OBCL, "FOGSIG", 6) ||
        !lookup->ATTArray.empty() || !FogInstruction(lookup->INST) ||
        object->n_attr < 0 || object->n_attr > 4096 ||
        (object->n_attr && (!object->att_array || !object->attVal)) ||
        (object->attVal && object->attVal->GetCount() != unsigned(object->n_attr)))
      return false;
    for (int i = 0; i < object->n_attr; ++i) {
      const char* name = object->att_array + i * 6;
      for (const char* excluded : {"ORIENT", "STATUS", "QUAPOS", "QUASOU"})
        if (!std::memcmp(name, excluded, 6)) return false;
    }
    // The verified resource fixes this lookup and bitmap. Do not force lazy
    // parsing or make the first paint depend on whether the rule is loaded yet.
    // An already loaded rule must agree exactly; altered chains remain stock.
    const auto* rule = lookup->ruleList;
    if (!rule) return true;
    if (rule->ruleType != RUL_SYM_PT || rule->next || !rule->INSTstr ||
        (std::strcmp(rule->INSTstr, "FOGSIG01)\037") &&
         std::strcmp(rule->INSTstr, "FOGSIG01)")) ||
        !rule->razRule || rule->b_private_razRule) return false;
    const auto* symbol = rule->razRule;
    const auto& position = symbol->pos.symb;
    return symbol->RCID == 1338 && !std::memcmp(symbol->name.SYNM, "FOGSIG01", 8) &&
           symbol->definition.SYDF == 'R' &&
           position.bnbox_w.SYHL == 12 && position.bnbox_h.SYVL == 13 &&
           position.pivot_x.SYCL == 15 && position.pivot_y.SYRW == -3 &&
           position.bnbox_x.SBXC == 0 && position.bnbox_y.SBXR == 0;
  }

  static bool Instruction(const wxString& value) {
    return value == "CS(LIGHTS05)" || value == "CS(LIGHTS05)\037";
  }
  static bool Instruction(const wxString* value) {
    return value && Instruction(*value);
  }
  static const char* Classify(const ObjRazRules* node, unsigned& color) {
    const auto* object = node->obj;
    const auto* lookup = node->LUP;
    if (!lookup || lookup->RCID != 31183 || lookup->TNAM != SIMPLIFIED ||
        std::memcmp(lookup->OBCL, "LIGHTS", 6) || !lookup->ATTArray.empty() ||
        !Instruction(lookup->INST) || object->n_attr < 1 ||
        !object->att_array || !object->attVal ||
        object->attVal->GetCount() != unsigned(object->n_attr)) return nullptr;
    unsigned seen = 0;
    double range = 9.;
    const char* symbol = nullptr;
    for (int i = 0; i < object->n_attr; ++i) {
      const char* name = object->att_array + i * 6;
      // Presence is enough to refuse: never sanitize malformed orientation or
      // reinterpret uncertain, special, obscured/faint or directional records.
      for (const char* excluded : {"ORIENT", "CATLIT", "LITVIS", "STATUS",
                                   "QUAPOS", "QUASOU"}) {
        if (std::memcmp(name, excluded, 6)) continue;
        return nullptr;
      }
      unsigned bit = 0;
      if (!std::memcmp(name, "COLOUR", 6)) bit = 1;
      else if (!std::memcmp(name, "VALNMR", 6)) bit = 2;
      else if (!std::memcmp(name, "SECTR1", 6)) bit = 4;
      else if (!std::memcmp(name, "SECTR2", 6)) bit = 8;
      if (!bit) continue;
      if (seen & bit) return nullptr;
      seen |= bit;
      const auto* value = object->attVal->Item(i);
      if (!value || !value->value) return nullptr;
      if (bit == 1) {
        if (value->valType != OGR_STR) return nullptr;
        const char* text = static_cast<const char*>(value->value);
        if (!std::strncmp(text, "3", 2)) symbol = "XNLIT011";
        else if (!std::strncmp(text, "4", 2)) symbol = "XNLIT012";
        else if (!std::strncmp(text, "1", 2) || !std::strncmp(text, "6", 2)) symbol = "XNLIT013";
        else return nullptr;
        color = unsigned(text[0] - '0');
      } else {
        if (value->valType != OGR_REAL) return nullptr;
        const double number = *static_cast<const double*>(value->value);
        if (!std::isfinite(number)) return nullptr;
        if (bit == 2) { if (number <= 0.) return nullptr; range = number; }
        else if (number < 0. || number > 360.) return nullptr;
      }
    }
    if (!symbol || ((seen & 12) != 0 && (seen & 12) != 12)) return nullptr;
    // Actual LIGHTS06 uses CA for complete sectors (including all-round sector
    // pairs), or for non-sector nominal range >=10. Do not evaluate CS here.
    if (!(seen & 12) && range < 10.) return nullptr;
    return symbol;
  }
  std::unordered_map<const S57Obj*, const char*> points_;
};

// Restores prior inventory on normal, early or exceptional scope exit. The
// library borrows this pointer only while the chart's existing pass is running.
template <class Library>
class CaLightPointScope {
 public:
  template <size_t N, size_t M>
  CaLightPointScope(Library* library, ObjRazRules* const (&heads)[N][M],
                    unsigned column)
      : library_(library),
        inventory_(library->PresentationCaLightsEnabled() && column == 0 && column < M
                       ? new (std::nothrow) CaLightPointInventory(true, heads, column)
                       : nullptr),
        previous_(library_->SetPresentationCaLights(inventory_.get())) {}
  // Restore the borrowed pointer before destroying this owned inventory. An
  // inactive or allocation-failed nested pass must shadow the outer inventory
  // with nullptr, never consume its still-live points.
  ~CaLightPointScope() { library_->SetPresentationCaLights(previous_); }
  CaLightPointScope(const CaLightPointScope&) = delete;
  CaLightPointScope& operator=(const CaLightPointScope&) = delete;
 private:
  Library* library_;
  std::unique_ptr<CaLightPointInventory> inventory_;
  CaLightPointInventory* previous_;
};
}  // namespace opennav::integration

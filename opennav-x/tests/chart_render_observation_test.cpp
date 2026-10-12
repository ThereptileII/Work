// Real extracted core/private SY and CA dispatch. Projection, alias predicates
// and terminal painters are explicit seams; no canvas/GPU acceptance is claimed.
#include "integration/ChartRenderObservation.h"
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>
using namespace opennav::integration;
constexpr int GEO_POINT = 1, SIMPLIFIED = 76;
struct Object {
  char FeatureName[8] = "LNDMRK";
  int Index = 7, Primitive_type = GEO_POINT, n_attr = 0;
  const char* att_array = nullptr;
  double x = 1, y = 2;
};
struct Rule {
  struct { char SYNM[9] = "TOWERS01"; } name;
  struct { char SYDF = 'R'; } definition;
};
struct Lookup { int TNAM = SIMPLIFIED, RCID = 31199; };
struct ObjRazRules { Object* obj; Lookup* LUP; };
struct Rules { Rule* razRule; char INSTstr[16] = "TOWERS01)"; };
struct wxPoint { int x = 0, y = 0; };
struct wxString { static std::string FromUTF8(const char* value) { return value; } };
static const char* tower_alias = "XNLIT013";
namespace opennav::integration {
template<class... T> const char* PresentationLightAlias(T...) { return nullptr; }
template<class... T> const char* PresentationSpecialBuoyAlias(T...) { return nullptr; }
template<class... T> const char* PresentationYellowBuoyAlias(T...) { return nullptr; }
template<class... T> const char* PresentationLightTowerAlias(T...) { return tower_alias; }
}
struct CaInventory {
  const char* name = "XNLIT013";
  const char* Take(Object*) { auto* result = name; name = nullptr; return result; }
};
class s52plib {
 public:
  bool m_presentationLightSymbols = true, m_presentationRasterSubmitted = false;
  ChartRenderObservation* m_presentationObservation = nullptr;
  CaInventory* m_presentationCaLights = nullptr;
  std::string m_ColorScheme = "DAY";
  double m_ChartScaleFactorExp = 1;
  long state = 11;
  bool submit = true;
  int calls = 0;
  Rule* painted = nullptr;
  std::map<std::string, Rule*> dictionary;
  decltype(dictionary)* _symb_sym = &dictionary;
  long GetStateHash() { return state; }
  ChartRenderObservation* SetPresentationObservation(ChartRenderObservation* value) {
    auto* previous = m_presentationObservation; m_presentationObservation = value; return previous;
  }
  bool GetDoubleAttr(Object*, const char*, double&) { return false; }
  void GetPointPixSingle(ObjRazRules*, double, double, wxPoint*) {}
  void RenderHPGL(ObjRazRules*, Rule* rule, wxPoint&, float, double) { painted = rule; ++calls; }
  bool RenderRasterSymbol(ObjRazRules*, Rule* rule, wxPoint&, float) {
    painted = rule; ++calls; m_presentationRasterSubmitted = submit;
    return true;  // Deliberately also true when nothing was submitted.
  }
  int RenderSY(ObjRazRules*, Rules*);
  void RenderPresentationCaLightPoint(ObjRazRules*);
};
#include "actual-render-observation.inc"
struct VP {
  double clat = 58, clon = 17, view_scale_ppm = 1, skew = 0, rotation = 0;
  double chart_scale = 20000;
  int pix_width = 1280, pix_height = 800, m_projection_type = 1;
  bool b_quilt = true;
  struct { int x = 0, y = 0, width = 1280, height = 800; } rv_rect;
};
static unsigned checks = 0;
static void Check(bool condition) {
  ++checks;
  if (!condition) throw std::runtime_error("render observation check " + std::to_string(checks));
}
static bool Available(const std::string& value) { return value.find("raster paint submitted:") != std::string::npos; }
int main() try {
  ChartRenderObservation observations;
  s52plib library;
  Object tower, clone;
  Lookup lup;
  ObjRazRules node{&tower, &lup};
  Rule stock, alias;
  std::memcpy(alias.name.SYNM, "XNLIT013", 9);
  library.dictionary["XNLIT013"] = &alias;
  Rules rules{&stock};
  auto view = ChartRenderObservation::View::Capture(VP{}, library.state);
  auto query = [&](Object* selected = nullptr) {
    observations.BeginQuery(view, true);
    observations.Select(selected ? selected : &tower, &tower);
    return observations.Describe(selected ? selected : &tower);
  };
  Check(!Available(query()));
  {
    ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
    stock.definition.SYDF = 'R'; library.RenderSY(&node, &rules);
    stock.definition.SYDF = 'V'; library.RenderSY(&node, &rules);
    scope.Complete();
  }
  Check(!Available(query())); // an observed raster cannot hide an unobserved vector
  {
    ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
    stock.definition.SYDF = 'R'; node.LUP = nullptr;
    library.RenderSY(&node, &rules); node.LUP = &lup;
    library.RenderSY(&node, &rules); scope.Complete();
  }
  Check(!Available(query())); // partial lookup metadata stays unavailable
  {
    ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
    library.RenderSY(&node, &rules);
#ifdef OBSERVATION_ENABLED
    Check(library.painted == &alias);
#else
    Check(library.painted == &stock);
#endif
    Check(!Available(query())); // unfinished render cannot supply Info
    scope.Complete();
  }
  Check(library.m_presentationObservation == nullptr);
#ifdef OBSERVATION_ENABLED
  auto text = query(&clone); // actual private clone mapping, not Index guessing
  Check(Available(text));
  Check(text.find("TOWERS01 / raster paint submitted: XNLIT013") != std::string::npos);
  Check(text.find("LUP RCID: 31199 / point table: 76") != std::string::npos);
  Check(!Available(observations.Describe(&tower))); // unselected source is not the clone
  clone.Index = 99;
  Check(!Available(observations.Describe(&clone)));
  clone.Index = 7;
  alias.name.SYNM[0] = 'Y'; // stored symbol names are copied
  Check(Available(observations.Describe(&clone)));
  Check(observations.Describe(&clone).find("XNLIT013") != std::string::npos);
  alias.name.SYNM[0] = 'X';
  ChartRenderObservation other_chart;
  other_chart.BeginQuery(view, true); other_chart.Select(&clone, &tower);
  Check(!Available(other_chart.Describe(&clone)));
  for (unsigned i = 0; i < view.numbers.size(); ++i) {
    auto different = view; different.numbers[i] += 1;
    observations.BeginQuery(different, true); observations.Select(&clone, &tower);
    Check(!Available(observations.Describe(&clone)));
  }
  for (unsigned i = 0; i < view.integers.size(); ++i) {
    auto different = view; ++different.integers[i];
    observations.BeginQuery(different, true); observations.Select(&clone, &tower);
    Check(!Available(observations.Describe(&clone)));
  }
  auto different = view; ++different.state;
  observations.BeginQuery(different, true); observations.Select(&clone, &tower);
  Check(!Available(observations.Describe(&clone)));
  observations.BeginQuery(view, false); observations.Select(&clone, &tower);
  Check(!Available(observations.Describe(&clone)));
  for (bool submit : {false, true}) {
    library.submit = submit;
    {
      ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
      library.RenderSY(&node, &rules); scope.Complete();
    }
    Check(Available(query()) == submit); // terminal true/no-draw is refused
    Object light; std::memcpy(light.FeatureName, "LIGHTS", 7);
    ObjRazRules light_node{&light, &lup}; CaInventory inventory;
    library.m_presentationCaLights = &inventory;
    {
      ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
      library.RenderPresentationCaLightPoint(&light_node); scope.Complete();
    }
    observations.BeginQuery(view, true); observations.Select(&light, &light);
    text = observations.Describe(&light);
    Check(Available(text) == submit);
    if (submit) Check(text.find("CA light point: XNLIT013") != std::string::npos);
  }
  // The same selected LIGHTS already has a valid actual SY observation. A
  // subsequent CA raster submission must not leave that partial record usable
  // when its lookup metadata is missing.
  for (bool missing_lookup : {false, true}) {
    Object light; std::memcpy(light.FeatureName, "LIGHTS", 7);
    ObjRazRules light_node{&light, &lup}; CaInventory inventory;
    library.m_presentationCaLights = &inventory;
    {
      ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
      library.RenderSY(&light_node, &rules);
      if (missing_lookup) light_node.LUP = nullptr;
      library.RenderPresentationCaLightPoint(&light_node);
      light_node.LUP = &lup;
      scope.Complete();
    }
    observations.BeginQuery(view, true); observations.Select(&light, &light);
    text = observations.Describe(&light);
    Check(Available(text) == !missing_lookup);
    if (!missing_lookup) {
      Check(text.find("Resolved SY:") != std::string::npos);
      Check(text.find("CA light point:") != std::string::npos);
    }
  }
  // Missing raster, vector-only and aborted scopes remain unavailable.
  library.dictionary.clear();
  {
    ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
    library.RenderSY(&node, &rules); // missing alias paints actual stock raster
    scope.Complete();
  }
  Check(query().find("TOWERS01 / raster paint submitted: TOWERS01") != std::string::npos);
  stock.definition.SYDF = 'V';
  {
    ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
    library.RenderSY(&node, &rules); scope.Complete();
  }
  Check(!Available(query()));
  {
    ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
    observations.Record(&tower, 1, 76, "TOWERS01", "XNLIT013", false);
    ++library.state; scope.Complete();
  }
  Check(!Available(query()));
  --library.state;
  {
    ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
    observations.Record(&tower, 1, 76, "TOWERS01", "XNLIT013", false);
  }
  Check(!Available(query()));
  {
    ChartRenderObservationScope<s52plib> scope(observations, &library, view, false);
    observations.Record(&tower, 1, 76, "TOWERS01", "XNLIT013", false); scope.Complete();
  }
  Check(!Available(query()));
  {
    ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
    for (unsigned i = 0; i < 1000; ++i)
      observations.Record(&tower, 1, 76, "TOWERS01", "XNLIT013", false);
    scope.Complete();
  }
  Check(Available(query())); // repeated clipped passes deduplicate, not overflow
  observations.BeginQuery(view, true);
  for (unsigned i = 0; i <= ChartRenderObservation::max_objects; ++i)
    observations.Select(&clone, &tower);
  Check(!Available(observations.Describe(&clone)));
  {
    ChartRenderObservationScope<s52plib> outer(observations, &library, view, true);
    { ChartRenderObservationScope<s52plib> inner(observations, &library, view, true); inner.Complete(); }
    outer.Complete();
  }
  Check(!Available(query()));
  {
    ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
    observations.Record(&tower, 1, 76, "BAD<CODE", "XNLIT013", false); scope.Complete();
  }
  Check(!Available(query()));
  std::vector<Object> many(ChartRenderObservation::max_objects + 1);
  {
    ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
    for (auto& object : many) observations.Record(&object, 1, 76, "TOWERS01", "XNLIT013", false);
    scope.Complete();
  }
  observations.BeginQuery(view, true); observations.Select(&many[0], &many[0]);
  Check(!Available(observations.Describe(&many[0])));
  {
    ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
    for (int rcid = 1; rcid <= 5; ++rcid) observations.Record(&tower, rcid, 76, "TOWERS01", "XNLIT013", false);
    scope.Complete();
  }
  Check(!Available(query()));
  view.numbers[0] = std::numeric_limits<double>::quiet_NaN();
  {
    ChartRenderObservationScope<s52plib> scope(observations, &library, view, true);
    observations.Record(&tower, 1, 76, "TOWERS01", "XNLIT013", false); scope.Complete();
  }
  Check(!Available(query()));
#else
  Check(!Available(query())); // Standard dispatch does not publish observations
#endif
  std::cout << checks << " actual dispatch/copy/generation/bounds checks passed\n";
} catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }

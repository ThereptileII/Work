// SCRUM-283: actual locked query, polygon tests, coordinate conversion and bridge.
// Only chart state, already-tessellated synthetic inputs and mariner settings are
// fixtures. This is not chart ingestion, tessellation generation or rendering.
#include <wx/wx.h>
#include <wx/list.h>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <list>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include "s52s57.h"
#include "s52utils.h"
#include "poly_math.h"
#include "georef.h"
#include "geometry-types.inc"

static int checks = 0, queries = 0, destroyedObjects = 0;
static void Check(bool ok, const std::string& name) {
  ++checks;
  if (!ok) throw std::runtime_error("area-query check failed: " + name);
}

WX_DECLARE_LIST(S57Obj, ListOfS57Obj);
#include <wx/listimpl.cpp>
WX_DEFINE_LIST(ListOfS57Obj);

// Deliberately a fixture class, not the complete chart class or its ABI. The
// helper verifies these declarations against the locked eSENCChart.h. Both
// method definitions below are extracted unchanged from eSENCChart.cpp.
class eSENCChart {
 public:
  double m_ref_lat = 0, m_ref_lon = 0;
  ObjRazRules* razRules[PRIO_NUM][LUPNAME_NUM]{};
  ListOfS57Obj* GetAssociatedObjects(S57Obj* obj);
  bool IsPointInObjArea(float lat, float lon, float select_radius, S57Obj* obj);
};

S57Obj::S57Obj() { Init(); }
// Chart objects/attributes below are stack-owned fixture data. Count destruction
// to check that deleting either returned list does not delete borrowed objects.
S57Obj::~S57Obj() { ++destroyedObjects; }
double S52_getMarinerParam(S52_MAR_param_t) { return 5.; }
#define UNKNOWN 1e6
#include "actual-bodies.inc"
#ifdef PATCHED_PRIVATE
#include "bridge.inc"
#endif

struct Object : S57Obj {
  wxArrayOfS57attVal values;
  double depth = 10;
  S57attVal depthValue{&depth, OGR_REAL};
  char depthName[7] = "DRVAL1";
  explicit Object(GeoPrim_t primitive = GEO_POINT) {
    Primitive_type = primitive;
    x = y = 0;
    m_DisplayCat = OTHER;
    m_chart_context = nullptr;
    std::memcpy(FeatureName, primitive == GEO_AREA ? "DEPARE" : "UWTROC", 7);
    if (primitive == GEO_AREA) {
      values.Add(&depthValue);
      attVal = &values;
      att_array = depthName;
      n_attr = 1;
      bIsAssociable = true;
    }
  }
};

// Every vertex is supplied in metres in this chart's existing SM coordinate
// system. The production coordinate functions produce geographical bounds.
static LLBBox Bounds(const eSENCChart& chart, double x1, double y1,
                    double x2, double y2) {
  double lat1, lon1, lat2, lon2;
  fromSM_Plugin(x1, y1, chart.m_ref_lat, chart.m_ref_lon, &lat1, &lon1);
  fromSM_Plugin(x2, y2, chart.m_ref_lat, chart.m_ref_lon, &lat2, &lon2);
  LLBBox box;
  box.Set(lat1, lon1, lat2, lon2);
  return box;
}

struct Area : Object {
  PolyTessGeo tess;
  ObjRazRules rule{};
  Area(eSENCChart& chart, int storage = DATA_TYPE_DOUBLE,
       unsigned primitive = PTG_TRIANGLES,
       std::vector<double> vertices = {0, 0, 1000, 0, 0, 1000})
      : Object(GEO_AREA) {
    auto* group = new PolyTriGroup;
    group->data_type = storage;
    auto* triangle = new TriPrim;
    triangle->type = primitive;
    triangle->nVert = static_cast<int>(vertices.size() / 2);
    triangle->p_next = nullptr;
    triangle->tri_box = Bounds(chart, 0, 0, 1000, 1000);
    if (storage == DATA_TYPE_DOUBLE) {
      triangle->p_vertex = static_cast<double*>(std::malloc(vertices.size() * sizeof(double)));
      Check(triangle->p_vertex != nullptr, "allocate double vertices");
      std::memcpy(triangle->p_vertex, vertices.data(), vertices.size() * sizeof(double));
    } else {
      auto* data = static_cast<float*>(std::malloc(vertices.size() * sizeof(float)));
      Check(data != nullptr, "allocate float vertices");
      for (size_t i = 0; i < vertices.size(); ++i) data[i] = static_cast<float>(vertices[i]);
      triangle->p_vertex = reinterpret_cast<double*>(data);
    }
    group->tri_prim_head = triangle;
    tess.Set_PolyTriGroup_head(group);
    tess.Set_OK(true);
    pPolyTessGeo = &tess;
    BBObj = triangle->tri_box;
    rule.obj = this;
  }
};

static void Query(eSENCChart& chart, Object& object, S57Obj* expected,
                  const std::string& name) {
  const int before = destroyedObjects;
  {
    std::unique_ptr<ListOfS57Obj> found(chart.GetAssociatedObjects(&object));
    Check(found != nullptr, name + ": container");
    Check(found->GetCount() == (expected ? 1u : 0u), name + ": count");
    if (expected) Check(found->GetFirst()->GetData() == expected, name + ": identity");
  }
  Check(destroyedObjects == before, name + ": borrowed lifetime");
  ++queries;
  std::cout << "query " << name << " " << (expected ? "match" : "empty") << '\n';
}

static void CheckQuerySemantics() {
  eSENCChart chart;
  Area first(chart), second(chart), symbolized(chart);
  chart.razRules[1][3] = &first.rule;
  Object object;
  object.x = object.y = 200;
  Query(chart, object, &first, "point-inside-triangle");
  object.x = object.y = 800;
  Query(chart, object, nullptr, "point-outside-polygon-inside-bounds");
  object.x = 1200; object.y = 200;
  Query(chart, object, nullptr, "point-outside-area-bounds");
  object.x = object.y = 200;
  first.bIsAssociable = false;
  Query(chart, object, nullptr, "nonassociable-skipped");
  first.bIsAssociable = true;
  first.rule.next = &second.rule;
  chart.razRules[1][4] = &symbolized.rule;
  Query(chart, object, &first, "first-plain-match-before-symbolized");
  first.bIsAssociable = false;
  Query(chart, object, &second, "later-plain-match");
  second.bIsAssociable = false;
  Query(chart, object, &symbolized, "symbolized-fallback");
  chart.razRules[1][4] = nullptr;
  chart.razRules[1][3] = nullptr;
  chart.razRules[2][3] = &symbolized.rule;
  Query(chart, object, nullptr, "other-priority-group-ignored");
  chart.razRules[1][3] = &symbolized.rule;
  symbolized.pPolyTessGeo = nullptr;
  Query(chart, object, nullptr, "no-tessellation");
  symbolized.pPolyTessGeo = &symbolized.tess;
  symbolized.tess.Set_OK(false);
  Query(chart, object, nullptr, "deferred-tessellation-not-accepted");
  symbolized.tess.Set_OK(true);
  auto* triangle = symbolized.tess.Get_PolyTriGroup_head()->tri_prim_head;
  triangle->tri_box = Bounds(chart, 600, 600, 1000, 1000);
  Query(chart, object, nullptr, "triangle-bounds-filter");
  triangle->tri_box = symbolized.BBObj;
  for (GeoPrim_t primitive : {GEO_LINE, GEO_AREA}) {
    object.Primitive_type = primitive;
    object.BBObj = Bounds(chart, 0, 0, 1000, 1000);
    object.x = object.y = 200;
    Query(chart, object, &symbolized, primitive == GEO_LINE ? "line-reference-inside" : "area-reference-inside");
    // The overlapping object bounds do not replace the documented reference-
    // point test. This does not assert general line/polygon intersection.
    object.x = object.y = 800;
    Query(chart, object, nullptr, primitive == GEO_LINE ? "line-reference-outside" : "area-reference-outside");
  }
  object.Primitive_type = GEO_META;
  object.x = object.y = 200;
  Query(chart, object, nullptr, "unsupported-primitive");
}

static void CheckTriangleStorage() {
  for (int storage : {DATA_TYPE_DOUBLE, DATA_TYPE_FLOAT}) {
    for (unsigned primitive : {PTG_TRIANGLES, PTG_TRIANGLE_STRIP, PTG_TRIANGLE_FAN}) {
      eSENCChart chart;
      // Hit the second triangle, not merely the first triplet of each format.
      const std::vector<double> vertices = primitive == PTG_TRIANGLES
          ? std::vector<double>{0, 0, 1000, 0, 0, 1000, 1000, 0, 1000, 1000, 0, 1000}
          : primitive == PTG_TRIANGLE_STRIP
              ? std::vector<double>{0, 0, 1000, 0, 0, 1000, 1000, 1000}
              : std::vector<double>{0, 0, 1000, 0, 1000, 1000, 0, 1000};
      Area area(chart, storage, primitive, vertices);
      chart.razRules[1][3] = &area.rule;
      Object object;
      object.x = primitive == PTG_TRIANGLE_FAN ? 200 : 800;
      object.y = 800;
      const std::string name = std::to_string(storage) + "-" + std::to_string(primitive);
      Query(chart, object, &area, "second-triangle-" + name);
      object.x = -200;
      Query(chart, object, nullptr, "outside-triangles-" + name);
    }
  }
  eSENCChart chart;
  chart.m_ref_lat = 58.; chart.m_ref_lon = 16.;
  Area miss(chart), hit(chart);
  auto* group = miss.tess.Get_PolyTriGroup_head();
  auto* later = hit.tess.Get_PolyTriGroup_head()->tri_prim_head;
  hit.tess.Get_PolyTriGroup_head()->tri_prim_head = nullptr;
  group->tri_prim_head->tri_box = Bounds(chart, 600, 600, 1000, 1000);
  group->tri_prim_head->p_next = later;
  chart.razRules[1][3] = &miss.rule;
  Object object;
  object.x = 25; object.x_rate = 4; object.x_origin = 100;
  object.y = 150; object.y_rate = 2; object.y_origin = -100;
  Query(chart, object, &miss, "later-primitive-transformed-reference-nonzero-chart-origin");
}

static void CheckConditionalLinkage(bool requireRepair) {
  eSENCChart chart;
  Area area(chart);
  chart.razRules[1][3] = &area.rule;
  Object rock;
  rock.x = rock.y = 200;
  chart_context context{};
  rock.m_chart_context = &context;
  context.chart = &chart;
#ifdef PATCHED_PRIVATE
  context.get_associated_objects = &SkagerAssociatedObjects;
#endif
  auto invoke = [&]() {
    rock.m_DisplayCat = OTHER;
    ObjRazRules rules{}; rules.obj = &rock;
    std::unique_ptr<wxString> result(_UDWHAZ03(&rock, 0, &rules, nullptr));
    return result->Contains("SY(ISODGR51)");
  };
  const bool isolated = invoke();
  if (requireRepair) Check(isolated, "original-missing-query-negative");
#ifdef PATCHED_PRIVATE
  Check(isolated && rock.m_DisplayCat == DISPLAYBASE, "query-to-conditional-DisplayBase");
  area.depth = 2;
  Check(!invoke() && rock.m_DisplayCat == OTHER, "shallow-surrounding-area");
  area.depth = 10;
  rock.x = rock.y = 800;
  Check(!invoke() && rock.m_DisplayCat == OTHER, "polygon-miss-not-isolated-danger");
  rock.x = rock.y = 200;
  const int before = destroyedObjects;
  {
    std::unique_ptr<std::list<S57Obj*>> found(SkagerAssociatedObjects(&chart, &rock));
    Check(found && found->size() == 1 && found->front() == &area, "bridge-borrowed-identity");
  }
  Check(destroyedObjects == before && area.depth == 10, "bridge-container-not-object-owner");
  {
    S57Obj clone = rock; clone.bIsClone = true;
    std::unique_ptr<std::list<S57Obj*>> found(SkagerAssociatedObjects(&chart, &clone));
    Check(found && found->size() == 1 && found->front() == &area, "synchronous-clone-context");
  }
  Check(invoke(), "original-still-usable-after-clone");
  eSENCChart other;
  Check(SkagerAssociatedObjects(&other, &rock) == nullptr, "mismatched-chart-rejected");
  context.get_associated_objects = nullptr;
  Check(!invoke() && SkagerAssociatedObjects(&chart, &rock) == nullptr, "missing-callback");
  context.get_associated_objects = &SkagerAssociatedObjects;
  rock.m_chart_context = nullptr;
  Check(!invoke() && SkagerAssociatedObjects(&chart, &rock) == nullptr, "missing-context");
  Check(SkagerAssociatedObjects(nullptr, &rock) == nullptr &&
        SkagerAssociatedObjects(&chart, nullptr) == nullptr, "null-bridge-input");
  rock.m_chart_context = &context;
  chart.razRules[1][3] = nullptr;
  Check(!invoke(), "removed-area-not-cached");
  chart.razRules[1][3] = &area.rule;
  Check(invoke(), "restored-area-query");
  std::cout << "linkage repaired: actual area query -> callback -> UDWHAZ03 -> DisplayBase; borrowed synchronous lifetime passed\n";
#else
  Check(!isolated && rock.m_DisplayCat == OTHER, "original-missing-associated-area-query");
  std::cout << "linkage original: query works directly, conditional still omits it (retained negative)\n";
#endif
}

int main(int argc, char** argv) {
  try {
    CheckQuerySemantics();
    CheckTriangleStorage();
    CheckConditionalLinkage(argc > 1 && std::string(argv[1]) == "require-repair");
    std::cout << queries << " actual area-query cases; " << checks << " checks passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

#include <sqlite3.h>
#include <deque>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

void Check(bool value, const char* why) { if (!value) throw std::runtime_error(why); }
struct wxString : std::string {
  using std::string::string;
  wxString(const std::string& value = {}) : std::string(value) {}
  std::string ToStdString() const { return *this; }
};
struct Date { bool IsValid() const { return true; } long GetTicks() const { return 100; } };
struct Colour { wxString GetAsString(int) const { return "#123456"; } };
constexpr int wxC2S_HTML_SYNTAX = 0;
struct Hyperlink { std::string GUID; wxString DescrText, Link, LType; };
struct Links {
  struct Node {
    Hyperlink value; Node* next = nullptr;
    Hyperlink* GetData() { return &value; }
    Node* GetNext() { return next; }
  };
  std::deque<Node> nodes;
  Node* GetFirst() { return nodes.empty() ? nullptr : &nodes.front(); }
  void Add(std::string id) {
    nodes.push_back({{id, "new description", "https://example.invalid/" + id, "text"}, nullptr});
    if (nodes.size() > 1) nodes[nodes.size() - 2].next = &nodes.back();
  }
};
struct RoutePoint {
  wxString m_GUID, name = "new point", m_TideStation, m_timestring = "2026-10-19T01:00:00Z";
  Links links; Links* m_HyperlinkList = &links;
  double m_lat = 57, m_lon = 16, m_WaypointArrivalRadius = .1, m_fWaypointRangeRingsStep = 1;
  int m_iWaypointRangeRingsNumber = 1, m_iWaypointRangeRingsStepUnits = 0;
  bool m_bShowWaypointRangeRings = true, m_bIsolatedMark = false;
  Colour m_wxcWaypointRangeRingsColour;
  double GetLatitude() const { return m_lat; }
  double GetLongitude() const { return m_lon; }
  wxString GetIconName() const { return "diamond"; }
  wxString GetName() const { return name; }
  wxString GetDescription() const { return "new point detail"; }
  double GetPlannedSpeed() const { return 5; }
  Date GetManualETD() const { return {}; }
  int GetScaMin() const { return 0; }
  int GetScaMax() const { return 100000; }
  bool GetUseSca() const { return false; }
  bool IsVisible() const { return true; }
  bool IsNameShown() const { return true; }
  bool IsShared() const { return false; }
};
struct Route {
  wxString m_GUID = "r", name = "new route", m_RouteDescription = "new route detail";
  wxString m_RouteStartString, m_RouteEndString, m_TimeDisplayFormat, m_Colour;
  Date m_PlannedDeparture; double m_PlannedSpeed = 5;
  int m_width = 1, m_style = 0;
  Links links; Links* m_HyperlinkList = &links;
  std::vector<RoutePoint*> points;
  wxString GetName() const { return name; }
  bool IsVisible() const { return true; }
  bool GetSharedWPViz() const { return true; }
  int GetnPoints() const { return int(points.size()); }
  RoutePoint* GetPoint(int n) { return points.at(n - 1); }
};
struct TrackPoint {
  double m_lat = 57, m_lon = 16;
  std::string GetTimeString() const { return "2026-10-19T01:00:00Z"; }
};
struct Track {
  wxString m_GUID = "t", name = "new track", m_TrackDescription = "new track detail";
  wxString m_TrackStartString, m_TrackEndString, m_Colour;
  int m_width = 1, m_style = 0; bool m_bVisible = false;
  Links links; Links* m_TrackHyperlinkList = &links;
  std::vector<TrackPoint> points{{57,16},{57.1,16.1}};
  wxString GetName() const { return name; }
  int GetnPoints() const { return int(points.size()); }
  TrackPoint* GetPoint(int n) { return &points.at(n); }
};
struct NavObj_dB {
  sqlite3* m_db; bool m_importing = false;
  bool UpdateDBTrackAttributes(Track*);
  bool UpdateDBRouteAttributes(Route*);
  bool UpdateDBRoutePointAttributes(RoutePoint*);
  bool UpdateRoute(Route*);
  bool UpdateRoutePoint(RoutePoint*);
  bool InsertRoutePoint(RoutePoint*);
  bool UpdateTrack(Track*);
};
void ReportError(const std::string&) {}
bool executeSQL(sqlite3* db, const char* sql) { return sqlite3_exec(db, sql, nullptr, nullptr, nullptr) == SQLITE_OK; }
#include "native.inc"

struct Fixture {
  sqlite3* db = nullptr; NavObj_dB store{nullptr}; Route route; RoutePoint a, b; Track track;
  Fixture() {
    Check(sqlite3_open(":memory:", &db) == SQLITE_OK, "open fixture SQLite");
    store.m_db = db; Check(CreateTables(db), "native schema");
    Sql("PRAGMA foreign_keys=ON");
    Sql("INSERT INTO routes(guid,name) VALUES('r','old route'),('other','other route')");
    Sql("INSERT INTO routepoints(guid,name) VALUES('a','old a'),('b','old b')");
    Sql("INSERT INTO routepoints_link(route_guid,point_guid,point_order) VALUES('r','a',1),('r','b',2)");
    Sql("INSERT INTO route_html_links VALUES('rl','r','old url','old description','text')");
    Sql("INSERT INTO routepoint_html_links VALUES('pl','a','old url','old description','text')");
    Sql("INSERT INTO tracks(guid,name,visibility) VALUES('t','old track',1),('other','other track',1)");
    Sql("INSERT INTO trk_points VALUES('t',56,15,'old time',0),('t',56.1,15.1,'old time2',1)");
    Sql("INSERT INTO track_html_links VALUES('tl','t','old url','old description','text')");
    a.m_GUID = "a"; b.m_GUID = "b"; route.points = {&a,&b};
    route.links.Add("rl"); route.links.Add("rl2"); a.links.Add("pl"); a.links.Add("pl2");
    track.links.Add("tl"); track.links.Add("tl2");
  }
  ~Fixture() { Check(sqlite3_close(db) == SQLITE_OK, "all SQLite statements finalized"); }
  void Sql(const std::string& sql) { Check(executeSQL(db, sql.c_str()), sql.c_str()); }
  bool Query(const char* sql) {
    sqlite3_stmt* stmt=nullptr;
    Check(sqlite3_prepare_v2(db,sql,-1,&stmt,nullptr)==SQLITE_OK,"query prepare");
    const bool result=sqlite3_step(stmt)==SQLITE_ROW && sqlite3_column_int(stmt,0)==1;
    sqlite3_finalize(stmt);return result;
  }
  std::string Snapshot() {
    std::string result;
    for (const char* table : {"routes", "routepoints", "routepoints_link", "route_html_links",
                             "routepoint_html_links", "tracks", "trk_points", "track_html_links"}) {
      sqlite3_stmt* stmt = nullptr;
      const std::string sql = std::string("SELECT * FROM ") + table + " ORDER BY rowid";
      Check(sqlite3_prepare_v2(db,sql.c_str(),-1,&stmt,nullptr)==SQLITE_OK,"snapshot prepare");
      result += table;
      while (sqlite3_step(stmt)==SQLITE_ROW) {
        result += '\n';
        for (int i=0;i<sqlite3_column_count(stmt);++i) {
          const auto* text=sqlite3_column_text(stmt,i);
          result += text ? reinterpret_cast<const char*>(text) : "<NULL>";
          result += '|';
        }
      }
      sqlite3_finalize(stmt);
    }
    return result;
  }
};
int checks = 0;
void Fails(const std::string& trigger, const std::function<bool(Fixture&)>& action) {
  Fixture f; const auto before = f.Snapshot();
  f.Sql("CREATE TRIGGER injected " + trigger + " BEGIN SELECT RAISE(ABORT,'injected failure'); END");
  Check(!action(f), "injected failure must reach caller");
  Check(f.Snapshot()==before, "failed save must retain all previous object data and links");
  Check(sqlite3_get_autocommit(f.db), "failed update leaves no transaction open");
  f.Sql("DROP TRIGGER injected"); Check(action(f), "save recovers after failure removal"); ++checks;
}
int main() {
  const auto route=[](Fixture& f){return f.store.UpdateRoute(&f.route);};
  const auto point=[](Fixture& f){return f.store.UpdateRoutePoint(&f.a);};
  const auto track=[](Fixture& f){return f.store.UpdateTrack(&f.track);};
  const auto attributes=[](Fixture& f){return f.store.UpdateDBTrackAttributes(&f.track);};
  for(const auto& trigger : {
      "BEFORE UPDATE ON routes", "BEFORE UPDATE ON routepoints WHEN NEW.guid='b'",
      "BEFORE DELETE ON routepoints_link", "BEFORE INSERT ON routepoints_link WHEN NEW.point_order=2",
      "BEFORE DELETE ON route_html_links", "BEFORE INSERT ON route_html_links WHEN NEW.guid='rl2'",
      "BEFORE DELETE ON routepoint_html_links", "BEFORE INSERT ON routepoint_html_links WHEN NEW.guid='pl2'"})
    Fails(trigger, route);
  for(const auto& trigger : {"BEFORE UPDATE ON routepoints", "BEFORE DELETE ON routepoint_html_links",
                            "BEFORE INSERT ON routepoint_html_links WHEN NEW.guid='pl2'"}) Fails(trigger, point);
  for(const auto& action : std::vector<std::function<bool(Fixture&)>>{track, attributes})
    for(const auto& trigger : {"BEFORE UPDATE ON tracks", "BEFORE DELETE ON track_html_links",
                              "BEFORE INSERT ON track_html_links WHEN NEW.guid='tl2'"}) Fails(trigger, action);
  Fails("BEFORE INSERT ON trk_points WHEN NEW.point_order=1", track);
  Fails("BEFORE DELETE ON trk_points", track);
  Fails("BEFORE INSERT ON routepoints", [](Fixture& f) {
    f.b.m_GUID="new-point";return f.store.UpdateRoute(&f.route);
  });
  {
    Fixture f;RoutePoint created;created.m_GUID="created";created.links.Add("new-link");
    const auto before=f.Snapshot();
    f.Sql("CREATE TRIGGER injected BEFORE INSERT ON routepoint_html_links BEGIN SELECT RAISE(ABORT,'injected'); END");
    Check(!f.store.InsertRoutePoint(&created),"named waypoint creation reports hyperlink failure");
    Check(f.Snapshot()==before,"failed named waypoint creation leaves no partial row");
    f.Sql("DROP TRIGGER injected");
    Check(f.store.InsertRoutePoint(&created),"named waypoint creation recovers");
    Check(f.Query("SELECT name='new point' FROM routepoints WHERE guid='created'"),"created waypoint name persists");
    Check(f.Query("SELECT count(*)=1 FROM routepoint_html_links WHERE routepoint_guid='created'"),"created waypoint links persist");++checks;
  }
  for(const auto& action : std::vector<std::function<bool(Fixture&)>>{route, point, track, attributes}) {
    Fixture f; const auto before=f.Snapshot();
    sqlite3_commit_hook(f.db, [](void*){return 1;}, nullptr);
    Check(!action(f), "commit refusal reaches caller");
    sqlite3_commit_hook(f.db, nullptr, nullptr);
    Check(f.Snapshot()==before && sqlite3_get_autocommit(f.db), "commit refusal rolls back entire operation");
    Check(action(f), "save succeeds after commit refusal removed"); ++checks;
  }
  for(const auto& table : {"routepoints_link", "route_html_links", "routepoint_html_links"}) {
    Fixture f; const auto before=f.Snapshot();
    sqlite3_set_authorizer(f.db, [](void* name,int op,const char* table,const char*,const char*,const char*) {
      return op==SQLITE_DELETE && table && std::string(table)==static_cast<const char*>(name) ? SQLITE_DENY : SQLITE_OK;
    }, const_cast<char*>(table));
    Check(!route(f), "prepare/authorization failure reaches caller");
    sqlite3_set_authorizer(f.db,nullptr,nullptr);
    Check(f.Snapshot()==before, "prepare failure rolls back preceding writes"); ++checks;
  }
  for(bool is_track : {false,true}) {
    Fixture f;
    f.Sql(is_track ? "INSERT INTO track_html_links VALUES('tl2','other','foreign','foreign','text')" :
                    "INSERT INTO route_html_links VALUES('rl2','other','foreign','foreign','text')");
    const auto before=f.Snapshot();
    Check(!(is_track ? attributes(f) : route(f)), "foreign hyperlink GUID collision is refused");
    Check(f.Snapshot()==before, "foreign hyperlink and own data survive collision"); ++checks;
  }
  {
    Fixture f; const auto before=f.Snapshot(); f.Sql("BEGIN");
    Check(route(f) && attributes(f), "native outer transaction supports nested object saves");
    f.Sql("ROLLBACK"); Check(f.Snapshot()==before, "outer rollback retains both objects"); ++checks;
  }
  {
    Fixture f; f.Sql("DELETE FROM tracks WHERE guid='t'"); const auto before=f.Snapshot();
    Check(!attributes(f) && f.Snapshot()==before, "missing track row cannot report rename success"); ++checks;
  }
  for(const auto& action : std::vector<std::function<bool(Fixture&)>>{route, point, attributes}) {
    Fixture f;const auto before=f.Snapshot();
    f.route.name=f.a.name=f.track.name=std::string(2048,'x');
    const int limit=sqlite3_limit(f.db,SQLITE_LIMIT_LENGTH,512);
    Check(!action(f),"oversized bound attribute cannot become a successful NULL write");
    sqlite3_limit(f.db,SQLITE_LIMIT_LENGTH,limit);
    Check(f.Snapshot()==before,"binding failure retains prior attributes and hyperlinks");++checks;
  }
  {
    Fixture f;Check(route(f)&&track(f),"complete native saves succeed");
    Check(f.Query("SELECT name='new route' AND description='new route detail' FROM routes WHERE guid='r'"),
          "route attributes persist");
    Check(f.Query("SELECT name='new point' AND viz_name=1 FROM routepoints WHERE guid='a'"),"waypoint naming persists");
    Check(f.Query("SELECT count(*)=2 FROM routepoints_link WHERE route_guid='r'"),"route point count retained");
    Check(f.Query("SELECT point_guid='b' FROM routepoints_link WHERE route_guid='r' AND point_order=2"),"route ordering retained");
    Check(f.Query("SELECT name='new track' AND visibility=0 FROM tracks WHERE guid='t'"),"track name and visibility persist");
    Check(f.Query("SELECT count(*)=2 FROM trk_points WHERE track_guid='t'"),"track point count retained");
    Check(f.Query("SELECT latitude=57.1 AND longitude=16.1 FROM trk_points WHERE track_guid='t' AND point_order=1"),"track geometry persists");
    Check(f.Query("SELECT count(*)=2 FROM track_html_links WHERE track_guid='t' AND html_description='new description'"),"track hyperlinks persist");
    Check(f.Query("SELECT count(*)=2 FROM route_html_links WHERE route_guid='r' AND html_description='new description'"),"route hyperlinks persist");
    Check(f.Query("SELECT count(*)=2 FROM routepoint_html_links WHERE routepoint_guid='a' AND html_description='new description'"),"waypoint hyperlinks persist");++checks;
  }
  std::cout << "navobj persistence: " << checks << " SQLite scenarios passed\n";
}

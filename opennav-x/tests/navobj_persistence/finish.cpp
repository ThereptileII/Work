#define OPENNAV_X
#include <iostream>
#include <stdexcept>

void Check(bool value,const char* why) { if(!value) throw std::runtime_error(why); }
struct Route { int count=2; int GetnPoints() const {return count;} void SetHiLite(int) {} };
struct NavObj_dB {
  int writes=0; bool succeeds=true;
  static NavObj_dB& GetInstance(){static NavObj_dB db;return db;}
  bool UpdateRoute(Route*){++writes;return succeeds;}
};
struct Frame { void SetMasterToolbarItemState(int,bool){} void RefreshAllCanvas(bool){} } frame;
auto* gFrame=&frame;
struct Toolbar {void EnableTooltips(){}};
Toolbar* g_MainToolbar=nullptr;
constexpr int ID_ROUTE=1,ID_MENU_ROUTE_NEW=2;
int cursor=0;int* pCursorArrow=&cursor;
bool g_brouteCreating=true;
struct Manager {int deleted=0;void DeleteRoute(Route*){++deleted;}} manager;
auto* g_pRouteMan=&manager;
struct RoutePropDlgImpl {
  static bool getInstanceFlag(){return false;}
  bool IsShown(){return false;} void SetRouteAndUpdate(Route*,bool){}
};
RoutePropDlgImpl* pRoutePropDialog=nullptr;
struct RouteManagerDialog {
  static bool getInstanceFlag(){return false;}
  bool IsShown(){return false;} void UpdateRouteListCtrl(){}
};
RouteManagerDialog* pRouteManagerDialog=nullptr;
struct Undo {void InvalidateUndo(){}};
struct ChartCanvas {
  int m_routeState=3;void* m_prev_pMousePoint=nullptr;bool m_bDrawingRoute=true;
  bool m_bAppendingRoute=false;Route* m_pMouseRoute=nullptr;Route* m_pSelectedRoute=nullptr;
  Undo owned_undo;Undo* undo=&owned_undo;
  void SetCursor(int){}void ShowGlobalToolbar(){}void FinishRoute();
};
namespace opennav {
bool xnav=true,already_saved=false;
bool IsXNav(){return xnav;}
bool PrepareRouteFinish(ChartCanvas& canvas) {
  return !xnav || !canvas.m_routeState || canvas.m_bAppendingRoute ||
      !canvas.m_pMouseRoute || canvas.m_pMouseRoute->GetnPoints()<2 || already_saved ||
      NavObj_dB::GetInstance().UpdateRoute(canvas.m_pMouseRoute);
}
}
#include "finish.inc"
int main(){
  int cases=0;
  for(bool xnav:{false,true})for(bool append:{false,true}){
    Route route;ChartCanvas canvas;canvas.m_pMouseRoute=&route;canvas.m_bAppendingRoute=append;
    opennav::xnav=xnav;opennav::already_saved=false;NavObj_dB::GetInstance()={};
    canvas.FinishRoute();
    Check(NavObj_dB::GetInstance().writes==1,"successful Finish persists once in XNav/Legacy/new/append");
    Check(!canvas.m_routeState&&!canvas.m_pMouseRoute&&!g_brouteCreating,"successful Finish clears draft state");++cases;
  }
  {
    Route route;ChartCanvas canvas;canvas.m_pMouseRoute=&route;opennav::xnav=true;
    auto& db=NavObj_dB::GetInstance();db={};db.succeeds=false;g_brouteCreating=true;
    canvas.FinishRoute();
    Check(db.writes==1&&canvas.m_routeState==3&&canvas.m_pMouseRoute==&route&&g_brouteCreating,
          "failed prepared save retains editable draft and does not try an unchecked second write");++cases;
  }
  {
    Route route;ChartCanvas canvas;canvas.m_pMouseRoute=&route;
    opennav::already_saved=true;NavObj_dB::GetInstance()={};
    canvas.FinishRoute();
    Check(NavObj_dB::GetInstance().writes==0&&!canvas.m_routeState,
          "named Save already persisted before native Finish");++cases;
  }
  {
    Route route;route.count=1;ChartCanvas canvas;canvas.m_pMouseRoute=&route;
    opennav::already_saved=false;NavObj_dB::GetInstance()={};manager.deleted=0;
    canvas.FinishRoute();
    Check(manager.deleted==1&&!canvas.m_pMouseRoute&&NavObj_dB::GetInstance().writes==0,
          "short draft retains native cleanup without persistence");++cases;
  }
  std::cout<<"native FinishRoute: "<<cases<<" scenarios passed\n";
}

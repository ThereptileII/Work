#include <wx/wx.h>
#include <stdexcept>
#include <iostream>
#include "/home/standard/Projects/X-nav-worktrees/linux-workspace-readiness-20261015/opennav-x/src/integration/DashboardPresentation.cpp"
OCPN_AUIManager* g_pauimgr=nullptr;
namespace opennav { bool IsXNav() { return true; } }
using namespace opennav::integration;
static unsigned checks=0;
void Check(bool ok,const char* message){++checks;if(!ok)throw std::runtime_error(message);}
void Record(const char* message){std::cout<<message<<"\n";}
class App:public wxApp{public:bool OnInit() override{return true;}};
wxIMPLEMENT_APP_NO_MAIN(App);
int main(int argc,char**argv){
if(!wxEntryStart(argc,argv)||!wxTheApp->CallOnInit())return 2;
Check(!presentation,"No global presentation exists before observation");
Check(!TestDashboardPresentationReady(),"Absent presentation is not ready");
Check(!presentation,"Readiness observation must not create presentation");
      {
        auto* other = new wxFrame(nullptr, wxID_ANY, "Presentation lifetime check");
        wxAuiManager manager(other);
        auto* visible = new wxPanel(other);
        auto* hidden = new wxPanel(other);
        auto* docked = new wxPanel(other);
        auto* foreign = new wxPanel(other);
        manager.AddPane(visible, wxAuiPaneInfo().Name("localized-owner-1").Caption("user caption").Float().FloatingPosition(37, 81).FloatingSize(220, 180));
        manager.AddPane(hidden, wxAuiPaneInfo().Name("localized-owner-2").Float().FloatingPosition(51, 93).Hide());
        manager.AddPane(docked, wxAuiPaneInfo().Name("localized-owner-3").Right().Row(2).Position(3).BestSize(160, 220));
        manager.AddPane(foreign, wxAuiPaneInfo().Name("Dashboard").Left());
        manager.Update();
        const auto saved = manager.SavePerspective();
        DashboardPresentation presentation(manager);
        presentation.Register(visible); presentation.Register(hidden); presentation.Register(docked);
        Check(!presentation.TestReady(), "Registered Dashboard is not ready before deferred enablement");
        presentation.Enable(true);
        Check(presentation.TestReady(), "Completed Dashboard enablement is observable without changing pane state");
        Check(!manager.GetPane(visible).IsShown() && !manager.GetPane(hidden).IsShown() &&
              !manager.GetPane(docked).IsShown() && manager.GetPane(foreign).IsShown(),
              "Only registered Dashboard windows are suppressed; names cannot grant ownership");
        presentation.BeginLayout(); presentation.BeginLayout();
        Check(!presentation.TestReady(), "Restoring panes during nested layout is not presentation readiness");
        Check(manager.GetPane(visible).IsShown() && !manager.GetPane(hidden).IsShown() && manager.GetPane(docked).IsShown(),
              "Plugin callbacks see original visible/hidden and docked state");
        presentation.EndLayout();
        Check(!presentation.TestReady(), "Inner layout completion cannot signal presentation readiness");
        Check(manager.GetPane(visible).IsShown(), "Nested callback cannot resuppress outer layout");
        presentation.EndLayout();
        Check(presentation.TestReady(), "Outer layout completion restores presentation readiness");
        Check(!manager.GetPane(visible).IsShown(), "Outer layout completion reapplies temporary presentation");
        auto* late = new wxPanel(other);
        manager.AddPane(late, wxAuiPaneInfo().Name("late pane").Float());
        presentation.Register(late); presentation.Register(late);
        presentation.BeginLayout();
        Check(manager.GetPane(visible).IsShown() && manager.GetPane(late).IsShown(),
              "Late registration and duplicate registration do not overwrite preserved state");
        manager.GetPane(visible).FloatingPosition(65, 98);
        presentation.EndLayout();
        presentation.BeginLayout();
        Check(manager.GetPane(visible).floating_pos == wxPoint(65, 98), "Explicit plugin layout change survives suppression");
        manager.DetachPane(late); delete late;  // Weak ownership: no DeInit callback required.
        manager.LoadPerspective(saved, false);
        presentation.EndLayout();
        presentation.Enable(false);
        Check(!presentation.TestReady(), "Disabled presentation is not ready");
        Check(manager.SavePerspective() == saved, "Exact original floating/docked workspace restored before persistence");
        presentation.Enable(true);
        manager.DetachPane(hidden); presentation.Unregister(hidden); delete hidden;
        auto* recreated = new wxPanel(other);
        manager.AddPane(recreated, wxAuiPaneInfo().Name("localized-owner-2").Float().Hide());
        presentation.Register(recreated); presentation.Enable(false);
        Check(!manager.GetPane(recreated).IsShown() && manager.GetPane(visible).IsShown(),
              "Deleted/recreated pane keeps its own visibility instead of an old object's state");
        manager.UnInit(); other->Destroy();
        Record("Dashboard presentation preserves ownership, nesting, reload, geometry and deleted/recreated window lifetime");
      }

Check(!presentation,"Local component fixture did not create global presentation");
wxTheApp->ProcessPendingEvents();wxTheApp->OnExit();wxEntryCleanup();
std::cout<<checks<<" actual Dashboard checks passed\n";return 0;}

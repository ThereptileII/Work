// Offline wxMemoryDC proof of the production label painter, not an ENC/GL test.
#include "integration/OnlineAisLabels.h"
#include <wx/app.h>
#include <wx/dcmemory.h>
#include <wx/frame.h>
#include <wx/image.h>
#include <iostream>
#include <stdexcept>

using namespace opennav;
namespace {
unsigned checks=0;
void Check(bool ok,const char *why) { ++checks;if(!ok)throw std::runtime_error(why); }
struct Drawn { wxString text;int x,y;wxColour ink;wxFont font; };
// Observe and forward to the actual wx drawing context; no replacement renderer.
struct ObservedDC {
  wxMemoryDC &dc;std::vector<Drawn> drawn;
  wxFont GetFont() const{return dc.GetFont();}
  wxColour GetTextForeground() const{return dc.GetTextForeground();}
  void SetFont(const wxFont &v){dc.SetFont(v);}
  void SetTextForeground(const wxColour &v){dc.SetTextForeground(v);}
  void GetTextExtent(const wxString &s,int *w,int *h,int *d){dc.GetTextExtent(s,w,h,d);}
  void DrawText(const wxString &s,int x,int y){drawn.push_back({s,x,y,dc.GetTextForeground(),dc.GetFont()});dc.DrawText(s,x,y);}
};
class App:public wxApp {public:bool OnInit() override{return true;}};
}
wxIMPLEMENT_APP_NO_MAIN(App);
int main(int argc,char **argv) {
  if(argc!=2){std::cerr<<"Expected output PNG path\n";return 2;}
  const std::string output=argv[1];
  if(!wxEntryStart(argc,argv)||!wxTheApp->CallOnInit())return 2;
  int result=0;
  try {
    wxInitAllImageHandlers();
    auto *window=new wxFrame(nullptr,wxID_ANY,"OFFLINE AIS LABEL FIXTURE");
    Check(window->FromDIP(100)==100,"Fixture must use actual 96-DPI logical pixels");
    wxBitmap combined(960,320,24);wxMemoryDC canvas(combined);
    const std::uint32_t water[]={0xD5E5E5,0x344F59,0x121E24};
    for(int theme=0;theme<3;++theme) {
      const auto mode=static_cast<ui::LightMode>(theme);
      wxBitmap tile(320,320,24);wxMemoryDC dc(tile);
      dc.SetBackground(wxBrush(ui::Colour(water[theme])));dc.Clear();
      dc.SetFont(ui::UiFont(*window,12));dc.SetTextForeground(ui::Colour(ui::FloatingTheme(mode).primary));
      dc.DrawText(wxString::FromUTF8(theme==0?"DAY · OFFLINE":theme==1?"DUSK · OFFLINE":"NIGHT · OFFLINE"),16,14);
      std::vector<integration::OnlineAisLabelTarget> targets;
      auto add=[&](int n,const char *name,int x,int y,ais::TargetAge age,bool selected=false){
        ais::ChartTarget t;t.mmsi=265000000+n;t.name=name;t.age=age;t.selected=selected;
        targets.push_back({t,{x,y}});
      };
      // Lower-MMSI collision must not win against the selected target.
      add(1,"COLLISION",30,72,ais::TargetAge::Live);
      add(2,"FIXTURE ALPHA",30,70,ais::TargetAge::Live,true);
      add(3,"FIXTURE BETA",30,130,ais::TargetAge::Aging);
      add(4,"",30,180,ais::TargetAge::Live);
      add(5,"CLIPPED",305,230,ais::TargetAge::Live);
      add(6,"STALE FIXTURE",30,250,ais::TargetAge::Stale);
      add(7,"\xff",30,290,ais::TargetAge::Live);
      const auto old_font=dc.GetFont();const auto old_ink=dc.GetTextForeground();
      ObservedDC observed{dc,{}};
      Check(integration::DrawOnlineAisLabels(observed,*window,mode,{320,320},targets)==2,
            "Only named live/aging labels that fit without collisions draw");
      Check(observed.drawn[0].text=="FIXTURE ALPHA"&&observed.drawn[1].text=="FIXTURE BETA",
            "Selected label wins, missing/stale/invalid/clipped/colliding labels omitted");
      Check(dc.GetFont()==old_font&&dc.GetTextForeground()==old_ink,"Drawing state restored");
      for(int i=0;i<2;++i) {
        const auto &d=observed.drawn[i];int width=0,height=0,descent=0;
        dc.SetFont(d.font);dc.GetTextExtent(d.text,&width,&height,&descent);
        Check(d.x==44&&d.y+height-descent==(i==0?66:126),"Exact x14/y-4 alphabetic baseline");
        Check(d.font==ui::UiFont(*window,9)&&d.ink==ui::Colour(ui::OnlineChartTheme(mode).label),
              "Production 9px font and effective theme ink used");
      }
      dc.SetFont(old_font);
      // Reference anchors only, not a substitute for production AIS geometry.
      dc.SetPen(wxPen(ui::Colour(ui::OnlineChartTheme(mode).stroke),1));dc.SetBrush(*wxTRANSPARENT_BRUSH);
      dc.DrawCircle(30,70,18);dc.DrawCircle(30,130,6);
      canvas.Blit(theme*320,0,320,320,&dc,0,0);
      dc.SelectObject(wxNullBitmap);
    }
    canvas.SelectObject(wxNullBitmap);
    Check(combined.ConvertToImage().SaveFile(wxString::FromUTF8(output),wxBITMAP_TYPE_PNG),"Fixture PNG saved");
    delete window;
    std::cout<<checks<<" label painter checks passed; fixture only, no ENC/GL/native Windows qualification\n";
  }catch(const std::exception &e){std::cerr<<e.what()<<'\n';result=1;}
  wxTheApp->OnExit();wxEntryCleanup();return result;
}

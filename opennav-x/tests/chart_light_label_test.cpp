// Offline production classifier, custom-appearance guard and shared raster.
#include "integration/ChartLightLabel.h"
#include <wx/app.h>
#include <chrono>
#include <iostream>
#include <stdexcept>
class App : public wxApp {public: bool OnInit() override{return true;}};
wxIMPLEMENT_APP_NO_MAIN(App);
int main(int argc,char** argv) {
  if(argc!=2||!wxEntryStart(argc,argv)||!wxTheApp->CallOnInit())return 2;
  int result=0,checks=0;
  auto check=[&](bool ok,const char* why){++checks;if(!ok)throw std::runtime_error(why);};
  try {
    using namespace opennav::integration;wxInitAllImageHandlers();
    for(const char* suffix:{"',3,3,3,'15110',2,-1,CHBLK,23)","',3,2,3,'15110',2,0,CHBLK,23)","',3,2,3,'15110',2,1,CHBLK,23)"}) {
      const std::string rule=std::string("'Fl(2) W 10s 25ft 8Nm")+suffix+";SY(OTHER)\037";
      check(IsGeneratedLightDescription("LIGHTS",rule.c_str(),true),"Actual instruction buffer may include following rules");
      check(!IsGeneratedLightDescription("WRECKS",rule.c_str(),true),"Other hazards stay stock");
      check(!IsGeneratedLightDescription("LIGHTS",rule.c_str(),false),"TE stays stock");
    }
    for(const char* rule:{"OBJNAM,3,2,3,'15110',2,0,CHBLK,23)","'Fl',3,2,3,'16110',2,0,CHBLK,23)","'Fl',3,2,3,'15110',2,0,CHBLK,21)","'Fl',3,2,3,'15110',2,0,CHRED,23)"})
      check(!IsGeneratedLightDescription("LIGHTS",rule,true),"Only exact normal description class accepted");
    wxFont system=*wxNORMAL_FONT,custom=system;
    check(FactoryLightTextFont(system,system,*wxBLACK),"Automatically saved factory appearance remains eligible");
    custom.SetPointSize(system.GetPointSize()+1);check(!FactoryLightTextFont(custom,system,*wxBLACK),"Custom size retained");
    custom=system;custom.SetWeight(wxFONTWEIGHT_BOLD);check(!FactoryLightTextFont(custom,system,*wxBLACK),"Custom weight retained");
    custom=system;custom.SetStyle(wxFONTSTYLE_ITALIC);check(!FactoryLightTextFont(custom,system,*wxBLACK),"Custom style retained");
    custom=system;custom.SetUnderlined(true);check(!FactoryLightTextFont(custom,system,*wxBLACK),"Custom underline retained");
    check(!FactoryLightTextFont(system,system,wxColour(255,0,0)),"Custom color retained");
    wxBitmap canvas(960,330,24);wxMemoryDC dc(canvas);
    const wxColour water[]={{213,229,229},{52,79,89},{18,30,36}};
    const wxColour ink[]={{104,123,122},{173,187,177},{117,133,121}};
    const wxColour shallow[]={{134,172,182},{113,140,147},{61,85,96}};
    wxFont font(6,wxFONTFAMILY_SWISS,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_NORMAL,false,"Arial");
    const wxString text="Fl(2) W 10s 25ft 8Nm";
    for(int theme=0;theme<3;++theme) {
      const int x=theme*320;
      dc.SetPen(*wxTRANSPARENT_PEN);dc.SetBrush(wxBrush(water[theme]));dc.DrawRectangle(x,0,320,330);
      dc.SetFont(wxFont(9,wxFONTFAMILY_SWISS,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_NORMAL));dc.SetTextForeground(ink[theme]);
      dc.DrawText(theme==0?"DAY / OFFLINE LIGHT LABELS":theme==1?"DUSK":"NIGHT",x+12,12);
      dc.SetBrush(wxBrush(shallow[theme]));dc.DrawRectangle(x,90,320,70);
      ChartLightLabelRaster raster;
      const auto coldStart=std::chrono::steady_clock::now();
      check(raster.Build(font,text,1,ink[theme],water[theme]),"Shared raster succeeds");
      std::cout<<"theme "<<theme<<" first raster "<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-coldStart).count()<<"us\n";
      check(raster.margin==3,"Fractional round halo fits with antialias border");
      dc.DrawBitmap(raster.bitmap,x+15,50,true);dc.DrawBitmap(raster.bitmap,x+15,112,true);
      const auto first=raster.builds;
      auto start=std::chrono::steady_clock::now();
      bool cached=true;
      for(int i=0;i<10000;++i)cached = raster.Build(font,text,1,ink[theme],water[theme]) && cached;
      check(cached,"Cached raster retained");
      check(raster.builds==first,"Identical frames never rerasterize");
      const auto us=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-start).count();
      std::cout<<"theme "<<theme<<" 10000 cache hits "<<us<<"us\n";
      int halo=0,glyph=0,visible=0;
      for(int y=0;y<raster.image.GetHeight();++y)for(int xx=0;xx<raster.image.GetWidth();++xx) {
        const auto a=raster.image.GetAlpha(xx,y);if(!a)continue;++visible;
        const wxColour c(raster.image.GetRed(xx,y),raster.image.GetGreen(xx,y),raster.image.GetBlue(xx,y));
        if(c==water[theme])++halo;
        if(c==ink[theme])++glyph;
      }
      std::cout<<"halo="<<halo<<" opaque glyph="<<glyph<<" visible="<<visible<<"\n";
      check(halo>20&&visible-halo>100,"Water halo and distinct antialiased foreground strokes remain visible");
      wxFont big=font;big.SetPointSize(12);
      check(raster.Build(big,text,2,ink[theme],water[theme]),"User enlargement builds new raster");
      dc.DrawBitmap(raster.bitmap,x+15,195,true);
      check(raster.builds==first+1,"Changed font/scale invalidates cache");
      check(!raster.Build(big,wxString('X',257),2,ink[theme],water[theme]),"Bounded text rejects atomically");
      check(!raster.Build(big,text,5,ink[theme],water[theme]),"Unsupported scale falls back");
    }
    dc.SelectObject(wxNullBitmap);check(canvas.ConvertToImage().SaveFile(argv[1],wxBITMAP_TYPE_PNG),"Fixture saved");
    std::cout<<checks<<" light label checks passed\n";
  }catch(const std::exception&e){std::cerr<<e.what()<<'\n';result=1;}
  wxTheApp->OnExit();wxEntryCleanup();return result;
}

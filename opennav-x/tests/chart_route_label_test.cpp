// Native production raster fixture; no route/model or sensor data is injected.
#include "integration/ChartRouteLabelRaster.h"
#include <wx/app.h>
#include <wx/fontenum.h>
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace opennav::integration;
class App:public wxApp { public: bool OnInit() override{return true;} };
wxIMPLEMENT_APP_NO_MAIN(App);
int main(int argc,char** argv) {
  if(!wxEntryStart(argc,argv) || !wxTheApp->CallOnInit())return 2;
  int result=0,checks=0;
  const auto check=[&](bool ok,const char* why){++checks;if(!ok)throw std::runtime_error(why);};
  try {
    wxInitAllImageHandlers();
    check(argc==2,"Supply fixture output PNG");
    wxFont factory=*wxNORMAL_FONT;
    check(FactoryRouteLabelFont(factory,factory,*wxBLACK),"Factory font eligible");
    for(int change=0;change<5;++change) {
      auto custom=factory;wxColour ink=*wxBLACK;
      if(change==0)custom.SetPointSize(factory.GetPointSize()+1);
      if(change==1)custom.SetWeight(wxFONTWEIGHT_BOLD);
      if(change==2)custom.SetStyle(wxFONTSTYLE_ITALIC);
      if(change==3)custom.SetUnderlined(true);
      if(change==4)ink=*wxRED;
      check(!FactoryRouteLabelFont(custom,factory,ink),"Custom font or colour remains stock");
    }
    wxString face="Arial";
    for(const auto* candidate:{"Segoe UI Variable Display","Segoe UI","Arial"})
      if(wxFontEnumerator::IsValidFacename(candidate)){face=candidate;break;}
#ifdef __WXMSW__
    wxFont font(wxFontInfo(wxSize(0,10)).FaceName(face));
#else
    wxFont font(wxFontInfo(7.5).FaceName(face));
#endif
    font.SetWeight(wxFONTWEIGHT_NORMAL);
    wxBitmap picture(1080,460,24);wxMemoryDC dc(picture);
    const wxColour water[]={{213,229,229},{52,79,89},{18,30,36}};
    for(int mode=0;mode<3;++mode) {
      const int origin=mode*360;
      dc.SetPen(*wxTRANSPARENT_PEN);dc.SetBrush(wxBrush(water[mode]));
      dc.DrawRectangle(origin,0,360,460);
      ChartRouteLabelRaster raster;
      check(raster.Build(font,"Westhaven",1,mode,false),"Default native text raster");
      check(raster.bounds.Contains(-18,18)&&raster.bounds.Contains(56,43),"Real card coordinates included");
      check(!raster.texture_current,"New pixels require GL upload");
      check(raster.bounds.width<raster.image.GetWidth() &&
            raster.bounds.height<raster.image.GetHeight(),"Transparent POT padding is not hit/cull area");
      bool contained=true;
      for(int y=0;y<raster.image.GetHeight();++y)for(int x=0;x<raster.image.GetWidth();++x)
        if(raster.image.GetAlpha(x,y))contained &= raster.bounds.Contains(x+raster.origin.x,y+raster.origin.y);
      check(contained,"Every painted pixel belongs to tight bounds");
      unsigned int texture=17;int deletes=0;
      raster.texture_owned=raster.texture_current=true;
      check(!raster.FailTexture(texture,[&](unsigned int){++deletes;}) &&
            texture==0 && deletes==1 && !raster.texture_owned && !raster.texture_current,
            "Failed upload drops only the owned styled texture and clears ownership");
      texture=23; // Existing stock builder recreated its own texture.
      check(!raster.FailTexture(texture,[&](unsigned int){++deletes;}) &&
            texture==23 && deletes==1,"Repeated failure preserves the stock texture without deletion churn");
      check(raster.Build(font,"Westhaven",1,mode,false)&&raster.texture_failed,"Same appearance retains known upload failure");
      raster.texture_failed=false;
      raster.texture_current=raster.texture_owned=true;
      const auto before=raster.builds;
      const auto start=std::chrono::steady_clock::now();
      bool hits=true;
      for(int n=0;n<10000;++n)hits &= raster.Build(font,"Westhaven",1,mode,false);
      check(hits,"Repeated cache hits");
      std::cout<<"theme "<<mode<<" 10000 cache hits "
          <<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-start).count()<<"us\n";
      check(raster.builds==before&&raster.texture_current,"Stable frames retain texture and raster");
      const auto colours=RouteLabelPalette(mode);
      int solid=0,ink=0;
      for(int y=0;y<raster.image.GetHeight();++y)for(int x=0;x<raster.image.GetWidth();++x) {
        if(!raster.image.GetAlpha(x,y))continue;
        const wxColour c(raster.image.GetRed(x,y),raster.image.GetGreen(x,y),raster.image.GetBlue(x,y));
        if(c==colours.fill)++solid;
        if(std::abs(c.Red()-colours.fill.Red())+std::abs(c.Green()-colours.fill.Green())+std::abs(c.Blue()-colours.fill.Blue())>80 && raster.image.GetAlpha(x,y)>200)++ink;
      }
      std::cout<<"solid="<<solid<<" contrast glyph pixels="<<ink<<"\n";
      check(solid>500&&ink>20,"Opaque floating surface and legible native ink");
      dc.DrawBitmap(raster.bitmap,origin+60+raster.origin.x,20+raster.origin.y,true);
      check(raster.Build(font,"Westhaven",1,mode,true),"Last placement");
      check(raster.bounds.x<= -88,"Last placement uses name width");
      check(!raster.texture_failed,"Changed appearance may retry a failed upload");
      check(!raster.texture_current&&raster.texture_owned,"Changed layout invalidates upload without forgetting its owner");
      dc.DrawBitmap(raster.bitmap,origin+300+raster.origin.x,70+raster.origin.y,true);
      const auto long_name=wxString("Long actual waypoint name continues beyond its card");
      check(raster.Build(font,long_name,1,mode,false),"Long name kept");
      check(raster.bounds.GetRight()>152,"Full glyph overflow included in dirty bounds");
      dc.DrawBitmap(raster.bitmap,origin+28+raster.origin.x,120+raster.origin.y,true);
      int row=0;
      for(const auto* text:{"Ångbåtsbryggan","مرحبا بالميناء","港口入口","Café ⚓"}) {
        check(raster.Build(font,wxString::FromUTF8(text),1,mode,false),"Whole native Unicode run shapes");

        dc.DrawBitmap(raster.bitmap,origin+50+raster.origin.x,175+row*25+raster.origin.y,true);
        ++row;
      }
      const wxString complex=wxString::FromUTF8("مرحبا بالميناء 港口 ⚓");
      check(raster.Build(font,complex,1,mode,false),"Complex native run");
      dc.DrawBitmap(raster.bitmap,origin+50+raster.origin.x,280+raster.origin.y,true);
      auto big=font;big.SetFractionalPointSize(font.GetFractionalPointSize()*1.5);
      check(raster.Build(big,"Westhaven",1.5,mode,false),"150 percent geometry and font");
      dc.DrawBitmap(raster.bitmap,origin+70+raster.origin.x,335+raster.origin.y,true);
      check(!raster.Build(font,"",1,mode,false),"Empty name stock fallback");
      check(!raster.Build(font,"Two\nlines",1,mode,false),"Multiline name unchanged by fallback");
      check(!raster.Build(font,wxString('W',257),1,mode,false),"Bounded long run fallback");
      check(!raster.Build(font,"Name",std::numeric_limits<double>::infinity(),mode,false),"Invalid scale fallback");
      check(!raster.Build(font,"Name",5,mode,false),"Unsupported scale fallback");
    }
    check(RouteLabelPalette(2).fill==wxColour(16,26,32)&&
          RouteLabelPalette(2).text==wxColour(142,152,136)&&
          RouteLabelPalette(2).border==wxColour(87,111,94,26),"Night ancestor brightness changes RGB, not alpha");
    dc.SelectObject(wxNullBitmap);
    check(picture.ConvertToImage().SaveFile(argv[1],wxBITMAP_TYPE_PNG),"Save raster evidence");
    std::cout<<checks<<" route label checks passed\n";
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';result=1;}
  wxTheApp->OnExit();wxEntryCleanup();return result;
}

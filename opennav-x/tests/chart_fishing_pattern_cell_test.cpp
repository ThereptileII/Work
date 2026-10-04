#include <wx/wx.h>
#include <wx/filename.h>
#include <map>
#include <iostream>
#include <stdexcept>
#include <limits>
#define private public
#include "chartsymbols.h"
#undef private
#include "integration/ChartFishingPattern.h"

// Only HPGL drawing and the owner container are fixtures. The actual pinned
// loader and complete CreatePatternBufferSpec body execute below. The fallback
// records its invocation; this does not qualify the original HPGL painting.
struct RecordedHPGL {
  int calls = 0;
  void SetTargetDC(wxMemoryDC*) {}
  void SetVP(int*) {}
  void Render(char*,char*,wxPoint,wxPoint,wxPoint,float,int,bool) { ++calls; }
};
class s52plib {
 public:
  wxArrayPtrVoid* pAlloc;
  std::map<wxString,Rule*> *_symb_sym, *_patt_sym;
  ChartSymbols m_chartSymbols;
  wxColour m_unused_wxColor{1,2,3};
  bool m_presentationLightSymbols = true;
  double canvas_pix_per_mm = 96.0/25.4;
  int vp_plib = 0, table = 0;
  RecordedHPGL hpgl;
  RecordedHPGL* HPGL = &hpgl;
  S52color* getColor(const char* name) { return m_chartSymbols.GetColor(name,table); }
  void DestroyPatternRuleNode(Rule*) { throw std::runtime_error("Unexpected duplicate pattern"); }
  render_canvas_parms* CreatePatternBufferSpec(ObjRazRules*,Rules*,bool,bool=false);
};
render_canvas_parms::render_canvas_parms() = default;
render_canvas_parms::~render_canvas_parms() = default;
#include "fishing-loader.inc"
#include "fishing-cell.inc"
class FixtureApp: public wxApp { public: bool OnInit() override { return true; } };
wxIMPLEMENT_APP_NO_MAIN(FixtureApp);
static int checks=0;
static void Check(bool b) { ++checks; if(!b) throw std::runtime_error("Fishing cell check "+std::to_string(checks)); }
static void Release(render_canvas_parms* p) { free(p->pix_buff);delete p; }

int main(int argc,char** argv) {
  if(argc!=3 || !wxEntryStart(argc,argv)) return 2;
  wxTheApp->CallOnInit();wxInitAllImageHandlers();
  try {
    wxArrayPtrVoid allocations;std::map<wxString,Rule*> symbols,patterns;
    s52plib s;s.pAlloc=&allocations;s._symb_sym=&symbols;s._patt_sym=&patterns;
    auto& loader=s.m_chartSymbols;loader.InitializeTables();loader.plib=&s;
    loader.configFileDirectory=wxString(argv[1]);
    pugi::xml_document doc;Check(doc.load_file((loader.configFileDirectory+"/chartsymbols.xml").fn_str()));
    auto colors=doc.child("chartsymbols").child("color-tables");loader.ProcessColorTables(colors);
    auto pats=doc.child("chartsymbols").child("patterns");loader.ProcessPatterns(pats);
    auto syms=doc.child("chartsymbols").child("symbols");loader.ProcessSymbols(syms);
    Rule* alias=patterns.at("XNFISH03");Rule* stock=patterns.at("FSHFAC03");
    Check(stock->RCID==2001 && symbols.at("FSHFAC03")->RCID==2067);
    Check(opennav::integration::FishingPatternEligible(true,alias));
    Check(!opennav::integration::FishingPatternEligible(false,alias));
    Check(!opennav::integration::FishingPatternEligible(true,stock));
    Check(!opennav::integration::FishingPatternEligible(true,symbols.at("FSHFAC03")));
    wxRect rect;loader.GetGLTextureRect(rect,"XNFISH03");Check(rect==wxRect(948,1160,24,24));
    loader.GetGLTextureRect(rect,"FSHFAC03");Check(rect!=wxRect(948,1160,24,24));
    Rules selected{};selected.razRule=alias;Rules original{};original.razRule=stock;
    const char* themes[]={"DAY_BRIGHT","DUSK","NIGHT"};
    for(auto theme:themes) {
      s.table=loader.FindColorTable(theme);Check(loader.LoadRasterFileForColorTable(s.table,true,ChartCtx(false,0)));
      wxImage glyph=loader.GetImage("XNFISH03");Check(glyph.IsOk());
      for(double ppmm:{3.0,96.0/25.4,120.0/25.4,144.0/25.4}) {
        s.canvas_pix_per_mm=ppmm;opennav::integration::FishingPatternPlacement placement;wxImage image;
        const bool composed=opennav::integration::ComposeFishingPattern(ppmm,glyph,image,&placement);
        if (!composed) { std::cerr<<"declined ppmm="<<ppmm<<" required inset="<<placement.inset_x<<","<<placement.inset_y<<"\n"; }
        Check(composed);
        std::cout<<theme<<" ppmm="<<ppmm<<" cell="<<placement.width<<"x"<<placement.height
          <<" old_center="<<placement.old_center_x<<","<<placement.old_center_y
          <<" new_center="<<placement.x+placement.side/2.0<<","<<placement.y+placement.side/2.0
          <<" inset="<<placement.inset_x<<","<<placement.inset_y;
        int left=image.GetWidth(),top=image.GetHeight(),right=-1,bottom=-1;
        for(int y=0;y<image.GetHeight();++y) for(int x=0;x<image.GetWidth();++x) if(image.GetAlpha(x,y)) {
          if(x<left)left=x;
          if(y<top)top=y;
          if(x>right)right=x;
          if(y>bottom)bottom=y;
        }
        std::cout<<" occupied="<<left<<","<<top<<"-"<<right<<","<<bottom<<"\n";
        for(bool pot:{false,true}) for(bool rgb:{false,true}) {
          int calls=s.hpgl.calls;
          auto actual=s.CreatePatternBufferSpec(nullptr,&selected,rgb,pot);
          Check(s.hpgl.calls==calls);
          auto prior=s.CreatePatternBufferSpec(nullptr,&original,rgb,pot);
          Check(s.hpgl.calls==calls+1);
          Check(actual->width==prior->width && actual->height==prior->height && actual->b_stagger==prior->b_stagger);
          Check(actual->w_pot==prior->w_pot && actual->h_pot==prior->h_pot && actual->x==prior->x && actual->y==prior->y);
          int nonzero=0;
          for(int y=0;y<actual->h_pot;++y) for(int x=0;x<actual->w_pot;++x) {
            auto pixel=actual->pix_buff+y*actual->pb_pitch+x*4;
            unsigned a=(x<image.GetWidth() && y<image.GetHeight()) ? image.GetAlpha(x,y):0;
            Check(pixel[3]==a);
            if(a) { ++nonzero;Check(pixel[0]==image.GetRed(x,y) && pixel[1]==image.GetGreen(x,y) && pixel[2]==image.GetBlue(x,y)); }
          }
          Check(nonzero>0);Release(actual);Release(prior);
        }
        if(ppmm==96.0/25.4) Check(image.SaveFile(wxString(argv[2])+"/"+theme+"-cell.png",wxBITMAP_TYPE_PNG));
      }
      wxImage image;
      opennav::integration::FishingPatternPlacement maximum;
      Check(!opennav::integration::ComposeFishingPattern(24, glyph, image, &maximum));
      Check(maximum.width==842 && maximum.height==518 && maximum.side==152);
      Check(maximum.inset_y>std::ceil(24/(96.0/25.4)) && !image.IsOk());
      s.canvas_pix_per_mm=24;int maximum_calls=s.hpgl.calls;
      auto maximum_fallback=s.CreatePatternBufferSpec(nullptr,&selected,false);
      Check(s.hpgl.calls==maximum_calls+1);Release(maximum_fallback);
      for(double ppmm:{0.0,0.5,24.001,1000.0,std::numeric_limits<double>::quiet_NaN()})
        Check(!opennav::integration::ComposeFishingPattern(ppmm,glyph,image));
      Check(!opennav::integration::ComposeFishingPattern(3,wxImage(),image));
      s.m_presentationLightSymbols=false;int calls=s.hpgl.calls;
      auto fallback=s.CreatePatternBufferSpec(nullptr,&selected,false);Check(s.hpgl.calls==calls+1);Release(fallback);
      s.m_presentationLightSymbols=true;
    }
    for(int which=0;which<7;++which) {
      Rule bad=*alias;
      if(which==0)bad.RCID++;
      if(which==1)bad.fillType.PATP='L';
      if(which==2)bad.pos.patt.minDist.PAMI++;
      if(which==3)bad.pos.patt.pivot_x.PACL++;
      if(which==4)bad.pos.patt.bnbox_h.PAVL++;
      if(which==5)bad.definition.PADF='R';
      if(which==6)bad.spacing.PASP='L';
      Check(!opennav::integration::FishingPatternEligible(true,&bad));
    }
    std::cout<<checks<<" actual loader/cell/alpha/fallback checks passed; no polygon or GL draw\n";
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
  wxTheApp->OnExit();wxEntryCleanup();return 0;
}

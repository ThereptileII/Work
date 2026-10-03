// The runner includes unchanged pinned ChartSymbols methods, not a replica of
// their selection/cropping logic. Only the owning S52 containers are fixtures;
// no full navigation engine, chart canvas or GL context is constructed.
#include <wx/wx.h>
#include <wx/filename.h>
#include <fstream>
#include <iostream>
#include <map>
#include <cmath>
#include <stdexcept>
#include <deque>
#include <limits>
#include "integration/ChartLightSymbol.h"
#include "integration/ChartSpecialBuoySymbol.h"
#include "integration/ChartYellowBuoySymbol.h"
#define private public
#include "chartsymbols.h"
#undef private
WX_DEFINE_SORTED_ARRAY(LUPrec*, wxArrayOfLUPrec);
class s52plib {
 public:
  wxArrayPtrVoid *pAlloc;
  std::map<wxString, Rule *> *_symb_sym;
  bool m_presentationLightSymbols = false;
  wxString m_ColorScheme = "DAY";
  double m_ChartScaleFactorExp = 1.0;
  Rule* painted = nullptr;
  char painter = 0;
  float paintedAngle = 0;
  wxPoint paintedPoint{};
#include "light-enable-method.inc"
  int RenderSY(ObjRazRules*, Rules*);
  int DoRenderObject(wxDC*,ObjRazRules*);
  LUPrec* FindBestLUP(wxArrayOfLUPrec*, unsigned, unsigned, S57Obj*, bool);
  void RenderPresentationYellowTopmark(ObjRazRules*);
  wxDC* m_pdc=nullptr;
  bool visible=true;
  int visibilityChecks=0;
  bool ObjectRenderCheckRules(ObjRazRules*,bool) { ++visibilityChecks; return visible; }
  void RenderTX(ObjRazRules*,Rules*) {}
  void RenderTE(ObjRazRules*,Rules*) {}
  void RenderLS(ObjRazRules*,Rules*) {}
  void RenderGLLS(ObjRazRules*,Rules*) {}
  void RenderLC(ObjRazRules*,Rules*) {}
  void RenderMPS(ObjRazRules*,Rules*) {}
  void RenderCARC(ObjRazRules*,Rules*) {}
  void GetAndAddCSRules(ObjRazRules*,Rules*) {}
  void GetPointPixSingle(ObjRazRules*,double,double,wxPoint* point) {
    *point=wxPoint(101,202); // Geometry projection is outside this fixture.
  }
  void RenderHPGL(ObjRazRules*,Rule* rule,wxPoint point,float angle,double) {
    painted=rule;painter='V';paintedAngle=angle;paintedPoint=point;
  }
  void RenderRasterSymbol(ObjRazRules*,Rule* rule,wxPoint point,float angle) {
    painted=rule;painter='R';paintedAngle=angle;paintedPoint=point;
  }
};
#include "anchor-loader-methods.inc"
#include "light-selector-method.inc"
// Only fixture object construction and text callback are supplied locally.
// Actual attribute lookup, numeric/string access and LIGHTS06 are executed.
S57Obj::S57Obj() : att_array(nullptr), attVal(nullptr), n_attr(0), x(0), y(0) {
  std::memcpy(FeatureName,"LIGHTS",7);
}
S57Obj::~S57Obj() {}
static wxString _LITDSN01(S57Obj*) { return wxString(); }
#define LISTSIZE 20
#include "light-render-methods.inc"

#include "plugin-adapters/ocharts/OwnedPresentationValidation.h"

class FixtureApp : public wxApp {
 public:
  bool OnInit() override { return true; }
};
wxIMPLEMENT_APP_NO_MAIN(FixtureApp);
static int checks = 0;
static void Check(bool value) {
  ++checks;
  if (!value) throw std::runtime_error("Seamark loader check " + std::to_string(checks));
}

struct LightFixture : S57Obj {
  wxArrayOfS57attVal values;
  std::deque<S57attVal> entries;
  std::map<std::string,double> numbers;
  std::map<std::string,std::string> strings;
  std::string names;
  LightFixture() {attVal=&values;}
  void Add(const char* name,void* value,OGRatt_t type) {
    names+=name;att_array=names.data();++n_attr;
    entries.push_back({value,type});values.Add(&entries.back());
  }
  void Number(const char* name,double value) {
    numbers[name]=value;Add(name,&numbers[name],OGR_REAL);
  }
  void String(const char* name,const char* value) {
    strings[name]=value;Add(name,strings[name].data(),OGR_STR);
  }
};

struct PillarFixture : LightFixture {
  int shape=4;
  PillarFixture() {
    std::memcpy(FeatureName,"BOYSPP",7); Primitive_type=GEO_POINT;
    Add("BOYSHP",&shape,OGR_INT);
    String("COLOUR","1,11"); String("COLPAT","1"); String("CATSPM","27");
  }
};

static void CheckPillarDispatch(s52plib& owner,std::map<wxString,Rule*>& symbols) {
  auto stock=symbols.at("BOYSPP11"); Rule saved=*stock;
  auto alias=symbols.at("XNSPPW01");
  Rules rules{}; rules.razRule=stock; char instruction[]="BOYSPP11)"; rules.INSTstr=instruction;
  LUPrec lookup{}; lookup.TNAM=SIMPLIFIED;
  auto draw=[&](S57Obj& object) { ObjRazRules rz{}; rz.obj=&object; rz.LUP=&lookup; owner.RenderSY(&rz,&rules); };
  PillarFixture valid; owner.m_presentationLightSymbols=false; draw(valid); Check(owner.painted==stock);
  owner.EnablePresentationLightSymbols(); draw(valid);
  Check(owner.painted==alias && owner.painter=='R' && owner.paintedPoint==wxPoint(101,202));
  Check(owner.paintedAngle==0 && rules.razRule==stock && !std::memcmp(stock,&saved,sizeof saved));
  for(const char* scheme:{"DAY","DUSK","NIGHT","DAY","DAY_BRIGHT","UNKNOWN","","dusk"}) {
    owner.m_ColorScheme=scheme; draw(valid);
    const bool eligible=owner.m_ColorScheme=="DAY" || owner.m_ColorScheme=="DAY_BRIGHT" || owner.m_ColorScheme=="NIGHT";
    Check(owner.painted==(eligible?alias:stock) && rules.razRule==stock);
    Check(!std::memcmp(stock,&saved,sizeof saved));
  }
  owner.m_ColorScheme="DAY";
  lookup.TNAM=PAPER_CHART; draw(valid); Check(owner.painted==stock); lookup.TNAM=SIMPLIFIED;
  for(int shape:{1,2,3,5,6,7,8,0,-1}) { PillarFixture x; x.shape=shape; draw(x); Check(owner.painted==stock); }
  for(const char* color:{"6","1","11","11,1","1,11,6","1, 11","01,11","1,11 ",""}) {
    PillarFixture x; x.entries[1].value=const_cast<char*>(color); draw(x); Check(owner.painted==stock);
  }
  for(const char* pattern:{"2","1,2","01",""}) { PillarFixture x; x.entries[2].value=const_cast<char*>(pattern); draw(x); Check(owner.painted==stock); }
  for(const char* category:{"14","27,1","027",""}) { PillarFixture x; x.entries[3].value=const_cast<char*>(category); draw(x); Check(owner.painted==stock); }
  for(int index=0;index<4;++index) {
    PillarFixture x; x.entries[index].value=nullptr; draw(x); Check(owner.painted==stock);
    PillarFixture wrong; wrong.entries[index].valType=OGR_REAL; draw(wrong); Check(owner.painted==stock);
    PillarFixture list; list.entries[index].valType=OGR_INT_LST; draw(list); Check(owner.painted==stock);
    PillarFixture duplicate; const char* names[]={"BOYSHP","COLOUR","COLPAT","CATSPM"};
    duplicate.Add(names[index],duplicate.entries[index].value,duplicate.entries[index].valType); draw(duplicate); Check(owner.painted==stock);
    PillarFixture missing; std::memcpy(missing.att_array+index*6,"UNKNWN",6); draw(missing); Check(owner.painted==stock);
  }
  for(const char* name:{"ORIENT","TOPSHP"}) { PillarFixture x; x.Number(name,0); draw(x); Check(owner.painted==stock); }
  PillarFixture other; std::memcpy(other.FeatureName,"BOYLAT",7); draw(other); Check(owner.painted==stock);
  PillarFixture area; area.Primitive_type=GEO_AREA; draw(area); Check(owner.painted==stock);
  PillarFixture count; ++count.n_attr; Check(!opennav::integration::PresentationSpecialBuoyAlias(true,true,&count,"BOYSPP11"));
  PillarFixture duplicateUnknown; duplicateUnknown.String("OBJNAM","Actual warning buoy"); draw(duplicateUnknown); Check(owner.painted==alias);
  const auto size=symbols.size(); symbols.erase("XNSPPW01"); draw(valid); Check(owner.painted==stock && symbols.size()==size-1);
  symbols["XNSPPW01"]=nullptr; draw(valid); Check(owner.painted==stock);
  symbols["XNSPPW01"]=alias; auto kind=alias->definition.SYDF; alias->definition.SYDF='V'; draw(valid); Check(owner.painted==stock); alias->definition.SYDF=kind;
  auto name=alias->name.SYNM[0];alias->name.SYNM[0]='?';draw(valid);Check(owner.painted==stock);alias->name.SYNM[0]=name;
  Check(!opennav::integration::PresentationSpecialBuoyAlias(true,true,nullptr,"BOYSPP11"));
  Check(!opennav::integration::PresentationSpecialBuoyAlias(true,true,&valid,"BOYSPP25"));
  Check(!std::memcmp(stock,&saved,sizeof saved) && rules.razRule==stock);
}

#include "chart_yellow_buoy_cases.inc"

static void CheckLightDispatch(s52plib& owner,std::map<wxString,Rule*>& symbols) {
  for(int i=11;i<=13;++i) {
    const auto originalName="LIGHTS"+std::to_string(i);
    const auto alias="XNLIT0"+std::to_string(i);
    Rule* original=symbols.at(originalName);Rule saved=*original;
    Check(original->definition.SYDF=='V' && original->pos.symb.pivot_x.SYCL==2250 &&
          original->pos.symb.pivot_y.SYRW==2302 && original->pos.symb.bnbox_h.SYVL==810 &&
          original->pos.symb.bnbox_w.SYHL==285 && std::strlen(original->vector.SVCT)>100);
    char instruction[24];std::snprintf(instruction,sizeof instruction,"LIGHTS%d,135)",i);
    Rules rules{};rules.razRule=original;rules.INSTstr=instruction;
    LightFixture light;ObjRazRules rz{};rz.obj=&light;
    owner.m_presentationLightSymbols=false;owner.RenderSY(&rz,&rules);
    Check(owner.painted==original&&owner.painter=='V'&&owner.paintedAngle==135);
    owner.EnablePresentationLightSymbols();owner.RenderSY(&rz,&rules);
    Check(owner.painted==symbols.at(alias)&&owner.painter=='R'&&owner.paintedPoint==wxPoint(101,202));
    // Real values keep the exact existing vector and angle, including non-finite.
    for(double bearing:{0.,45.,270.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
      LightFixture oriented;oriented.Number("ORIENT",bearing);rz.obj=&oriented;
      owner.RenderSY(&rz,&rules);
      const double expected=bearing+180>360?bearing-180:bearing+180;
      Check(owner.painted==original&&owner.painter=='V');
      Check(std::isnan(expected)?std::isnan(owner.paintedAngle):owner.paintedAngle==expected);
    }
    rz.obj=&light;
    // Presence, not conversion validity, determines the owned-art guard.
    Check(!opennav::integration::PresentationLightAlias(true,"LIGHTS",originalName.c_str(),"COLOURORIENT",2));
    Check(!opennav::integration::PresentationLightAlias(true,"LIGHTS",originalName.c_str(),nullptr,1));
    Check(!opennav::integration::PresentationLightAlias(true,"LIGHTS",originalName.c_str(),nullptr,4097));
    auto owned=symbols.at(alias);const auto size=symbols.size();symbols.erase(alias);
    owner.RenderSY(&rz,&rules);Check(owner.painted==original&&symbols.size()==size-1);
    symbols[alias]=nullptr;owner.RenderSY(&rz,&rules);Check(owner.painted==original);
    symbols[alias]=owned;const auto kind=owned->definition.SYDF;owned->definition.SYDF='V';
    owner.RenderSY(&rz,&rules);Check(owner.painted==original);owned->definition.SYDF=kind;
    const auto letter=owned->name.SYNM[0];owned->name.SYNM[0]='?';
    owner.RenderSY(&rz,&rules);Check(owner.painted==original);owned->name.SYNM[0]=letter;
    std::memcpy(light.FeatureName,"BOYLAT",7);owner.RenderSY(&rz,&rules);Check(owner.painted==original);
    Check(!std::memcmp(original,&saved,sizeof saved)&&rules.razRule==original&&symbols.size()==size);
  }
  // Execute unchanged LIGHTS06 dispatch using real typed attributes/accessors.
  for(const auto& item:std::map<std::string,std::string>{{"3","11"},{"4","12"},{"1","13"},{"1,3","11"},{"1,4","12"}}) {
    LightFixture light;light.String("COLOUR",item.first.c_str());ObjRazRules rz{};rz.obj=&light;
    auto text=static_cast<char*>(LIGHTS06(&rz));Check(std::string(text).find(";SY(LIGHTS"+item.second+",135)")==0);free(text);
    light.Number("VALNMR",10.);text=static_cast<char*>(LIGHTS06(&rz));Check(std::string(text).find(";CA(")==0);free(text);
  }
  for(const char* category:{"1","14","8","11","9"}) {
    LightFixture light;light.String("COLOUR","3");light.String("CATLIT",category);light.Number("ORIENT",72.);
    ObjRazRules rz{};rz.obj=&light;auto text=static_cast<char*>(LIGHTS06(&rz));
    Check(std::string(text).find((std::string(category)=="1"||std::string(category)=="14")?";SY(QUESMRK1)":std::string(category)=="9"?";SY(LIGHTS81)":";SY(LIGHTS82)")==0);free(text);
  }
  LightFixture sector;sector.String("COLOUR","4");sector.Number("SECTR1",45.);sector.Number("SECTR2",90.);
  ObjRazRules rz{};rz.obj=&sector;auto text=static_cast<char*>(LIGHTS06(&rz));Check(std::string(text).find(";CA(OUTLW, 4,LITGN, 2")==0);free(text);
}

int main(int argc, char **argv) {
  if (argc != 4) return 2;
  const wxString folder(argv[1]), svgFile(argv[2]), out(argv[3]);
  if (!wxEntryStart(argc, argv)) return 3;
  wxTheApp->CallOnInit();
  wxInitAllImageHandlers();
  try {
    // Execute the actual upstream conditional color selector. Sector/all-round
    // CA paths and other color classes must never be mapped to the new symbol.
    for (const int color : {1,6,9}) {
      char attr[] = {static_cast<char>(color),0,0};
      Check(_selSYcol(attr,false,9) == ";SY(LIGHTS13");
      Check(_selSYcol(attr,true,9) == ";CA(OUTLW, 4,LITYW, 2,0,360,12,0");
    }
    char red[] = {3,0,0}, green[] = {4,0,0}, unknown[] = {12,0,0};
    char redWhite[] = {1,3,0}, greenWhite[] = {1,4,0}, other[] = {1,6,0};
    Check(_selSYcol(red,false,9) == ";SY(LIGHTS11");
    Check(_selSYcol(green,false,9) == ";SY(LIGHTS12");
    Check(_selSYcol(unknown,false,9) == ";SY(LITDEF11");
    Check(_selSYcol(redWhite,false,9) == ";SY(LIGHTS11");
    Check(_selSYcol(greenWhite,false,9) == ";SY(LIGHTS12");
    Check(_selSYcol(other,false,9) == ";SY(LITDEF11");
    Check(_selSYcol(red,true,5) == ";CA(OUTLW, 4,LITRD, 2,0,360,4,0");
    Check(_selSYcol(green,true,20) == ";CA(OUTLW, 4,LITGN, 2,0,360,15,0");
    Check(_selSYcol(unknown,true,35) == ";CA(OUTLW, 4,CHMGD, 2,0,360,23,0");
    ChartSymbols loader;
    loader.InitializeTables();
    wxArrayPtrVoid allocations;
    std::map<wxString, Rule *> symbols;
    s52plib owner{&allocations, &symbols};
    loader.plib = &owner;
    loader.configFileDirectory = folder;
    pugi::xml_document doc;
    Check(doc.load_file((folder+"/chartsymbols.xml").fn_str()));
    Check(skager::ocharts::ValidateOwnedPresentation(doc,folder));
    auto tables = doc.child("chartsymbols").child("color-tables");
    loader.ProcessColorTables(tables);
    auto definitions = doc.child("chartsymbols").child("symbols");
    loader.ProcessSymbols(definitions);
    const char *names[] = {"XNLAT013","XNLAT014","XNLAT023","XNLAT024","XNCAN072","XNCAN073","XNCON066","XNCON067","BOYISD12","BOYSAW12","XNLIT011","XNLIT012","XNLIT013","XNSPPW01","XNSPPY01","XNBCNG01","XNSPPT01"};
    const int rcids[] = {60001,60002,60003,60004,60005,60006,60007,60008,2049,1294,60009,60010,60011,60012,60013,60014,60015};
    const int atlasX[] = {244,276,308,340,372,404,436,468,500,532,564,596,628,660,692,724,756};
    const char *themes[] = {"DAY_BRIGHT", "DUSK", "NIGHT"};
    wxRect rect;
    loader.GetGLTextureRect(rect,"ACHARE51");
    Check(rect == wxRect(20,1160,20,20));
    for (int n=0; n<int(sizeof(names)/sizeof(names[0])); ++n) {
      auto rule = symbols.at(names[n]);
      Check(rule->RCID == rcids[n] && rule->definition.SYDF == 'R');
      Check(rule->pos.symb.pivot_x.SYCL == 12 && rule->pos.symb.pivot_y.SYRW == 14);
      Check(rule->pos.symb.bnbox_w.SYHL == 24 && rule->pos.symb.bnbox_h.SYVL == 28);
      loader.GetGLTextureRect(rect,names[n]);
      Check(rect == wxRect(atlasX[n],1160,24,28));
      for (int theme=0; theme<3; ++theme) {
        wxImage svg(svgFile+"/"+names[n]+"-"+themes[theme]+".png",wxBITMAP_TYPE_PNG);
        Check(svg.IsOk() && svg.HasAlpha() && svg.GetWidth()==24 && svg.GetHeight()==28);
        const int table=loader.FindColorTable(themes[theme]);
        Check(loader.LoadRasterFileForColorTable(table,true,ChartCtx(false,0)));
        Check(loader.rasterSymbols.GetWidth()==1500 && loader.rasterSymbols.GetHeight()==1200);
        auto tile=loader.GetImage(names[n]);
        Check(tile.IsOk() && tile.GetWidth()==24 && tile.GetHeight()==28 && tile.HasAlpha());
        int pixels=0;
        for (int y=0;y<28;++y) for (int x=0;x<24;++x) {
          const int a=tile.GetAlpha(x,y);
          Check(a==svg.GetAlpha(x,y));
          if (!a) continue;
          ++pixels;
          const int reference[]={svg.GetRed(x,y),svg.GetGreen(x,y),svg.GetBlue(x,y)};
          const int rgb[]={tile.GetRed(x,y),tile.GetGreen(x,y),tile.GetBlue(x,y)};
          for (int ch=0;ch<3;++ch)
            Check(std::abs(rgb[ch]*a/255.0-reference[ch]*a/255.0)<=1.01);
        }
        Check(pixels >= (!std::strcmp(names[n], "XNSPPT01") ? 30 : 40));
        Check(tile.SaveFile(out+"/"+names[n]+"-"+themes[theme]+".png",wxBITMAP_TYPE_PNG));
      }
    }
    // Separate9x9 generic-building check; the17 prior24x28 cases remain exact.
    auto building = symbols.at("XNBLDG01");
    Check(building->RCID == 60016 && building->definition.SYDF == 'R');
    Check(building->pos.symb.pivot_x.SYCL == 4 && building->pos.symb.pivot_y.SYRW == 4);
    Check(building->pos.symb.bnbox_w.SYHL == 9 && building->pos.symb.bnbox_h.SYVL == 9);
    Check(!std::strcmp(building->colRef.SCRF, "WXNBLOKXNBLF"));
    loader.GetGLTextureRect(rect,"XNBLDG01");
    Check(rect == wxRect(788,1160,9,9));
    const int buildingFill[3][3] = {{124,133,138},{168,187,183},{98,115,108}};
    for (int theme=0;theme<3;++theme) {
      Check(loader.LoadRasterFileForColorTable(loader.FindColorTable(themes[theme]),true,ChartCtx(false,0)));
      auto original = loader.GetImage("BUISGL01");
      auto tile = loader.GetImage("XNBLDG01");
      Check(tile.IsOk() && tile.HasAlpha() && tile.GetWidth()==9 && tile.GetHeight()==9);
      for (int y=0;y<9;++y) for (int x=0;x<9;++x)
        Check(tile.GetAlpha(x,y)==original.GetAlpha(x,y));
      Check(tile.GetRed(4,4)==buildingFill[theme][0] &&
            tile.GetGreen(4,4)==buildingFill[theme][1] &&
            tile.GetBlue(4,4)==buildingFill[theme][2]);
      Check(tile.SaveFile(out+"/XNBLDG01-"+themes[theme]+".png",wxBITMAP_TYPE_PNG));
    }
    CheckLightDispatch(owner,symbols);
    CheckPillarDispatch(owner,symbols); CheckYellowDispatch(owner,symbols); CheckYellowLookups(owner,doc.child("chartsymbols").child("lookups"));
    for (auto &entry:symbols) {
      free(entry.second->colRef.SCRF);
      free(entry.second->vector.SVCT);
      delete entry.second->exposition.SXPO;
    }
    for (auto entry:allocations) free(entry);
    loader.DeleteGlobals();
    std::cout << checks << " actual pinned loader checks passed; GL context and native Windows acceptance remain open\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    wxTheApp->OnExit(); wxEntryCleanup(); return 1;
  }
  wxTheApp->OnExit(); wxEntryCleanup(); return 0;
}

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
#define private public
#include "chartsymbols.h"
#undef private
class s52plib {
 public:
  wxArrayPtrVoid *pAlloc;
  std::map<wxString, Rule *> *_symb_sym;
  bool m_presentationLightSymbols = false;
  double m_ChartScaleFactorExp = 1.0;
  Rule* painted = nullptr;
  char painter = 0;
  float paintedAngle = 0;
  wxPoint paintedPoint{};
#include "light-enable-method.inc"
  int RenderSY(ObjRazRules*, Rules*);
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
    const char *names[] = {"XNLAT013","XNLAT014","XNLAT023","XNLAT024","XNCAN072","XNCAN073","XNCON066","XNCON067","BOYISD12","BOYSAW12","XNLIT011","XNLIT012","XNLIT013"};
    const int rcids[] = {60001,60002,60003,60004,60005,60006,60007,60008,2049,1294,60009,60010,60011};
    const char *themes[] = {"DAY_BRIGHT", "DUSK", "NIGHT"};
    wxRect rect;
    loader.GetGLTextureRect(rect,"ACHARE51");
    Check(rect == wxRect(20,1160,20,20));
    for (int n=0; n<13; ++n) {
      auto rule = symbols.at(names[n]);
      Check(rule->RCID == rcids[n] && rule->definition.SYDF == 'R');
      Check(rule->pos.symb.pivot_x.SYCL == 12 && rule->pos.symb.pivot_y.SYRW == 14);
      Check(rule->pos.symb.bnbox_w.SYHL == 24 && rule->pos.symb.bnbox_h.SYVL == 28);
      loader.GetGLTextureRect(rect,names[n]);
      Check(rect == wxRect(244+32*n,1160,24,28));
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
        Check(pixels>=40);
        Check(tile.SaveFile(out+"/"+names[n]+"-"+themes[theme]+".png",wxBITMAP_TYPE_PNG));
      }
    }
    CheckLightDispatch(owner,symbols);
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

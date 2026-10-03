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
#define private public
#include "chartsymbols.h"
#undef private
class s52plib {
 public:
  wxArrayPtrVoid *pAlloc;
  std::map<wxString, Rule *> *_symb_sym;
};
#include "anchor-loader-methods.inc"
#include "light-selector-method.inc"
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
    const char *names[] = {"XNLAT013","XNLAT014","XNLAT023","XNLAT024","XNCAN072","XNCAN073","XNCON066","XNCON067","BOYISD12","BOYSAW12","LIGHTS13"};
    const int rcids[] = {60001,60002,60003,60004,60005,60006,60007,60008,2049,1294,1398};
    const char *themes[] = {"DAY_BRIGHT", "DUSK", "NIGHT"};
    wxRect rect;
    loader.GetGLTextureRect(rect,"ACHARE51");
    Check(rect == wxRect(20,1160,20,20));
    for (int n=0; n<11; ++n) {
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

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

class FixtureApp : public wxApp {
 public:
  bool OnInit() override { return true; }
};
wxIMPLEMENT_APP_NO_MAIN(FixtureApp);
static int checks = 0;
static void Check(bool value) {
  ++checks;
  if (!value) throw std::runtime_error("Anchor loader check " + std::to_string(checks));
}

int main(int argc, char **argv) {
  if (argc != 4) return 2;
  const wxString folder(argv[1]), svgFile(argv[2]), out(argv[3]);
  if (!wxEntryStart(argc, argv)) return 3;
  wxTheApp->CallOnInit();
  wxInitAllImageHandlers();
  try {
    ChartSymbols loader;
    loader.InitializeTables();
    wxArrayPtrVoid allocations;
    std::map<wxString, Rule *> symbols;
    s52plib owner{&allocations, &symbols};
    loader.plib = &owner;
    loader.configFileDirectory = folder;
    pugi::xml_document doc;
    Check(doc.load_file((folder+"/chartsymbols.xml").fn_str()));
    auto tables = doc.child("chartsymbols").child("color-tables");
    loader.ProcessColorTables(tables);
    auto definitions = doc.child("chartsymbols").child("symbols");
    loader.ProcessSymbols(definitions);
    auto rule = symbols.at("ACHARE51");
    Check(rule->RCID == 1105 && rule->definition.SYDF == 'R');
    Check(rule->pos.symb.pivot_x.SYCL == 10 && rule->pos.symb.pivot_y.SYRW == 10);
    Check(rule->pos.symb.bnbox_w.SYHL == 20 && rule->pos.symb.bnbox_h.SYVL == 20);
    wxRect rect;
    loader.GetGLTextureRect(rect, "ACHARE51");
    Check(rect == wxRect(20,1160,20,20));
    // The adjacent anchoring-point symbol retains its own stock tile.
    loader.GetGLTextureRect(rect, "ACHPNT02");
    Check(rect == wxRect(408,415,25,29));
    wxImage svg(svgFile, wxBITMAP_TYPE_PNG);
    Check(svg.IsOk() && svg.HasAlpha() && svg.GetWidth()==20 && svg.GetHeight()==20);
    const char *themes[] = {"DAY_BRIGHT", "DUSK", "NIGHT"};
    const int ink[3][3] = {{124,133,138},{168,187,183},{98,115,108}};
    for (int theme=0; theme<3; ++theme) {
      const int table = loader.FindColorTable(themes[theme]);
      Check(loader.LoadRasterFileForColorTable(table,true,ChartCtx(false,0)));
      Check(loader.rasterSymbols.GetWidth()==1500 && loader.rasterSymbols.GetHeight()==1200);
      auto tile=loader.GetImage("ACHARE51");
      Check(tile.IsOk() && tile.GetWidth()==20 && tile.GetHeight()==20 && tile.HasAlpha());
      int pixels=0;
      for (int y=0; y<20; ++y) for (int x=0; x<20; ++x) {
        const int a=tile.GetAlpha(x,y);
        Check(a==svg.GetAlpha(x,y));
        if (!a) continue;
        ++pixels;
        const int rgb[]={tile.GetRed(x,y),tile.GetGreen(x,y),tile.GetBlue(x,y)};
        for (int ch=0;ch<3;++ch)
          Check(std::abs(rgb[ch]*a/255.0-ink[theme][ch]*a/255.0)<=1.01);
      }
      Check(pixels==126);
      Check(tile.SaveFile(out+"/"+themes[theme]+".png",wxBITMAP_TYPE_PNG));
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

#include "integration/ChartCanvasInk.h"
#include "integration/ChartPresentationPolicy.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
int main() {
  using namespace opennav;unsigned checks=0;
  auto check=[&](bool value){++checks;if(!value)throw std::runtime_error("Chart paint policy check failed");};
  check(integration::ChartCanvasInk(ui::LightMode::Night,0x91bca2)==0x71937e);
  check(integration::ChartCanvasInk(ui::LightMode::Night,0x152129)==0x101a20);
  for(unsigned r=0;r<256;++r) {
    const unsigned g=255-r,b=(r*17)%256,rgb=(r<<16)|(g<<8)|b;
    check(integration::ChartCanvasInk(ui::LightMode::Day,rgb)==rgb);
    check(integration::ChartCanvasInk(ui::LightMode::Dusk,rgb)==rgb);
    const auto expected=(unsigned(std::lround(r*.78))<<16)|
        (unsigned(std::lround(g*.78))<<8)|unsigned(std::lround(b*.78));
    check(integration::ChartCanvasInk(ui::LightMode::Night,rgb)==expected);
  }
  // Floating controls retain raw theme roles. They are outside chart-canvas.
  check(ui::ActiveRouteInk(ui::LightMode::Night)==0x91bca2);
  check(ui::FloatingTheme(ui::LightMode::Night).surface==0x152129);

  // Only the native depth-area AC roles become uniform. Drying, hazard fills,
  // patterns and safety contours retain their meaning even in Very Light.
  using integration::VeryLightAreaColor;
  using integration::VeryLightOrdinaryContour;
  for (const char* feature : {"DEPARE", "DRGARE"}) {
    for (const char* color : {"DEPDW", "DEPMD", "DEPMS", "DEPVS"}) {
      check(std::strcmp(VeryLightAreaColor(true,feature,color),"DEPDW")==0);
      check(VeryLightAreaColor(false,feature,color)==color);
    }
    for (const char* preserved : {"DEPIT", "DEPSC", "DRGARE01", "CHBLK"})
      check(VeryLightAreaColor(true,feature,preserved)==preserved);
  }
  for (const char* hazard : {"UWTROC", "WRECKS", "OBSTRN"}) {
    const char* shallow="DEPVS";
    check(VeryLightAreaColor(true,hazard,shallow)==shallow);
    check(!VeryLightOrdinaryContour(true,hazard,"DEPCN"));
  }
  check(VeryLightOrdinaryContour(true,"DEPCNT","DEPCN"));
  check(VeryLightOrdinaryContour(true,"DEPCNT","DEPCN)"));
  check(!VeryLightOrdinaryContour(false,"DEPCNT","DEPCN"));
  check(!VeryLightOrdinaryContour(true,"DEPCNT","DEPSC"));
  check(VeryLightOrdinaryContour(true,"DEPARE","DEPCN",true));
  check(!VeryLightOrdinaryContour(true,"DEPARE","DEPCN",false));
  check(!VeryLightOrdinaryContour(true,"DEPARE","DEPSC",true));
  check(!VeryLightOrdinaryContour(false,"DEPARE","DEPCN",true));
  for (const char* feature : {"DEPARE", "DRGARE", "COALNE", "LNDARE", "LIGHTS"})
    check(!VeryLightOrdinaryContour(true,feature,"DEPCN"));

  // Native feature/color fields can be fixed-width without terminators. Short
  // or empty diagnostic labels must also fail closed without over-reading.
  const char area[6]={'D','E','P','A','R','E'};
  const char contour[6]={'D','E','P','C','N','T'};
  const char depth[5]={'D','E','P','V','S'};
  const char ordinary[5]={'D','E','P','C','N'};
  check(std::strcmp(VeryLightAreaColor(true,area,depth),"DEPDW")==0);
  check(VeryLightOrdinaryContour(true,contour,ordinary));
  for (const char* feature : {static_cast<const char*>(nullptr), "", "D", "DEP", "DEPCN"}) {
    const char* color="DEPVS";
    check(VeryLightAreaColor(true,feature,color)==color);
    check(!VeryLightOrdinaryContour(true,feature,"DEPCN"));
  }
  for (const char* color : {static_cast<const char*>(nullptr), "", "D", "DEP"}) {
    check(VeryLightAreaColor(true,"DEPARE",color)==color);
    check(!VeryLightOrdinaryContour(true,"DEPCNT",color));
  }

  // The opaque blend moves ordinary ink toward the current theme water. It
  // preserves equal channels and unrelated native color metadata.
  struct Color { unsigned char R,G,B; int identity; };
  const auto faint=integration::FaintDepthContour(Color{200,0,100,7},Color{100,100,100,9});
  check(faint.R==150 && faint.G==50 && faint.B==100 && faint.identity==7);
  const auto pale=integration::FaintDepthContour(Color{0,0,0,1},Color{255,255,255,2});
  check(pale.R==128 && pale.G==128 && pale.B==128);

  // Very Light ordinary object ink must not leak into classified navigation
  // aids, isolated-danger magenta, depth digits or the night palette.
  using integration::SymbolInkRgb;
  using integration::VeryLightObjectSymbol;
  using integration::RecolorVeryLightObjectPixel;
  for (const char* object : {"UWTROC03", "UWTROC04", "WRECKS01", "WRECKS04",
                            "WRECKS05", "WRECKS07", "OBSTRN01", "OBSTRN02",
                            "OBSTRN03", "OBSTRN11", "OBSTRN18", "DANGER01",
                            "DANGER02", "DANGER03", "DANGER51", "DANGER52",
                            "PILPNT02", "MORFAC03", "MORFAC04"}) {
    check(VeryLightObjectSymbol(true, "DAY", object));
    check(VeryLightObjectSymbol(true, "DAY_BRIGHT", object));
    check(!VeryLightObjectSymbol(false, "DAY_BRIGHT", object));
    check(!VeryLightObjectSymbol(true, "NIGHT", object));
    check(!VeryLightObjectSymbol(true, "DUSK", object));
  }
  for (const char* preserved : {"ISODGR51", "SOUNDS01", "SOUNDG01", "BOYLAT01",
                                "BCNCAR01", "LIGHTS13", "XNLIT013", "UNKNOWN1",
                                "", "UWTROC", static_cast<const char*>(nullptr)})
    check(!VeryLightObjectSymbol(true, "DAY_BRIGHT", preserved));
  check(!VeryLightObjectSymbol(true, nullptr, "UWTROC03"));
  check(!VeryLightObjectSymbol(true, "", "UWTROC03"));
  const char fixedSymbol[8]={'U','W','T','R','O','C','0','3'};
  check(VeryLightObjectSymbol(true, "DAY", fixedSymbol));
  const SymbolInkRgb black{83,100,95}, gray{83,100,95};
  for (auto pixel : {black, SymbolInkRgb{104,123,122}, SymbolInkRgb{7,7,7},
                    SymbolInkRgb{125,137,140}}) {
    // The fourth byte represents alpha and must remain byte-identical.
    for (unsigned alpha : {0,1,127,254,255}) {
      unsigned char rgba[]{pixel[0],pixel[1],pixel[2],static_cast<unsigned char>(alpha)};
      check(RecolorVeryLightObjectPixel(rgba,black,gray));
      check(rgba[0]==89 && rgba[1]==89 && rgba[2]==89 && rgba[3]==alpha);
    }
  }
  for (auto pixel : {SymbolInkRgb{197,218,220}, SymbolInkRgb{181,201,171},
                    SymbolInkRgb{182,110,108}, SymbolInkRgb{80,141,120},
                    SymbolInkRgb{172,141,76}, SymbolInkRgb{255,0,255},
                    SymbolInkRgb{104,123,121}}) {
    const auto original=pixel;
    check(!RecolorVeryLightObjectPixel(pixel.data(),black,gray));
    check(pixel==original);
  }
  check(!RecolorVeryLightObjectPixel(nullptr,black,gray));
  std::cout<<checks<<" chart paint policy checks passed\n";
}

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
  check(faint.R==135 && faint.G==65 && faint.B==100 && faint.identity==7);
  const auto pale=integration::FaintDepthContour(Color{0,0,0,1},Color{255,255,255,2});
  check(pale.R==166 && pale.G==166 && pale.B==166);
  std::cout<<checks<<" chart paint policy checks passed\n";
}

#include "integration/ChartCanvasInk.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
int main() {
  using namespace opennav;unsigned checks=0;
  auto check=[&](bool value){++checks;if(!value)throw std::runtime_error("Night chart input check failed");};
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
  std::cout<<checks<<" chart-only Night input checks passed\n";
}

#pragma once
#include <cstdint>
#include <cstring>

namespace opennav::integration {
// Same HLS saturation x1.20 treatment as the owned aid SVGs. Only native red
// and green lenses opt in; white and unknown classes keep their existing ink.
// Apply after theme dimming. Night's weighted luminance cannot increase.
constexpr std::uint32_t RicherLightSectorInk(unsigned color, std::uint32_t rgb,
                                            bool night=false) {
  if(color!=3 && color!=4)return rgb;
  unsigned r=(rgb>>16)&255, g=(rgb>>8)&255, b=rgb&255;
  const unsigned high=r>g?(r>b?r:b):(g>b?g:b);
  const unsigned low=r<g?(r<b?r:b):(g<b?g:b);
  if(high==low)return rgb;
  // Keep the same normalized arithmetic/rounding as Python colorsys used by
  // the SVG generator, including channels exactly at a half-integer boundary.
  const double maxc=high/255.,minc=low/255.,light=(maxc+minc)/2.;
  const double span=maxc-minc;
  double saturation=light<=.5?span/(maxc+minc):span/(2.-maxc-minc);
  const double rc=(maxc-r/255.)/span,gc=(maxc-g/255.)/span,bc=(maxc-b/255.)/span;
  double hue=(r==high?bc-gc:g==high?2.+rc-bc:4.+gc-rc)/6.;
  if(hue<0)hue+=1.;
  saturation*=1.2;if(saturation>1)saturation=1;
  const double m2=light<=.5?light*(1.+saturation):light+saturation-light*saturation;
  const double m1=2.*light-m2;
  const auto channel=[&](double h) {
    if(h<0)h+=1.;
    if(h>=1)h-=1.;
    const double v=h<1./6.?m1+(m2-m1)*h*6.:
        h<.5?m2:h<2./3.?m1+(m2-m1)*(2./3.-h)*6.:m1;
    return unsigned(v*255.+.5);
  };
  const unsigned before=r*2126+g*7152+b*722;
  r=channel(hue+1./3.);g=channel(hue);b=channel(hue-1./3.);
  const unsigned after=r*2126+g*7152+b*722;
  if(night && after>before) {
    r=r*before/after;g=g*before/after;b=b*before/after;
  }
  return (r<<16)|(g<<8)|b;
}
// Only verified presentation instances may select owned light artwork. S-57
// attribute names are fixed six-byte fields. Any ORIENT presence keeps the
// original vector, including malformed/non-finite values handled upstream.
inline const char* PresentationLightAlias(bool enabled, const char* feature,
                                         const char* symbol, const char* attributes,
                                         int count) {
  if (!enabled || !feature || !symbol || std::memcmp(feature, "LIGHTS", 6) ||
      count < 0 || count > 4096 || (count && !attributes)) return nullptr;
  for (int i = 0; i < count; ++i)
    if (!std::memcmp(attributes + i * 6, "ORIENT", 6)) return nullptr;
  if (!std::memcmp(symbol, "LIGHTS11", 8)) return "XNLIT011";
  if (!std::memcmp(symbol, "LIGHTS12", 8)) return "XNLIT012";
  if (!std::memcmp(symbol, "LIGHTS13", 8)) return "XNLIT013";
  return nullptr;
}
} // namespace opennav::integration

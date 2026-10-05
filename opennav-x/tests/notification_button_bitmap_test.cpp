#include "ui/NotificationButtonBitmap.h"
#include <wx/app.h>
#include <wx/frame.h>
#include <wx/image.h>
#include <wx/dcmemory.h>
#include <cstdio>
#include <cstdlib>

class TestApp : public wxApp {
 public:
  bool OnInit() override { return true; }
};
wxIMPLEMENT_APP_NO_MAIN(TestApp);

void Check(bool passed, const char *description) {
  if (!passed) { std::fprintf(stderr, "FAIL: %s\n", description); std::exit(1); }
}
int main(int argc, char **argv) {
  Check(wxEntryStart(argc, argv), "wx start");
  Check(wxTheApp->CallOnInit(), "wx init");
  wxInitAllImageHandlers();
  auto *window = new wxFrame(nullptr, wxID_ANY, "isolated notification fixture");
  const auto physical = opennav::ui::NotificationButtonSize(*window);
  Check(physical.x == physical.y && physical.x >= 44, "full 44 DIP target");
  wxBitmap sheet(264, 198, 32);
  wxMemoryDC dc(sheet);
  dc.SetBackground(wxBrush(wxColour(0xD5,0xE6,0xE8))); dc.Clear();
  using namespace opennav::ui;
  const char *names[]{"notification-info-2", "notification-warning-2", "notification-critical-2"};
  const LightMode modes[]{LightMode::Day,LightMode::Dusk,LightMode::Night};
  int checks=0;
  for(int theme=0;theme<3;++theme) {
    const auto colors=Theme(modes[theme]);
    const std::uint32_t inks[]{NavigationContextInk(modes[theme]),colors.attention,colors.alarm};
    for(int severity=0;severity<3;++severity) {
      for(int size : {44,55,66,88}) {
        const auto bitmap=NotificationButtonBitmap(names[severity],modes[theme],wxSize(size,size));
        Check(bitmap.IsOk() && bitmap.HasAlpha(),"valid alpha bitmap");
        Check(bitmap.GetSize()==wxSize(size,size),"DPI raster keeps full target");
        const auto image=bitmap.ConvertToImage();
        Check(image.GetAlpha(0,0)==0,"rounded transparent corner");
        Check(image.GetAlpha(size/2,size/2)==255,"opaque readable surface");
        int ink_pixels=0,background_pixels=0;
        for(int y=0;y<size;++y) for(int x=0;x<size;++x) {
          const std::uint32_t rgb=(image.GetRed(x,y)<<16)|(image.GetGreen(x,y)<<8)|image.GetBlue(x,y);
          if(rgb==inks[severity] && image.GetAlpha(x,y)==255) ++ink_pixels;
          if(rgb==colors.background && image.GetAlpha(x,y)==255) ++background_pixels;
        }
        Check(ink_pixels>10,"semantic severity ink survives actual rasterization");
        Check(background_pixels>size*size/2,"prototype alert surface survives rasterization");
        if(size==44) dc.DrawBitmap(bitmap,22+severity*88,11+theme*66,true);
        checks+=6;
      }
    }
  }
  Check(!NotificationButtonBitmap("future-upstream-icon",LightMode::Day,wxSize(44,44)).IsOk(),"unknown artwork falls back");
  Check(!NotificationButtonBitmap(names[0],LightMode::Day,wxSize(0,44)).IsOk(),"invalid geometry falls back");
  dc.SelectObject(wxNullBitmap);
  if(argc>1) Check(sheet.SaveFile(wxString::FromUTF8(argv[1]),wxBITMAP_TYPE_PNG),"save paint fixture");
  delete window;
  wxTheApp->OnExit(); wxEntryCleanup();
  std::printf("%d bitmap checks passed across 3 severities, 3 themes and 4 DPI sizes\n",checks+3);
}

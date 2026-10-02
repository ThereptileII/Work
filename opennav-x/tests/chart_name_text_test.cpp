// Offline native text painter; no chart/profile, input source or hardware.
#include "integration/ChartNameText.h"
#include <wx/app.h>
#include <wx/dcmemory.h>
#include <wx/image.h>
#include <iostream>
#include <stdexcept>

class TestApp : public wxApp { public: bool OnInit() override { return true; } };
wxIMPLEMENT_APP_NO_MAIN(TestApp);

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  const std::string output = argv[1];
  if (!wxEntryStart(argc, argv) || !wxTheApp->CallOnInit()) return 2;
  int result = 0, checks = 0;
  const auto check = [&checks](bool ok, const char* why) {
    ++checks; if (!ok) throw std::runtime_error(why);
  };
  try {
    using namespace opennav::integration;
    wxInitAllImageHandlers();
    wxBitmap bitmap(1080,250,24); wxMemoryDC dc(bitmap);
    const wxColour backgrounds[] = {{213,229,229},{52,79,89},{18,30,36}};
    const wxColour inks[] = {{104,123,122},{173,187,177},{117,133,121}};
    for (int theme = 0; theme < 3; ++theme) {
      dc.SetPen(*wxTRANSPARENT_PEN); dc.SetBrush(wxBrush(backgrounds[theme]));
      dc.DrawRectangle(theme*360,0,360,250);
      dc.SetFont(wxFont(9,wxFONTFAMILY_SWISS,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_NORMAL,false,"Arial"));
      dc.SetTextForeground(inks[theme]);
      dc.DrawText(theme==0 ? "DAY / OFFLINE FONT FIXTURE" :
                  theme==1 ? "DUSK / OFFLINE FONT FIXTURE" : "NIGHT / OFFLINE FONT FIXTURE",theme*360+16,12);
      ChartNameTextRun land(dc,wxString::FromUTF8("ARKÖSUND"),1);
      check(land.extra_width==8,"1px tracking includes eight SVG advances");
      land.DrawOpaque(dc,theme*360+16,48);
      dc.SetFont(wxFont(12,wxFONTFAMILY_SWISS,wxFONTSTYLE_ITALIC,wxFONTWEIGHT_NORMAL,false,"Arial"));
      ChartNameTextRun water(dc,wxString::FromUTF8("ÖSTERSJÖN"),5);
      check(water.extra_width==45,"5px water tracking includes nine advances");
      check(water.Draw(dc,theme*360+16,85,inks[theme],92),"Actual alpha-capable native DC required");
      check(dc.GetTextForeground()==inks[theme],"Graphics opacity must not mutate DC ink");
      ChartNameTextRun combined(dc,wxString::FromUTF8("Arko\xCC\x88sund"),1.25);
      check(combined.starts.empty()&&combined.extra_width==0,"Combining name retains whole-string native shaping");
      combined.DrawOpaque(dc,theme*360+16,127);
      ChartNameTextRun shaped(dc,wxString::FromUTF8("\xD8\xA7\xD9\x84\xD8\xA8\xD8\xAD\xD8\xB1"),5);
      check(shaped.starts.empty()&&shaped.extra_width==0,"Arabic retains native whole-string shaping");
      int calls=0;
      shaped.Paint([&](const wxString& text,double x,double y) {
        ++calls;check(text==shaped.text&&x==1&&y==2,"Fallback never rewrites text or origin");
      },1,2);
      check(calls==1,"Unsupported script is painted as exactly one whole string");
      shaped.DrawOpaque(dc,theme*360+16,166);
      ChartNameTextRun stock(dc,"STOCK",0);
      check(stock.extra_width==0&&stock.starts.empty(),"Stock path remains untracked");
    }
    dc.SelectObject(wxNullBitmap);
    const wxImage image=bitmap.ConvertToImage();
    for(int theme=0;theme<3;++theme) {
      int changed=0;
      for(int y=80;y<122;++y)for(int x=theme*360+10;x<(theme+1)*360;++x) {
        const int r=image.GetRed(x,y),g=image.GetGreen(x,y),b=image.GetBlue(x,y);
        if(r!=backgrounds[theme].Red()||g!=backgrounds[theme].Green()||b!=backgrounds[theme].Blue()) {
          ++changed;
          // No rendered water-label pixel can exceed the .36 opacity bound.
          const int channels[]={r,g,b};
          const int bg[]={backgrounds[theme].Red(),backgrounds[theme].Green(),backgrounds[theme].Blue()};
          const int fg[]={inks[theme].Red(),inks[theme].Green(),inks[theme].Blue()};
          for(int c=0;c<3;++c)
            check(std::abs(channels[c]-bg[c])<=std::ceil(std::abs(fg[c]-bg[c])*92./255.)+1,
                  "Water label must not become opaque or ignore alpha");
        }
      }
      check(changed>20,"Water name remains painted");
    }
    check(image.SaveFile(output,wxBITMAP_TYPE_PNG),"Native evidence image saved");
    std::cout<<checks<<" geographic name painter checks passed\n";
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';result=1;}
  wxTheApp->OnExit();wxEntryCleanup();return result;
}

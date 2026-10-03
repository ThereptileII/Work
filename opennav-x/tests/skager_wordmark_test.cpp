// Actual wx bitmap compositing: unchanged approved glyphs, theme ink and rejection.
#include "ui/SkagerWordmark.h"
#include "application/SkagerBrandAsset.h"
#include <wx/app.h>
#include <wx/dcmemory.h>
#include <wx/mstream.h>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <iostream>
#include <stdexcept>
class App : public wxApp { public: bool OnInit() override {return true;} };
wxIMPLEMENT_APP_NO_MAIN(App);
int main(int argc,char** argv) {
  if(argc!=2||!wxEntryStart(argc,argv)||!wxTheApp->CallOnInit())return 2;
  int result=0,checks=0;
  auto check=[&](bool ok,const char* why){++checks;if(!ok)throw std::runtime_error(why);};
  try {
    using namespace opennav::ui;wxInitAllImageHandlers();
    wxMemoryInputStream stream(opennav::application::kSkagerWordmarkPng,
                               sizeof(opennav::application::kSkagerWordmarkPng));
    const wxImage original(stream,wxBITMAP_TYPE_PNG),saved=original.Copy();
    SkagerWordmark wordmark(original);check(wordmark.UsesCoverage(),"Approved crop accepted");
    wordmark.Bitmap(680,LightMode::Day);
    const auto& full=wordmark.Raster();
    check(full.IsOk()&&full.HasAlpha(),"Approved source must render with coverage alpha");
    for(int y=0;y<214;++y)for(int x=0;x<680;++x)
      if(x<8||x>=672||y<8||y>=205||(y>=120&&y<160))
        check(full.GetAlpha()[y*680+x]==0,"Background-only margins and row separator are completely transparent");
    auto colour=[](unsigned rgb){return wxColour(rgb>>16,(rgb>>8)&255,rgb&255);};
    // Independent literal expectations from the final prototype's semantic roles.
    const unsigned backgrounds[]={0x152326,0x1d282e,0x0c1115};
    const unsigned primary[]={0xf3f5ee,0xe2e5db,0x91988e};
    const unsigned accent[]={0xb6efce,0x9bc5b1,0x85a995};
    const LightMode modes[]={LightMode::Day,LightMode::Dusk,LightMode::Night};
    const char* names[]={"Day","Dusk","Night"};
    wxBitmap sheet(960,520);wxMemoryDC sheetdc(sheet);
    sheetdc.SetBackground(*wxWHITE_BRUSH);sheetdc.Clear();sheetdc.SetTextForeground(*wxBLACK);
    for(int theme=0;theme<3;++theme) {
      sheetdc.DrawText(names[theme],theme*320+10,8);
      for(int size=0;size<3;++size) {
        const int percent=100+size*25,width=148*percent/100,pw=180*percent/100,ph=68*percent/100;
        const auto before=wordmark.Builds();
        const auto& bitmap=wordmark.Bitmap(width,modes[theme]);
        check(bitmap.IsOk()&&wordmark.Builds()==before+1,"Theme/size change rebuilds");
        wordmark.Bitmap(width,modes[theme]);check(wordmark.Builds()==before+1,"Repeated paint reuses bitmap");
        const auto& image=wordmark.Raster();
        check(image.GetWidth()==width&&image.GetHeight()==width*214/680&&image.HasAlpha(),"Aspect and soft alpha retained");
        const auto* alpha=image.GetAlpha();const auto* rgb=image.GetData();
        for(int row=0;row<2;++row) {
          const int y0=row?image.GetHeight()*160/214:0,y1=row?image.GetHeight():image.GetHeight()*120/214;
          int runs=0,lit=0,peak=0;bool in=false;
          for(int x=0;x<width;++x) {
            bool occupied=false;
            for(int y=y0;y<y1;++y) {const int at=y*width+x;
              if(alpha[at]>32) {occupied=true;++lit;peak=(std::max)(peak,int(alpha[at]));}
              if(alpha[at]>200) {
                const unsigned expected=row?accent[theme]:primary[theme];
                check(std::abs(int(rgb[at*3])-int(expected>>16))<=1 &&
                      std::abs(int(rgb[at*3+1])-int((expected>>8)&255))<=1 &&
                      std::abs(int(rgb[at*3+2])-int(expected&255))<=1,"Ink follows prototype role without teal contamination");
              }
            }
            if(occupied&&!in)++runs;
            in=occupied;
          }
          check(runs==(row?3:6),"Each approved letter remains separately readable");
          std::cout<<names[theme]<<" "<<percent<<"% row "<<row<<": runs="<<runs<<" lit="<<lit<<" peak="<<peak<<"\n";
          check(lit>(row?100:1000)&&peak>200,"Both rows retain substantial visible ink");
        }
        wxBitmap panel(pw,ph);wxMemoryDC dc(panel);dc.SetBackground(wxBrush(colour(backgrounds[theme])));dc.Clear();
        dc.DrawBitmap(bitmap,16*percent/100,(ph-image.GetHeight())/2,true);dc.SelectObject(wxNullBitmap);
        const auto composed=panel.ConvertToImage();
        const int ox=16*percent/100,oy=(ph-image.GetHeight())/2;
        auto luminance=[](int r,int g,int b){
          auto channel=[](int v){const double c=v/255.;return c<=.04045?c/12.92:std::pow((c+.055)/1.055,2.4);};
          return .2126*channel(r)+.7152*channel(g)+.0722*channel(b);
        };
        const double background_luminance=luminance(backgrounds[theme]>>16,(backgrounds[theme]>>8)&255,backgrounds[theme]&255);
        for(int row=0;row<2;++row) {
          double strongest=0;
          for(int y=row?image.GetHeight()*160/214:0;y<(row?image.GetHeight():image.GetHeight()*120/214);++y)
            for(int x=0;x<width;++x)
              strongest=std::max(strongest,luminance(composed.GetRed(x+ox,y+oy),composed.GetGreen(x+ox,y+oy),composed.GetBlue(x+ox,y+oy)));
          const double contrast=(strongest+.05)/(background_luminance+.05);
          check(contrast>=4.5,"Both rows retain a readable bright core in each theme/size");
          std::cout<<names[theme]<<" "<<percent<<"% row "<<row<<": peak contrast="<<contrast<<"\n";
        }
        for(int y=0;y<image.GetHeight();++y) for(int x=0;x<width;++x)
          if(alpha[y*width+x]==0) {
            check(composed.GetRed(x+ox,y+oy)==(backgrounds[theme]>>16)&&
                  composed.GetGreen(x+ox,y+oy)==((backgrounds[theme]>>8)&255)&&
                  composed.GetBlue(x+ox,y+oy)==(backgrounds[theme]&255),"Transparent logo pixels exactly reveal native header");
          }
        sheetdc.DrawText(wxString::Format("%d%%",percent),theme*320+10,35+size*130);
        sheetdc.DrawBitmap(panel,theme*320+10,55+size*130,false);
      }
      // Same size with a new light mode must also rebuild, independent of DPI.
      const auto before=wordmark.Builds();wordmark.Bitmap(222,modes[(theme+1)%3]);
      check(wordmark.Builds()==before+1,"Light-mode change alone invalidates cache");
    }
    for(int variant=0;variant<3;++variant) {
      wxImage wrong=original.Copy();
      if(variant==0)wrong.GetData()[0]^=1;
      if(variant==1)wrong.InitAlpha();
      if(variant==2)wrong=wrong.Scale(679,214);
      SkagerWordmark rejected(wrong);check(!rejected.UsesCoverage(),"Unexpected identity/dimensions/alpha fails closed");
      check(rejected.Bitmap(185,LightMode::Night).IsOk(),"Original artwork fallback remains drawable");
      const auto expected=wrong.Scale(185,185*wrong.GetHeight()/wrong.GetWidth(),wxIMAGE_QUALITY_HIGH);
      check(std::memcmp(expected.GetData(),rejected.Raster().GetData(),185*expected.GetHeight()*3)==0,"Rejected image is not recolored");
    }
    check(std::memcmp(original.GetData(),saved.GetData(),680*214*3)==0&&!original.HasAlpha(),"Approved decoded image remains unchanged");
    SkagerWordmark missing{wxImage()};check(!missing.UsesCoverage()&&!missing.Bitmap(148,LightMode::Day).IsOk(),"Missing source safely rejects");
    check(!wordmark.Bitmap(0,LightMode::Day).IsOk()&&!wordmark.Bitmap(2049,LightMode::Day).IsOk(),"Invalid target size rejects");
    sheetdc.DrawText("Actual wx native component raster; rows 100 / 125 / 150%; Linux component evidence only",10,475);
    sheetdc.SelectObject(wxNullBitmap);check(sheet.ConvertToImage().SaveFile(argv[1],wxBITMAP_TYPE_PNG),"Native drawing fixture saved");
    std::cout<<checks<<" wordmark checks passed; 9 native theme/size drawings\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';result=1;}
  wxTheApp->OnExit();wxEntryCleanup();return result;
}

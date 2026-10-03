// Bounded actual wx controls only; no application/profile/navigation fixture.
#include "ui/FloatingSurface.h"
#include <wx/app.h>
#include <wx/dcscreen.h>
#include <wx/dcmemory.h>
#include <wx/dcbuffer.h>
#include <wx/uiaction.h>
#include <wx/image.h>
#include <wx/sizer.h>
#include <wx/timer.h>
#include <iostream>
#ifdef __WXGTK__
#include <gtk/gtk.h>
#endif
using namespace opennav::ui;
class ChromeTest final : public wxApp {
 public:
  bool OnInit() override {
    if(argc!=2)return false; directory_=argv[1];wxInitAllImageHandlers();
    frame_=new wxFrame(nullptr,wxID_ANY,"Component only",{0,0},{420,200},wxBORDER_NONE);
    frame_->SetClientSize(420,200);
    panel_=new wxPanel(frame_,wxID_ANY);
    nav_=new XNavButton(panel_,wxID_ANY,"Instruments","Vessel instruments");
    oracle_=new wxPanel(panel_,wxID_ANY,{9,10},{61,61});
    oracle_->SetBackgroundStyle(wxBG_STYLE_PAINT);
    oracle_->Bind(wxEVT_PAINT,[this](wxPaintEvent&) {
      wxAutoBufferedPaintDC dc(oracle_);
      dc.SetBackground(wxBrush(Colour(Theme(mode_).background)));dc.Clear();
      dc.SetFont(UiFont(*nav_,10));dc.SetTextForeground(Colour(Theme(mode_).muted));
      dc.DrawText("Instruments",(61-dc.GetTextExtent("Instruments").x)/2,39);
    });
    nav_->SetNavigationItem();nav_->SetIcon(XNavIcon::Instruments);nav_->SetSize(9,82,61,61);
    tools_=new XNavFloatingSurface(*frame_,"Component tools");
    auto* row=new wxBoxSizer(wxHORIZONTAL);row->AddSpacer(4);
    for(auto icon:{XNavIcon::Ruler,XNavIcon::Pin,XNavIcon::Plus,XNavIcon::Minus}) {
      auto* b=new XNavButton(tools_,wxID_ANY,"", "Component action");
      b->SetIcon(icon);b->SetIconOnly();b->SetFloating();b->SetRole(ButtonRole::Quiet);b->SetMinSize({44,44});
      row->Add(b,0,wxTOP|wxBOTTOM,4);buttons_.push_back(b);
      if(icon==XNavIcon::Pin)row->AddSpacer(5);
    }
    row->AddSpacer(4);tools_->SetSizerAndFit(row);
    frame_->Show();wxUIActionSimulator().MouseMove({1,1});timer_.SetOwner(this);Bind(wxEVT_TIMER,&ChromeTest::Step,this);timer_.StartOnce(100);return true;
  }
 private:
  void Step(wxTimerEvent&) {
    if(step_%2==0) {
      mode_=LightMode(step_/2);frame_->SetBackgroundColour(Colour(Theme(mode_).background));
      panel_->SetBackgroundColour(Colour(Theme(mode_).background));
      nav_->SetLightMode(mode_);for(auto* b:buttons_)b->SetLightMode(mode_);
#ifdef SKAGER_CHROME_CORRECTION
      tools_->SetChartToolsTheme(mode_);
#else
      tools_->SetBackgroundColour(Colour(FloatingTheme(mode_).surface));
#endif
      tools_->Present({180,70});frame_->Refresh();oracle_->Refresh();tools_->Refresh();
    } else {
      if(nav_->GetSize()!=wxSize(61,61) || tools_->GetSize()!=wxSize(189,52)) {std::cerr<<"Wrong component geometry\n";std::exit(1);}
      const auto name=mode_==LightMode::Day?"Day":mode_==LightMode::Dusk?"Dusk":"Night";

#ifdef __WXGTK__
      // Read the actual private X11 root through GTK; force GDK_BACKEND=x11
      // when running the fixture to avoid the host Wayland display.
      auto* pixels=gdk_pixbuf_get_from_window(gdk_get_default_root_window(),0,0,420,200);
      if(!pixels || !gdk_pixbuf_save(pixels,(directory_+"/"+name+".png").utf8_str(),"png",nullptr,nullptr))std::exit(1);
      g_object_unref(pixels);
#else
      wxScreenDC screen;wxBitmap bitmap(420,200);wxMemoryDC copy(bitmap);
      copy.Blit(0,0,420,200,&screen,0,0);copy.SelectObject(wxNullBitmap);
      bitmap.ConvertToImage().SaveFile(directory_+"/"+name+".png",wxBITMAP_TYPE_PNG);
#endif
      wxClientDC measure(nav_);measure.SetFont(UiFont(*nav_,10));
      const auto extent=measure.GetTextExtent("Instruments");
      std::cout<<name<<" caption width="<<extent.x<<"; button="<<nav_->GetSize().x<<"x"<<nav_->GetSize().y
        <<"; tools="<<tools_->GetSize().x<<"x"<<tools_->GetSize().y<<'\n';
    }
    if(++step_==6){tools_->Destroy();frame_->Destroy();ExitMainLoop();return;}
    timer_.StartOnce(150);
  }
  wxFrame* frame_=nullptr;wxPanel* panel_=nullptr;wxPanel* oracle_=nullptr;XNavButton* nav_=nullptr;XNavFloatingSurface* tools_=nullptr;
  std::vector<XNavButton*>buttons_;wxTimer timer_;wxString directory_;int step_=0;LightMode mode_{};
};
wxIMPLEMENT_APP_NO_MAIN(ChromeTest);
int main(int argc,char**argv){return wxEntry(argc,argv);}

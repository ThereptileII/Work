// Executes extracted production cache/theme/hit-rectangle methods and the
// production CreateBmp entry hook. The stock renderer is a counted stand-in.
#include "ui/NotificationButtonBitmap.h"
#include "color_types.h"
#include <wx/app.h>
#include <wx/frame.h>
#include <cassert>
#include <iostream>
bool g_bopengl=true;
bool modern=true;
namespace opennav { bool IsXNav() { return modern; } }
class NotificationButton {
 public:
  wxWindow *m_parent;
  wxBitmap m_StatBmp;
  wxRect m_rect{123,101,37,37};
  wxString m_NoteIconName="notification-info-2",m_lastNoteIconName;
  bool m_xnavBitmap=false;
  unsigned m_texobj=1;
  ColorScheme m_cs=GLOBAL_COLOR_SCHEME_DAY;
  int builds=0,textures=0,fallbacks=0;
  bool UpdateStatus(bool bnew=false);
  void SetColorScheme(ColorScheme);
  wxRect GetLogicalRect() const;
  void CreateBmp(bool);
  void CreateTexture() { ++textures; }
};
#include "notification-methods.inc"
class App:public wxApp { public:bool OnInit()override{return true;} };
wxIMPLEMENT_APP_NO_MAIN(App);
int main(int argc,char **argv) {
  assert(wxEntryStart(argc,argv));assert(wxTheApp->CallOnInit());
  auto *parent=new wxFrame(nullptr,wxID_ANY,"cache fixture");
  NotificationButton b;b.m_parent=parent;
  assert(b.UpdateStatus() && b.m_xnavBitmap && b.builds==1 && b.textures==1);
  const auto target=opennav::ui::NotificationButtonSize(*parent);
  assert(b.m_rect.GetSize()==target && b.m_rect.GetPosition()==wxPoint(123,101));
  const auto hit=b.GetLogicalRect();
  assert(hit.GetSize()==parent->FromDIP(wxSize(44,44)));
  assert(hit.Contains(hit.GetTopLeft()) && hit.Contains(hit.GetBottomRight()));
  assert(!b.UpdateStatus() && b.builds==1); // unchanged severity reuses bitmap
  for(auto scheme:{GLOBAL_COLOR_SCHEME_DUSK,GLOBAL_COLOR_SCHEME_NIGHT,GLOBAL_COLOR_SCHEME_DAY}) {
    const int before=b.builds;
    b.SetColorScheme(scheme);
    assert(b.builds==before+1 && b.m_xnavBitmap && b.m_rect.GetPosition()==wxPoint(123,101));
  }
  b.m_NoteIconName="notification-warning-2";
  assert(b.UpdateStatus() && b.m_xnavBitmap);
  b.m_NoteIconName="notification-critical-2";
  assert(b.UpdateStatus() && b.m_xnavBitmap);
  const int builds=b.builds,textures=b.textures;
  b.m_StatBmp=wxBitmap(55,55,32); // stale cached raster after a DPI transition
  assert(b.UpdateStatus() && b.builds==builds+1 && b.textures==textures+1);
  assert(b.m_StatBmp.GetSize()==target && b.m_rect.GetSize()==target);
  b.m_NoteIconName="unrecognized-upstream-artwork";
  assert(b.UpdateStatus() && !b.m_xnavBitmap && b.fallbacks==1);
  modern=false;b.m_NoteIconName="notification-info-2";
  assert(b.UpdateStatus() && !b.m_xnavBitmap && b.fallbacks==2);
  const int stock=b.builds;
  b.m_StatBmp=wxBitmap(19,19,32);
  assert(!b.UpdateStatus() && b.builds==stock); // no modern sizing in Legacy
  delete parent;wxTheApp->OnExit();wxEntryCleanup();
  std::cout<<"Actual cache/theme/hit methods: severity, theme return, DPI replacement, unchanged placement, unknown/Legacy fallback passed\n";
}

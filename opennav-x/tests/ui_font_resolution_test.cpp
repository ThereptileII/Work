// Read the actual production font selection; never install or copy fonts.
#include "ui/Controls.h"
#include <wx/app.h>
#include <wx/dcmemory.h>
#include <wx/fontenum.h>
#include <wx/frame.h>
#include <iostream>
#include <stdexcept>
#ifdef __WXMSW__
#include <wx/msw/wrapwin.h>
#endif

class App : public wxApp { public: bool OnInit() override { return true; } };
wxIMPLEMENT_APP_NO_MAIN(App);
int main(int argc, char** argv) {
  if (!wxEntryStart(argc, argv) || !wxTheApp->CallOnInit()) return 2;
  int result = 0;
  try {
    wxFrame frame(nullptr, wxID_ANY, "Isolated font resolution");
    wxString first;
    for (const auto* name : {"Segoe UI Variable Display", "Segoe UI", "Arial"}) {
      const bool available = wxFontEnumerator::IsValidFacename(name);
      std::cout << "available: " << name << "=" << available << '\n';
      if (available && first.empty()) first = name;
    }
    if (first.empty()) first = "Arial";
    std::cout << "system-font: " << wxNORMAL_FONT->GetFaceName() << '\n';
    wxBitmap bitmap(512, 128); wxMemoryDC dc(bitmap);
    for (const auto role : {std::pair<int,int>{11,400}, {23,650}, {48,400}}) {
      const auto font = opennav::ui::UiFontWeight(frame, role.first, role.second);
      if (!font.IsOk() || font.GetFaceName() != first ||
          font.GetNumericWeight() != role.second)
        throw std::runtime_error("Production font differs from the first installed prototype family/weight");
      dc.SetFont(font);
      std::cout << "production: px=" << role.first << " weight=" << role.second
                << " selected=" << font.GetFaceName();
#ifdef __WXMSW__
      wchar_t actual[256]{};
      if (!GetTextFaceW(static_cast<HDC>(dc.GetHDC()), 256, actual))
        throw std::runtime_error("Could not measure the actual selected Windows font");
      const wxString rendered(actual);
      std::cout << " GDI-face=" << rendered << " dpi=" << frame.GetDPI().x;
      if (rendered.CmpNoCase(first) != 0)
        throw std::runtime_error("Windows substituted the selected prototype face");
#else
      std::cout << " actual-platform-face-not-qualified-on-Linux";
#endif
      std::cout << '\n';
    }
    std::cout << "Production family/weight checks passed; native Windows/boat DPI review remains required\n";
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; result = 1; }
  wxTheApp->OnExit(); wxEntryCleanup(); return result;
}

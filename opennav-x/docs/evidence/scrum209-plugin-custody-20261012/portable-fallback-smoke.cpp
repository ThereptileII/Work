#include <wx/string.h>
#include <wx/log.h>
#include "integration/PluginPresentationFallback.h"
#include <iostream>
#include <stdexcept>
using namespace opennav::integration;
unsigned checks=0;
void Check(bool v) { ++checks; if(!v) throw std::runtime_error("fallback smoke failed"); }
bool Residue(wxDynamicLibrary& lib,const wxString&,const PluginCompatibilityCheck&) {
  Check(lib.Load("/usr/lib/libm.so.6")); return false;
}
int main() {
  wxLogNull quiet;
  PluginModuleInspectionCustody custody;
  wxDynamicLibrary lib;
  Check(LoadPluginWithPresentationFallback(lib,"/usr/lib/libm.so.6",{},custody)==PluginModuleLoadOutcome::Loaded);
  Check(lib.IsLoaded() && lib.GetSymbol("cos")!=nullptr);
  Check(LoadPluginWithPresentationFallback(lib,"/nonexistent/skager-inert-no-library.so",{},custody)==PluginModuleLoadOutcome::LoadFailed);
  Check(!lib.IsLoaded());
  RegisterPluginPresentationLoader(Residue);
  Check(LoadPluginWithPresentationFallback(lib,"/nonexistent/skager-inert-no-library.so",{},custody)==PluginModuleLoadOutcome::UnloadFailed);
  Check(lib.IsLoaded());
  Check(UnloadPluginModuleChecked(lib,custody));
  Check(!lib.IsLoaded() && !custody);
  RegisterPluginPresentationLoader(nullptr);
  std::cout<<"PASS: "<<checks<<" actual portable fallback checks\n";
}

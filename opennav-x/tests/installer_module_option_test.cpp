#include "integration/InstallerSelfTest.h"
#include <wx/cmdline.h>
#include <wx/init.h>
#include <wx/file.h>
#include <wx/filename.h>
#include <wx/jsonreader.h>
#include <wx/jsonval.h>
#include <iostream>
#include <stdexcept>
int main(int argc,char**argv) {
  wxInitializer initialized;
  if(!initialized.IsOk() || argc!=2)return 2;
  using namespace opennav::integration;
  const wxString folder=wxString::FromUTF8(argv[1]);
  auto check=[](bool ok){if(!ok)throw std::runtime_error("Early self-test option contract");};
  auto parse=[&](const wxString& line){wxCmdLineParser parser(line);AddInstallerSelfTest(parser);
    check(parser.Parse(false)==0);return ParseInstallerSelfTest(parser);};
  try {
    check(!parse(""));check(!InstallerSelfTestRequested());check(RunInstallerSelfTest()==2);
    check(parse("--skager-chart-module-check unused.dll"));
    check(InstallerSelfTestRequested());check(RunInstallerSelfTest()==2);
    const auto standard=folder+"/normal.json";
    check(parse("--opennav-self-test "+standard));check(RunInstallerSelfTest()==0);
    wxFile file(standard);wxString text;check(file.ReadAll(&text));wxJSONValue result;
    check(wxJSONReader().Parse(text,&result)==0);
    check(result["passed"].AsBool() && !result["profile_initialized"].AsBool() &&
      !result["plugins_loaded"].AsBool() && !result.HasMember("chart_module"));
    check(RunInstallerSelfTest()==2); // Existing report cannot be overwritten.
    check(parse("--opennav-self-test relative.json"));check(RunInstallerSelfTest()==2);
#ifndef __WXMSW__
    const auto module=folder+"/module.json";
    check(parse("--opennav-self-test "+module+" --skager-chart-module-check unused.dll"));
    check(RunInstallerSelfTest()==1);
    wxFile report(module);check(report.ReadAll(&text));check(wxJSONReader().Parse(text,&result)==0);
    check(!result["passed"].AsBool() && !result["chart_module"]["passed"].AsBool());
    check(result["chart_module"]["reason"].AsString()=="Native Windows required");
#endif
    std::cout<<"Early self-test default/explicit-only/refusal contracts passed\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}

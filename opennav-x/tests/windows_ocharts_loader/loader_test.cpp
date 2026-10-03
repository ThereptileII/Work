#include "integration/OChartsModuleLoader.h"
#include "integration/PluginPresentationFallback.h"
#include "picosha2.h"
#include <wx/init.h>
#include <wx/log.h>
#include <windows.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <thread>
#include <vector>

namespace fs=std::filesystem;
using namespace opennav::integration;
namespace {
void Require(bool value,const std::string& why) {if(!value) throw std::runtime_error(why);}
wxString W(const fs::path& p) {return wxString(p.wstring());}
std::string Read(const fs::path& p) {
  if(!fs::exists(p)) return {};
  std::ifstream in(p,std::ios::binary);
  return {std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>()};
}
std::string Hash(const fs::path& p) {return picosha2::hash256_hex_string(Read(p));}
void Copy(const fs::path& from,const fs::path& to) {
  fs::create_directories(to.parent_path());
  fs::copy_file(from,to,fs::copy_options::overwrite_existing);
}
void Env(const wchar_t* key,const fs::path& value) {
  Require(SetEnvironmentVariableW(key,value.c_str())!=0,"set fixture environment");
}
bool Writable(const fs::path& path) {
  HANDLE h=CreateFileW(path.c_str(),GENERIC_WRITE,FILE_SHARE_READ|FILE_SHARE_WRITE|FILE_SHARE_DELETE,
                       nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
  if(h==INVALID_HANDLE_VALUE) return false;
  CloseHandle(h);return true;
}
void NeverLoadedVendor() {
  Require(GetModuleHandleW(L"o-charts_pi.dll")==nullptr,"vendor DLL must never be loaded by mechanics tests");
}
void Flip(const fs::path& path) {
  std::fstream f(path,std::ios::binary|std::ios::in|std::ios::out);char b=0;
  Require(static_cast<bool>(f.get(b)),"read byte for negative control");
  f.seekp(0);f.put(b^1);f.close();
}
struct Case {
  fs::path dir,original,adapter,resources,events;
  OChartsModuleRequest request;
  int compatibility_calls=0;
  Case(const fs::path& work,const std::string& name,const fs::path& vendor,
       const fs::path& fixture) : dir(work/name),original(dir/"original/o-charts_pi.dll"),
       adapter(dir/"adapter/skager-ocharts-adapter.dll"),resources(dir/L"charts-å"),events(dir/"events.log") {
    Require(!fs::exists(dir),"case output must be fresh: "+name);
    Copy(vendor,original);Copy(fixture,adapter);fs::create_directories(resources);
    request.original=W(original);request.adapter=W(adapter);request.resources=W(resources);
    request.package_available=true;request.main_thread=true;request.safe_mode=false;
    request.adapter_sha256=Hash(adapter);request.adapter_bytes=fs::file_size(adapter);
    Env(L"SKAGER_LOADER_EVENTS",events);Env(L"SKAGER_LOADER_ORIGINAL",original);
    Env(L"SKAGER_LOADER_RESOURCES",resources);SetEnvironmentVariableW(L"SKAGER_LOADER_MODE",L"");
  }
  PluginCompatibilityCheck Compatible() {
    return [this](const wxString& path) {
      ++compatibility_calls;Require(path==request.adapter,"actual adapter compatibility input");
      // The production callback intentionally runs between the two locked hashes.
      Require(Writable(original),"original lock must be released for compatibility inspection");
      // The caller's ABI predicate is an explicit contract input. The runner
      // verifies this exact locally-built fixture is a Win32 PE before use.
      // This does not qualify the real adapter's OpenCPN import ABI.
      return true;
    };
  }
  void Mode(const wchar_t* mode) {SetEnvironmentVariableW(L"SKAGER_LOADER_MODE",mode);}
  void Events(const std::string& expected) {
    const auto actual=Read(events);Require(actual==expected,"unexpected DLL lifecycle in "+dir.filename().string()+": "+actual);
  }
};
const std::string entered="attach\nbind\noriginal-write-delete-locked\n";
int passed=0;
void Pass(const char* name) {++passed;std::cout<<"PASS "<<name<<"\n";}
void Refused(Case& c,const char* reason,int calls,const std::string& events="") {
  wxDynamicLibrary library;
  const auto result=LoadOChartsModule(library,c.request,c.Compatible());
  Require(!result.loaded && result.applicable && result.reason==reason,"wrong refusal: "+result.reason);
  Require(!library.IsLoaded(),"refused adapter must be unloaded");
  Require(c.compatibility_calls==calls,"wrong compatibility-call count");
  c.Events(events);NeverLoadedVendor();Pass(c.dir.filename().string().c_str());
}
fs::path fallback_adapter;
int callback_calls=0;
bool CallbackLoad(wxDynamicLibrary& library,const wxString&,const PluginCompatibilityCheck& check) {
  ++callback_calls;Require(check && check(W(fallback_adapter)),"callback forwarded compatibility");
  return library.Load(W(fallback_adapter));
}
bool CallbackCleanRefusal(wxDynamicLibrary& library,const wxString& original,const PluginCompatibilityCheck& check) {
  CallbackLoad(library,original,check);
  Require(UnloadPluginModuleChecked(library),"clean refused fixture unload");return false;
}
bool CallbackResidue(wxDynamicLibrary& library,const wxString& original,const PluginCompatibilityCheck& check) {
  CallbackLoad(library,original,check);return false;
}
void StatusCases() {
  SkagerChartPresentationStatusV1 value{};value.structBytes=sizeof(value);value.version=1;
  for(auto state : {SKAGER_CHART_UNBOUND,SKAGER_CHART_BOUND_PENDING_INITIALIZATION,
                     SKAGER_CHART_SELECTED,SKAGER_CHART_STANDARD_FALLBACK}) {
    value.state=state;
    value.reason=state==SKAGER_CHART_UNBOUND ? SKAGER_CHART_REASON_UNBOUND :
                 state==SKAGER_CHART_STANDARD_FALLBACK ? SKAGER_CHART_REASON_RESOURCE_VERIFICATION : SKAGER_CHART_REASON_NONE;
    Require(ValidOChartsStatus(value),"valid copied status refused");
    auto wrong=value;wrong.reserved[7]=1;Require(!ValidOChartsStatus(wrong),"reserved status accepted");
    wrong=value;wrong.reason=99;Require(!ValidOChartsStatus(wrong),"unknown reason accepted");
  }
  value.state=SKAGER_CHART_STANDARD_FALLBACK;value.reason=SKAGER_CHART_REASON_RENDERER_INITIALIZATION;
  Require(ValidOChartsStatus(value),"initialization fallback status");
  value.state=99;Require(!ValidOChartsStatus(value),"unknown state accepted");
  Pass("copied status matrix");
}
}
int main(int argc,char**argv) {
  try {
    Require(argc==7,"arguments: vendor fixture-directory work original-junction adapter-junction report");
    wxInitializer initialize;Require(initialize.IsOk(),"wx initialization");
    delete wxLog::SetActiveTarget(new wxLogStderr());
    const fs::path vendor=fs::absolute(argv[1]),fixtures=fs::absolute(argv[2]),work=fs::absolute(argv[3]);
    const fs::path original_junction=fs::absolute(argv[4]),adapter_junction=fs::absolute(argv[5]);
    Require(Hash(vendor)==OriginalOChartsSha256,"exact accepted vendor hash required");
    Require(!fs::exists(work),"fresh test work directory required");fs::create_directories(work);
    const auto good=fixtures/"fixture_good.dll";
    NeverLoadedVendor();StatusCases();
    for(const char* mode : {"unavailable","safe","not-main","standard-empty-resources","wrong-original-name"}) {
      Case c(work,mode,vendor,good);
      if(std::string(mode)=="unavailable") c.request.package_available=false;
      if(std::string(mode)=="safe") c.request.safe_mode=true;
      if(std::string(mode)=="not-main") c.request.main_thread=false;
      if(std::string(mode)=="standard-empty-resources") c.request.resources.clear();
      if(std::string(mode)=="wrong-original-name") c.request.original=W(c.dir/"different.dll");
      wxDynamicLibrary library;auto result=LoadOChartsModule(library,c.request,c.Compatible());
      Require(!result.loaded && !library.IsLoaded() && !c.compatibility_calls,"ineligible load reached module/compatibility");
      Require(result.applicable==(std::string(mode)=="unavailable"),"wrong applicability");
      if(result.applicable) Require(result.reason=="private chart presentation is not included in this build","unavailable reason");
      c.Events("");NeverLoadedVendor();Pass(mode);
    }
    {Case c(work,"actual-worker-thread",vendor,good);OChartsModuleResult r;
      std::thread worker([&]{wxDynamicLibrary lib;r=LoadOChartsModule(lib,c.request,c.Compatible());});worker.join();
      Require(!r.loaded && !r.applicable && c.compatibility_calls==0,"actual worker bypassed main-thread guard");c.Events("");Pass("actual worker thread");}
    {Case c(work,"wrong-original-hash",vendor,good);Flip(c.original);Refused(c,"original plugin identity is unsupported",0);}
    {Case c(work,"wrong-adapter-hash",vendor,good);c.request.adapter_sha256=std::string(64,'0');Refused(c,"private adapter is missing or changed",0);}
    {Case c(work,"wrong-adapter-size",vendor,good);++c.request.adapter_bytes;Refused(c,"private adapter is missing or changed",0);}
    {Case c(work,"missing-adapter",vendor,good);fs::remove(c.adapter);Refused(c,"private adapter is missing or changed",0);}
    {Case c(work,"relative-original",vendor,good);c.request.original="o-charts_pi.dll";Refused(c,"original plugin identity is unsupported",0);}
    {Case c(work,"unc-original",vendor,good);c.request.original="\\\\localhost\\skager-no-share\\o-charts_pi.dll";Refused(c,"original plugin identity is unsupported",0);}
    {Case c(work,"original-reparse-parent",vendor,good);c.request.original=W(original_junction/"o-charts_pi.dll");
      const auto attr=GetFileAttributesW(original_junction.c_str());Require(attr!=INVALID_FILE_ATTRIBUTES && (attr&FILE_ATTRIBUTE_REPARSE_POINT),"original junction not created");Refused(c,"original plugin identity is unsupported",0);}
    {Case c(work,"adapter-reparse-parent",vendor,good);c.request.adapter=W(adapter_junction/"skager-ocharts-adapter.dll");
      const auto attr=GetFileAttributesW(adapter_junction.c_str());Require(attr!=INVALID_FILE_ATTRIBUTES && (attr&FILE_ATTRIBUTE_REPARSE_POINT),"adapter junction not created");Refused(c,"private adapter is missing or changed",0);}
    {Case c(work,"exclusive-original-lock",vendor,good);
      HANDLE h=CreateFileW(c.original.c_str(),GENERIC_WRITE,0,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
      Require(h!=INVALID_HANDLE_VALUE,"create conflicting original lock");Refused(c,"original plugin identity is unsupported",0);CloseHandle(h);}
    {Case c(work,"exclusive-adapter-lock",vendor,good);
      HANDLE h=CreateFileW(c.adapter.c_str(),GENERIC_WRITE,0,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
      Require(h!=INVALID_HANDLE_VALUE,"create conflicting adapter lock");Refused(c,"private adapter is missing or changed",0);CloseHandle(h);}
    for(bool mutate_original : {false,true}) {
      Case c(work,mutate_original ? "changed-original-after-compatibility" : "changed-adapter-after-compatibility",vendor,good);
      wxDynamicLibrary library;auto r=LoadOChartsModule(library,c.request,[&](const wxString& path){
        Require(path==c.request.adapter,"race callback path");Flip(mutate_original ? c.original : c.adapter);return true;});
      Require(!r.loaded && r.reason=="plugin changed during qualification" && !library.IsLoaded(),"post-compatibility identity not rechecked");
      c.Events("");NeverLoadedVendor();Pass(c.dir.filename().string().c_str());
    }
    {Case c(work,"compatibility-refused",vendor,good);wxDynamicLibrary lib;int calls=0;
      auto r=LoadOChartsModule(lib,c.request,[&](const wxString& p){++calls;Require(p==c.request.adapter,"compatibility path");return false;});
      Require(!r.loaded && r.reason=="private adapter ABI is incompatible" && calls==1 && !lib.IsLoaded(),"compatibility refusal");c.Events("");Pass("compatibility refused");}
    {Case c(work,"compatibility-missing",vendor,good);wxDynamicLibrary lib;
      auto r=LoadOChartsModule(lib,c.request,{});Require(!r.loaded && r.reason=="private adapter ABI is incompatible" && !lib.IsLoaded(),"missing compatibility callback");c.Events("");Pass("compatibility missing");}
    {Case c(work,"invalid-pe-after-exact-hash",vendor,good);{std::ofstream f(c.adapter,std::ios::binary|std::ios::trunc);f<<"not a DLL";}
      c.request.adapter_sha256=Hash(c.adapter);c.request.adapter_bytes=fs::file_size(c.adapter);
      wxDynamicLibrary lib;auto r=LoadOChartsModule(lib,c.request,[](const wxString&){return true;});
      Require(!r.loaded && r.reason=="private adapter did not load" && !lib.IsLoaded(),"LoadLibrary failure did not refuse");c.Events("");Pass("invalid PE load refused");}
    {Case c(work,"over-capacity-utf8-resource",vendor,good);
      c.request.resources="C:\\"+wxString(wxUniChar(0x00e5),2048);
      Refused(c,"presentation path cannot be represented",1);}
    for(const char* variant : {"no_bind","no_status","no_create","no_destroy"}) {
      Case c(work,variant,vendor,fixtures/(std::string("fixture_")+variant+".dll"));
      Refused(c,"private adapter rejected presentation binding",1,"attach\ndetach\n");
    }
    {Case c(work,"binding-rejected",vendor,good);c.Mode(L"reject-bind");Refused(c,"private adapter rejected presentation binding",1,entered+"detach\n");}
    for(const wchar_t* mode : {L"status-return",L"status-size",L"status-version",L"status-reserved",L"status-reason",L"status-selected",L"status-unknown"}) {
      Case c(work,wxString(mode).ToStdString(),vendor,good);c.Mode(mode);
      Refused(c,"private adapter returned invalid pre-initialization state",1,entered+"status\ndetach\n");
    }
    {Case c(work,"bound-success",vendor,good);wxDynamicLibrary lib;
      auto r=LoadOChartsModule(lib,c.request,c.Compatible());
      Require(r.loaded && r.applicable && r.reason.empty() && lib.IsLoaded() && c.compatibility_calls==1,"qualified fake adapter did not load");
      c.Events(entered+"status\n");NeverLoadedVendor();Require(Writable(c.original),"explicit locks survived function return");
      auto second=LoadOChartsModule(lib,c.request,c.Compatible());
      Require(!second.loaded && second.reason=="destination module is already loaded" && c.compatibility_calls==1 && lib.IsLoaded(),"loaded module was replaced");
      Require(UnloadPluginModuleChecked(lib),"successful module unload");c.Events(entered+"status\ndetach\n");Pass("single bound module and lock lifetime");}
    // From here on, originals are locally built harmless fixtures, never vendor bytes.
    {Case c(work,"callback-and-fallback",vendor,good);Copy(good,c.original);
      wxDynamicLibrary lib;RegisterPluginPresentationLoader(nullptr);
      Require(!TryPluginPresentationLoader(lib,W(c.original),{}),"null callback must be inactive");
      Require(LoadPluginWithPresentationFallback(lib,W(c.original),{})==PluginModuleLoadOutcome::Loaded && lib.IsLoaded(),"actual null-hook original fallback");
      Require(LoadPluginWithPresentationFallback(lib,W(c.dir/"missing.dll"),{})==PluginModuleLoadOutcome::LoadFailed && !lib.IsLoaded(),"failed original load after prior module cleanup");
      c.Events("attach\ndetach\n");
      fs::remove(c.events);fallback_adapter=c.adapter;callback_calls=0;
      RegisterPluginPresentationLoader(CallbackLoad);
      Require(LoadPluginWithPresentationFallback(lib,W(c.original),[](const wxString&){return true;})==PluginModuleLoadOutcome::Loaded,"forwarded callback load");
      Require(callback_calls==1 && lib.IsLoaded(),"forwarded callback/module count");
      Require(UnloadPluginModuleChecked(lib),"benign callback unload");c.Events("attach\ndetach\n");
      fs::remove(c.events);RegisterPluginPresentationLoader(CallbackCleanRefusal);
      Require(LoadPluginWithPresentationFallback(lib,W(c.original),[](const wxString&){return true;})==PluginModuleLoadOutcome::Loaded,
              "clean rejected adapter did not allow original fallback");
      Require(callback_calls==2 && lib.IsLoaded(),"clean rejection forwarding count");
      c.Events("attach\ndetach\nattach\n");
      Require(UnloadPluginModuleChecked(lib),"clean fallback fixture cleanup");c.Events("attach\ndetach\nattach\ndetach\n");
      fs::remove(c.events);RegisterPluginPresentationLoader(CallbackResidue);
      Require(LoadPluginWithPresentationFallback(lib,W(c.original),[](const wxString&){return true;})==PluginModuleLoadOutcome::UnloadFailed,"residual module did not block fallback");
      Require(callback_calls==3 && lib.IsLoaded(),"residue ownership lost");c.Events("attach\n");
      Require(UnloadPluginModuleChecked(lib),"residual benign module cleanup");c.Events("attach\ndetach\n");
      RegisterPluginPresentationLoader(nullptr);Pass("actual null/forwarded callback and residue-safe fallback");}
    {Case c(work,"injected-invalid-module-handle",vendor,good);Copy(good,c.original);
      wxDynamicLibrary lib;
      void* reserved=VirtualAlloc(nullptr,65536,MEM_RESERVE,PAGE_NOACCESS);
      Require(reserved!=nullptr,"reserve non-module address for fault injection");
      struct Cleanup {
        wxDynamicLibrary& library; void* address;
        ~Cleanup() {
          const auto handle=library.Detach();
          if(handle && handle!=reinterpret_cast<wxDllType>(address)) FreeLibrary(handle);
          VirtualFree(address,0,MEM_RELEASE);
        }
      } cleanup{lib,reserved};
      const auto invalid=reinterpret_cast<wxDllType>(reserved);lib.Attach(invalid);
      SetLastError(ERROR_SUCCESS);
      Require(!UnloadPluginModuleChecked(lib),"injected invalid HMODULE unexpectedly unloaded");
      const auto error=GetLastError();
      Require(lib.IsLoaded() && lib.GetLibHandle()==invalid,"failed unload lost ownership");
      Require(LoadPluginWithPresentationFallback(lib,W(c.original),{})==PluginModuleLoadOutcome::UnloadFailed,
              "injected unload refusal reached original fallback");
      Require(lib.GetLibHandle()==invalid,"fallback refusal lost invalid-handle ownership");
      c.Events("");
      std::cout<<"Injected invalid-handle FreeLibrary error: "<<error<<"\n";
      Pass("injected invalid-handle refusal retains ownership");
    }
    NeverLoadedVendor();Require(Hash(vendor)==OriginalOChartsSha256,"vendor input changed");
    std::ofstream report(argv[6]);report<<"{\"status\":\"passed\",\"checks\":"<<passed<<",\"vendorExecuted\":false,\"pluginFactoryExecuted\":false,\"invalidHandleFaultInjected\":true,\"productAcceptance\":false}\n";
    std::cout<<"PASS "<<passed<<" native loader groups; no plugin factory/helper execution\n";return 0;
  } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<"\n";return 1;}
}

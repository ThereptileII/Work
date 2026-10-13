#include "integration/OChartsModuleLoader.h"
#include "integration/PluginPresentationFallback.h"
#include "picosha2.h"
#include <wx/init.h>
#include <wx/log.h>
#include <windows.h>
#include <cwchar>
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
bool SharingDenied(const fs::path& path,DWORD access) {
  HANDLE handle=CreateFileW(path.c_str(),access,
      FILE_SHARE_READ|FILE_SHARE_WRITE|FILE_SHARE_DELETE,nullptr,OPEN_EXISTING,
      FILE_ATTRIBUTE_NORMAL,nullptr);
  if(handle!=INVALID_HANDLE_VALUE) {CloseHandle(handle);return false;}
  return GetLastError()==ERROR_SHARING_VIOLATION;
}
BY_HANDLE_FILE_INFORMATION FileIdentity(const fs::path& path) {
  HANDLE handle=CreateFileW(path.c_str(),GENERIC_READ,FILE_SHARE_READ,nullptr,
      OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
  Require(handle!=INVALID_HANDLE_VALUE,"open mapped/inspected file identity");
  BY_HANDLE_FILE_INFORMATION identity{};
  const bool valid=GetFileInformationByHandle(handle,&identity)!=0;
  CloseHandle(handle);Require(valid,"read mapped/inspected file identity");
  return identity;
}
bool SameFile(const BY_HANDLE_FILE_INFORMATION& a,const BY_HANDLE_FILE_INFORMATION& b) {
  return a.dwVolumeSerialNumber==b.dwVolumeSerialNumber &&
         a.nFileIndexHigh==b.nFileIndexHigh && a.nFileIndexLow==b.nFileIndexLow;
}
std::wstring CurrentDllDirectory() {
  wchar_t directory[32768]{};
  SetLastError(ERROR_SUCCESS);
  const auto length=GetDllDirectoryW(32768,directory);
  const auto error=GetLastError();
  Require(length<32768,"process DLL directory exceeds bounded query buffer");
  Require(length!=0 || error==ERROR_SUCCESS,
          "cannot read process DLL directory; Win32 error="+std::to_string(error));
  Require(directory[length]==L'\0',"process DLL directory query is not terminated");
  return std::wstring(directory,length);
}
// Change only this disposable process's current-directory search location.
// Unlike SetDllDirectory, current_path has an exactly recoverable prior state.
struct ScopedSearchDirectory {
  fs::path previous=fs::current_path();
  std::wstring dll_directory=CurrentDllDirectory();
  bool restored=false;
  explicit ScopedSearchDirectory(const fs::path& directory) {
    // A zero-capacity query reports required size, not actual path length.
    // Read/log the string; leave the DLL-directory setting entirely untouched.
    std::cout<<"First-map process DLL directory: '"<<fs::path(dll_directory).u8string()
             <<"'; characters="<<dll_directory.size()<<"\n";
    Require(dll_directory.empty(),"unexpected nonempty process DLL search override: "+
            fs::path(dll_directory).u8string());
    fs::current_path(directory);
  }
  ~ScopedSearchDirectory() {
    if(!restored) SetCurrentDirectoryW(previous.c_str());
  }
  void Restore() {
    fs::current_path(previous);
    Require(fs::current_path()==previous,"process search directory was not restored");
    Require(CurrentDllDirectory()==dll_directory,"process DLL directory changed during first-map control");
    restored=true;
  }
};
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
      std::thread worker([&]{PluginModuleInspectionCustody custody; wxDynamicLibrary lib;r=LoadOChartsModule(lib,c.request,c.Compatible());});worker.join();
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
    {Case c(work,"compatibility-refused",vendor,good);PluginModuleInspectionCustody custody; wxDynamicLibrary lib;int calls=0;
      auto r=LoadOChartsModule(lib,c.request,[&](const wxString& p){++calls;Require(p==c.request.adapter,"compatibility path");return false;});
      Require(!r.loaded && r.reason=="private adapter ABI is incompatible" && calls==1 && !lib.IsLoaded(),"compatibility refusal");c.Events("");Pass("compatibility refused");}
    {Case c(work,"compatibility-missing",vendor,good);PluginModuleInspectionCustody custody; wxDynamicLibrary lib;
      auto r=LoadOChartsModule(lib,c.request,{});Require(!r.loaded && r.reason=="private adapter ABI is incompatible" && !lib.IsLoaded(),"missing compatibility callback");c.Events("");Pass("compatibility missing");}
    {Case c(work,"invalid-pe-after-exact-hash",vendor,good);{std::ofstream f(c.adapter,std::ios::binary|std::ios::trunc);f<<"not a DLL";}
      c.request.adapter_sha256=Hash(c.adapter);c.request.adapter_bytes=fs::file_size(c.adapter);
      PluginModuleInspectionCustody custody; wxDynamicLibrary lib;auto r=LoadOChartsModule(lib,c.request,[](const wxString&){return true;});
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
    {Case c(work,"bound-success",vendor,good);PluginModuleInspectionCustody custody; wxDynamicLibrary lib;
      auto r=LoadOChartsModule(lib,c.request,c.Compatible());
      Require(r.loaded && r.applicable && r.reason.empty() && lib.IsLoaded() && c.compatibility_calls==1,"qualified fake adapter did not load");
      c.Events(entered+"status\n");NeverLoadedVendor();Require(Writable(c.original),"explicit locks survived function return");
      auto second=LoadOChartsModule(lib,c.request,c.Compatible());
      Require(!second.loaded && second.reason=="destination module is already loaded" && c.compatibility_calls==1 && lib.IsLoaded(),"loaded module was replaced");
      Require(UnloadPluginModuleChecked(lib,custody),"successful module unload");c.Events(entered+"status\ndetach\n");Pass("single bound module and lock lifetime");}
    // From here on, originals are locally built harmless fixtures, never vendor bytes.
    {Case c(work,"callback-and-fallback",vendor,good);Copy(good,c.original);
      PluginModuleInspectionCustody custody; wxDynamicLibrary lib;RegisterPluginPresentationLoader(nullptr);
      Require(!TryPluginPresentationLoader(lib,W(c.original),{}),"null callback must be inactive");
      Require(LoadPluginWithPresentationFallback(lib,W(c.original),{},custody)==PluginModuleLoadOutcome::Loaded && lib.IsLoaded(),"actual null-hook original fallback");
      Require(LoadPluginWithPresentationFallback(lib,W(c.dir/"missing.dll"),{},custody)==PluginModuleLoadOutcome::LoadFailed && !lib.IsLoaded(),"failed original load after prior module cleanup");
      c.Events("attach\ndetach\n");
      fs::remove(c.events);fallback_adapter=c.adapter;callback_calls=0;
      RegisterPluginPresentationLoader(CallbackLoad);
      Require(LoadPluginWithPresentationFallback(lib,W(c.original),[](const wxString&){return true;},custody)==PluginModuleLoadOutcome::Loaded,"forwarded callback load");
      Require(callback_calls==1 && lib.IsLoaded(),"forwarded callback/module count");
      Require(UnloadPluginModuleChecked(lib,custody),"benign callback unload");c.Events("attach\ndetach\n");
      fs::remove(c.events);RegisterPluginPresentationLoader(CallbackCleanRefusal);
      Require(LoadPluginWithPresentationFallback(lib,W(c.original),[](const wxString&){return true;},custody)==PluginModuleLoadOutcome::Loaded,
              "clean rejected adapter did not allow original fallback");
      Require(callback_calls==2 && lib.IsLoaded(),"clean rejection forwarding count");
      c.Events("attach\ndetach\nattach\n");
      Require(UnloadPluginModuleChecked(lib,custody),"clean fallback fixture cleanup");c.Events("attach\ndetach\nattach\ndetach\n");
      fs::remove(c.events);RegisterPluginPresentationLoader(CallbackResidue);
      Require(LoadPluginWithPresentationFallback(lib,W(c.original),[](const wxString&){return true;},custody)==PluginModuleLoadOutcome::UnloadFailed,"residual module did not block fallback");
      Require(callback_calls==3 && lib.IsLoaded(),"residue ownership lost");c.Events("attach\n");
      Require(UnloadPluginModuleChecked(lib,custody),"residual benign module cleanup");c.Events("attach\ndetach\n");
      RegisterPluginPresentationLoader(nullptr);Pass("actual null/forwarded callback and residue-safe fallback");}
    // These DLLs are locally built inert controls, with one real import from a
    // locally built libeay32.dll stub. Verify the guard, not a missing-DLL error.
    for (const auto* kind : {"legacy_normal", "legacy_delay"}) {
      Case c(work,kind,vendor,good);
      Copy(fixtures/(std::string("fixture_")+kind+".dll"),c.original);
      Copy(fixtures/"libeay32.dll",c.original.parent_path()/"libeay32.dll");
      const auto before=Hash(c.original);
      const auto raw=Read(c.original);
      const std::vector<unsigned char> bytes(raw.begin(),raw.end());
      const auto policy=RefuseLegacyPluginTlsImports(bytes);
      Require(policy.find(std::string(kind)=="legacy_delay" ? "delay import" : "TLS import")!=std::string::npos,
              "fixture lacks intended native import metadata");
      PluginModuleInspectionCustody custody; wxDynamicLibrary lib;RegisterPluginPresentationLoader(nullptr);
      // Control: these exact harmless bytes are loadable without the guard.
      // Explicitly preload the stub because normal LoadLibrary does not add the
      // original's directory to the dependency search path.
      wxDynamicLibrary dependency;Require(dependency.Load(W(c.original.parent_path()/"libeay32.dll")),"stub loads");
      Require(lib.Load(W(c.original)),"unguarded control loads");
      Require(UnloadPluginModuleChecked(lib,custody),"unguarded control unloads");
      c.Events("attach\ndetach\n");fs::remove(c.events);
      for (int attempt=0;attempt<2;++attempt) {
        Require(LoadPluginWithPresentationFallback(lib,W(c.original),{},custody)==PluginModuleLoadOutcome::LoadFailed,
                "legacy import crossed original load boundary");
        Require(!lib.IsLoaded(),"refused original is mapped");c.Events("");
      }
      fallback_adapter=c.adapter;RegisterPluginPresentationLoader(CallbackCleanRefusal);
      Require(LoadPluginWithPresentationFallback(lib,W(c.original),[](const wxString&){return true;},custody)==PluginModuleLoadOutcome::LoadFailed,
              "clean adapter refusal bypassed TLS guard");
      Require(!lib.IsLoaded(),"legacy original mapped after adapter refusal");
      c.Events("attach\ndetach\n");
      Require(Hash(c.original)==before && Writable(c.original),"refusal changed original or leaked file lock");
      RegisterPluginPresentationLoader(nullptr);Pass(kind);
    }
    // The entry DLL itself has no denied TLS import. Only its locally built
    // sibling dependency imports the harmless legacy-name stub.
    for (const auto* kind : {"indirect_legacy", "indirect_clean"}) {
      const bool denied=std::string(kind)=="indirect_legacy";
      Case c(work,kind,vendor,good);
      Copy(fixtures/(std::string("fixture_")+kind+".dll"),c.original);
      const auto helper=c.original.parent_path()/(denied ? "helper_legacy.dll" : "helper_clean.dll");
      Copy(fixtures/helper.filename(),helper);
      if(denied) Copy(fixtures/"libeay32.dll",c.original.parent_path()/"libeay32.dll");
      const auto before=Hash(c.original),helper_before=Hash(helper);
      const auto raw=Read(c.original);
      const std::vector<unsigned char> bytes(raw.begin(),raw.end());
      Require(RefuseLegacyPluginTlsImports(bytes).empty(),"entry DLL has a direct TLS import");
      PluginModuleInspectionCustody custody; wxDynamicLibrary lib;RegisterPluginPresentationLoader(nullptr);
      if(denied) {
        Require(LoadPluginWithPresentationFallback(lib,W(c.original),{},custody)==PluginModuleLoadOutcome::LoadFailed,
                "indirect legacy import crossed original load boundary");
        Require(!lib.IsLoaded(),"indirect legacy original is mapped");c.Events("");
      }
      // Windows may resolve an already loaded dependency before searching the
      // original's directory. Pin only these inert local controls for a
      // loadability comparison; the production guard still reads disk bytes.
      wxDynamicLibrary stub;
      if(denied) Require(stub.Load(W(c.original.parent_path()/"libeay32.dll")),"indirect stub loads");
      wxDynamicLibrary dependency;Require(dependency.Load(W(helper)),"indirect helper loads");
      if(denied) {
        Require(lib.Load(W(c.original)),"unguarded indirect control loads");
        Require(UnloadPluginModuleChecked(lib,custody),"unguarded indirect control unloads");
        c.Events("attach\ndetach\n");fs::remove(c.events);
        Require(LoadPluginWithPresentationFallback(lib,W(c.original),{},custody)==PluginModuleLoadOutcome::LoadFailed,
                "mapped indirect legacy helper bypassed disk guard");
        Require(!lib.IsLoaded(),"indirect legacy original mapped after refusal");c.Events("");
      } else {
        Require(LoadPluginWithPresentationFallback(lib,W(c.original),{},custody)==PluginModuleLoadOutcome::Loaded,
                "benign local dependency refused or failed to load");
        c.Events("attach\n");
        Require(UnloadPluginModuleChecked(lib,custody),"benign indirect control unloads");
        c.Events("attach\ndetach\n");
      }
      Require(UnloadPluginModuleChecked(dependency),"indirect helper cleanup");
      if(denied) Require(UnloadPluginModuleChecked(stub),"indirect stub cleanup");
      Require(Hash(c.original)==before && Hash(helper)==helper_before &&
                  Writable(c.original) && Writable(helper),"indirect guard changed input or leaked lock");
      Pass(kind);
    }
    // First mapping of a unique benign sibling while the actual production
    // guard owns the inspected original and sibling handles. This direct class
    // control complements (and does not replace) the fallback groups above.
    {Case c(work,"unmapped-sibling-under-inspection-lock",vendor,good);
      Copy(fixtures/"fixture_indirect_unmapped.dll",c.original);
      const auto helper=c.original.parent_path()/"helper_unmapped.dll";
      Copy(fixtures/"unmapped/helper_unmapped.dll",helper);
      wchar_t executable_path[32768]{};
      const auto executable_count=GetModuleFileNameW(nullptr,executable_path,32768);
      Require(executable_count && executable_count<32768,"cannot read executable directory");
      Require(!fs::exists(fs::path(executable_path).parent_path()/"helper_unmapped.dll"),
              "first-map helper must have no application-directory duplicate");
      Require(GetModuleHandleW(L"helper_unmapped.dll")==nullptr,"first-map helper was preloaded");
      const auto original_before=Hash(c.original),helper_before=Hash(helper);
      ScopedSearchDirectory search(c.original.parent_path());
      PluginModuleInspectionCustody custody; wxDynamicLibrary lib;
      {
        OriginalPluginTlsGuard guard(W(c.original));
        Require(guard.Refusal().empty(),"first-map benign graph refused: "+guard.Refusal());
        for(const auto& path : {c.original,helper})
          Require(SharingDenied(path,GENERIC_WRITE) && SharingDenied(path,DELETE),
                  "inspection did not hold original/sibling write-delete locks");
        const auto inspected=FileIdentity(helper);
        Require(GetModuleHandleW(L"helper_unmapped.dll")==nullptr,
                "inspection unexpectedly mapped benign sibling");
        Require(lib.Load(W(c.original)),"first sibling mapping under inspection handles failed");
        const auto mapped=GetModuleHandleW(L"helper_unmapped.dll");
        Require(mapped!=nullptr,"original load did not map expected sibling");
        wchar_t mapped_path[32768]{};
        const auto count=GetModuleFileNameW(mapped,mapped_path,32768);
        Require(count && count<32768,"cannot read mapped sibling path");
        const fs::path actual(mapped_path);
        Require(_wcsicmp(actual.lexically_normal().c_str(),helper.lexically_normal().c_str())==0,
                "first-map sibling resolved outside inspected directory");
        Require(SameFile(inspected,FileIdentity(actual)) && Hash(actual)==helper_before,
                "mapped sibling identity differs from inspected locked file");
        for(const auto& path : {c.original,helper})
          Require(SharingDenied(path,GENERIC_WRITE) && SharingDenied(path,DELETE),
                  "inspection lock disappeared during first mapping");
        c.Events("attach\n");
        std::cout<<"First-map sibling path: "<<actual.u8string()
                 <<"; volume="<<inspected.dwVolumeSerialNumber
                 <<"; file-index="<<inspected.nFileIndexHigh<<":"<<inspected.nFileIndexLow
                 <<"; sha256="<<helper_before<<"\n";
        Require(UnloadPluginModuleChecked(lib,custody),"first-map original unload");
        Require(GetModuleHandleW(L"helper_unmapped.dll")==nullptr,"first-map helper leaked after original unload");
        c.Events("attach\ndetach\n");
      }
      Require(Writable(c.original) && Writable(helper) && Hash(c.original)==original_before &&
                  Hash(helper)==helper_before,"first-map inputs changed or inspection handles leaked");
      search.Restore();
      Pass("previously unmapped sibling maps with inspected path/identity under live guard");
    }
    // Native refusal before DllMain, covering the historical application-only
    // hole, a clean sibling masking the unsafe host candidate, and a chain
    // alternating plugin and application roots. No production search override.
    for (const char* mode : {"host-only", "clean-sibling", "cross-root"}) {
      const bool cross=std::string(mode)=="cross-root";
      Case c(work,std::string("application-directory-refusal-")+mode,vendor,good);
      Copy(fixtures/(cross ? "fixture_indirect_cross.dll" : "fixture_indirect_external.dll"),c.original);
      wchar_t executable_path[32768]{};
      const auto executable_count=GetModuleFileNameW(nullptr,executable_path,32768);
      Require(executable_count && executable_count<32768,"host refusal executable path");
      const auto application=fs::path(executable_path).parent_path();
      const auto helper=application/(cross ? "helper_cross_host.dll" : "helper_external.dll");
      const auto stub=application/"libeay32.dll";
      Require(fs::equivalent(application,fixtures),"host refusal fixture directory mismatch");
      Require(fs::exists(helper) && fs::exists(stub),"inert host dependencies missing");
      Require(!fs::exists(c.original.parent_path()/helper.filename()) &&
                  !fs::exists(c.original.parent_path()/stub.filename()),"host dependency must not be a sibling");
      fs::path sibling;
      if(std::string(mode)=="clean-sibling") {
        sibling=c.original.parent_path()/"helper_external.dll";
        Copy(fixtures/"helper_clean.dll",sibling);
      } else if(cross) {
        sibling=c.original.parent_path()/"helper_cross.dll";
        Copy(fixtures/"crossroot/helper_cross.dll",sibling);
        Require(!fs::exists(application/sibling.filename()),"cross-root first helper must be sibling-only");
      }
      Require(GetModuleHandleW(helper.filename().c_str())==nullptr &&
                  GetModuleHandleW(stub.filename().c_str())==nullptr,
              "refusal dependencies were already mapped");
      const auto dll_directory=CurrentDllDirectory();
      const auto working_directory=fs::current_path();
      Require(dll_directory.empty(),"host refusal requires unchanged default DLL search");
      const auto original_before=Hash(c.original),helper_before=Hash(helper),stub_before=Hash(stub);
      const auto sibling_before=sibling.empty() ? std::string{} : Hash(sibling);
      {
        OriginalPluginTlsGuard guard(W(c.original));
        Require(guard.Refusal().find("Unsupported legacy TLS import: libeay32.dll")!=std::string::npos,
                "host legacy graph was not refused: "+guard.Refusal());
        for(const auto& locked : {c.original,helper})
          Require(SharingDenied(locked,GENERIC_WRITE) && SharingDenied(locked,DELETE),
                  "host inspection did not retain original/helper custody");
        if(!sibling.empty())
          Require(SharingDenied(sibling,GENERIC_WRITE) && SharingDenied(sibling,DELETE),
                  "cross-root/duplicate sibling was not retained");
        c.Events("");
      }
      RegisterPluginPresentationLoader(nullptr);
      PluginModuleInspectionCustody custody; wxDynamicLibrary lib;
      Require(LoadPluginWithPresentationFallback(lib,W(c.original),{},custody)==PluginModuleLoadOutcome::LoadFailed,
              "unsafe host graph reached original load");
      Require(!lib.IsLoaded() && !custody,"refused original load retained module/custody");
      Require(GetModuleHandleW(helper.filename().c_str())==nullptr &&
                  GetModuleHandleW(stub.filename().c_str())==nullptr,
              "host refusal allowed dependency initialization");
      c.Events("");
      Require(Writable(c.original) && Writable(helper) && Hash(c.original)==original_before &&
                  Hash(helper)==helper_before && Hash(stub)==stub_before,
              "host refusal changed bytes or leaked lock");
      if(!sibling.empty())
        Require(Writable(sibling) && Hash(sibling)==sibling_before,"sibling changed or lock leaked");
      Require(fs::current_path()==working_directory && CurrentDllDirectory()==dll_directory,
              "host refusal changed process search state");
      NeverLoadedVendor();
      Pass((std::string("application-directory legacy refusal before initialization: ")+mode).c_str());
    }
    // Compatible first mapping from the application directory while the exact
    // production fallback owns both inspected files through unload.
    {Case c(work,"benign-application-directory-first-mapping",vendor,good);
      Copy(fixtures/"fixture_indirect_clean.dll",c.original);
      const auto helper=fixtures/"helper_clean.dll";
      Require(!fs::exists(c.original.parent_path()/helper.filename()),"benign helper must be host-only");
      Require(GetModuleHandleW(helper.filename().c_str())==nullptr,"benign host helper already mapped");
      const auto before=Hash(c.original),helper_before=Hash(helper);
      const auto expected=FileIdentity(helper);
      const auto working_directory=fs::current_path();
      const auto dll_directory=CurrentDllDirectory();
      PluginModuleInspectionCustody custody; wxDynamicLibrary lib;
      RegisterPluginPresentationLoader(nullptr);
      Require(LoadPluginWithPresentationFallback(lib,W(c.original),{},custody)==PluginModuleLoadOutcome::Loaded,
              "benign host-only graph refused");
      Require(custody!=nullptr,"benign host graph missing inspection owner");
      for(const auto& locked : {c.original,helper})
        Require(SharingDenied(locked,GENERIC_WRITE) && SharingDenied(locked,DELETE),
                "benign host graph lock did not survive load");
      const auto mapped=GetModuleHandleW(helper.filename().c_str());
      Require(mapped!=nullptr,"benign host dependency did not first-map");
      wchar_t mapped_path[32768]{};
      const auto count=GetModuleFileNameW(mapped,mapped_path,32768);
      Require(count && count<32768,"benign mapped host path unavailable");
      const fs::path actual(mapped_path);
      Require(_wcsicmp(actual.lexically_normal().c_str(),helper.lexically_normal().c_str())==0 &&
                  SameFile(expected,FileIdentity(actual)) && Hash(actual)==helper_before,
              "benign host module differs from inspected locked identity");
      const auto probe=lib.GetSymbol("exercise_indirect_import");
      Require(probe && reinterpret_cast<int (__cdecl*)()>(probe)()==209,"benign host helper execution");
      c.Events("attach\n");
      Require(UnloadPluginModuleChecked(lib,custody) && !custody,"benign host original unload");
      Require(GetModuleHandleW(helper.filename().c_str())==nullptr,"benign host helper leaked after unload");
      c.Events("attach\ndetach\n");
      Require(Writable(c.original) && Writable(helper) && Hash(c.original)==before && Hash(helper)==helper_before,
              "benign host graph changed bytes or leaked inspection locks");
      Require(fs::current_path()==working_directory && CurrentDllDirectory()==dll_directory,
              "benign host mapping changed process search state");
      NeverLoadedVendor();
      Pass("benign application-directory first mapping matches retained identity through unload");
    }
    // Native export-only dependencies: no normal/delay descriptor names the
    // forwarded module. Resolve the inert export, never a plugin factory.
    const struct ForwardCase {
      const char* kind; const char* target; bool denied; bool ambiguous;
    } forward_cases[] = {
      {"forward_legacy_name","libeay32.legacy_tls_fixture",true,false},
      {"forward_legacy_ordinal","libeay32.#209",true,false},
      {"forward_ssleay_ordinal","ssleay32.#209",true,false},
      {"forward_indirect","forward_helper.forwarded_probe",true,false},
      {"forward_clean_name","forward_clean.fixture_value",false,false},
      {"forward_clean_ordinal","forward_clean.#209",false,false},
      {"forward_explicit_dll","forward_clean.dll.fixture_value",false,true},
      {"forward_multi_dot","forward.clean.fixture_value",false,true}
    };
    for (const auto& test : forward_cases) {
      Case c(work,test.kind,vendor,good);
      Copy(fixtures/"forwarders"/(std::string("fixture_")+test.kind+".dll"),c.original);
      const auto directory=c.original.parent_path();
      const bool indirect=std::string(test.kind)=="forward_indirect";
      const bool ssleay=std::string(test.kind)=="forward_ssleay_ordinal";
      const bool multi=std::string(test.kind)=="forward_multi_dot";
      std::vector<fs::path> siblings;
      if(test.denied) {
        const auto stub=directory/(ssleay ? "ssleay32.dll" : "libeay32.dll");
        Copy(ssleay ? fixtures/"forwarders/ssleay32.dll" : fixtures/"libeay32.dll",stub);
        siblings.push_back(stub);
        if(indirect) {const auto helper=directory/"forward_helper.dll";
          Copy(fixtures/"forwarders/forward_helper.dll",helper);siblings.push_back(helper);}
      } else if(multi) {
        for(const auto* name : {"forward.dll","forward.clean.dll"}) {
          const auto path=directory/name;Copy(fixtures/"forwarders"/name,path);siblings.push_back(path);
        }
        // Also prepare the exact extension-bearing module spelling. Native
        // path/identity evidence below decides which harmless alternative loads.
        const auto exact=directory/"forward.clean";
        Copy(fixtures/"forwarders/forward.clean.dll",exact);siblings.push_back(exact);
      } else {
        const auto helper=directory/"forward_clean.dll";
        Copy(fixtures/"forwarders/forward_clean.dll",helper);siblings.push_back(helper);
      }
      const auto raw=Read(c.original);
      const std::vector<unsigned char> bytes(raw.begin(),raw.end());
      const auto declared=ReadWin32PeImports(bytes);
      Require(declared.parsed && declared.forwarders==std::vector<std::string>{test.target},
              "MSVC fixture did not emit the exact intended forwarder string");
      for(const auto& item : declared.modules)
        Require(item.name!="libeay32.dll" && item.name!="ssleay32.dll" &&
                    item.name.find("forward")==std::string::npos,
                "forwarder fixture unexpectedly has an ordinary dependency import");
      const auto original_before=Hash(c.original);
      std::vector<std::string> sibling_hashes;
      for(const auto& path : siblings) sibling_hashes.push_back(Hash(path));
      ScopedSearchDirectory search(directory);
      PluginModuleInspectionCustody custody; wxDynamicLibrary lib;
      RegisterPluginPresentationLoader(nullptr);
      for(const auto& path : siblings)
        Require(GetModuleHandleW(path.filename().c_str())==nullptr,"forwarder sibling preloaded before guard");
      if(test.denied || test.ambiguous) {
        {OriginalPluginTlsGuard guard(W(c.original));
          Require(guard.Refusal().find(test.ambiguous ? "Unsupported plugin export forwarder grammar" :
                      "TLS export forwarder")!=std::string::npos,"wrong forwarded-graph refusal");
          Require(SharingDenied(c.original,GENERIC_WRITE) && SharingDenied(c.original,DELETE),
                  "forwarder refusal did not hold inspected original");
          if(indirect) Require(SharingDenied(siblings.back(),GENERIC_WRITE) &&
                                  SharingDenied(siblings.back(),DELETE),"forwarded sibling was not held");
          c.Events("");
        }
        for(int attempt=0;attempt<2;++attempt) {
          Require(LoadPluginWithPresentationFallback(lib,W(c.original),{},custody)==PluginModuleLoadOutcome::LoadFailed,
                  "declared forwarder crossed original load boundary");
          Require(!lib.IsLoaded(),"refused forwarder original is mapped");c.Events("");
          for(const auto& path : siblings)
            Require(GetModuleHandleW(path.filename().c_str())==nullptr,"refused forwarded sibling reached DllMain");
        }
        fallback_adapter=c.adapter;RegisterPluginPresentationLoader(CallbackCleanRefusal);
        Require(LoadPluginWithPresentationFallback(lib,W(c.original),[](const wxString&){return true;},custody)==PluginModuleLoadOutcome::LoadFailed,
                "clean adapter refusal bypassed forwarder guard");
        Require(!lib.IsLoaded(),"forwarder original mapped after adapter refusal");
        c.Events("attach\ndetach\n");fs::remove(c.events);RegisterPluginPresentationLoader(nullptr);
      }
      // Direct denied-basename controls explicitly preload their private inert
      // stub, avoiding the earlier fixtures' application-directory duplicate.
      wxDynamicLibrary stub;
      if(test.denied) Require(stub.Load(W(siblings.front())),"private inert forwarded TLS stub load");
      const auto exercise = [&](bool guarded) {
        std::vector<BY_HANDLE_FILE_INFORMATION> inspected;
        for(const auto& path : siblings) inspected.push_back(FileIdentity(path));
        if(guarded) {
          Require(LoadPluginWithPresentationFallback(lib,W(c.original),{},custody)==PluginModuleLoadOutcome::Loaded,
                  "actual benign forwarder fallback load");
          Require(static_cast<bool>(custody),"actual fallback did not retain import inspection");
          for(const auto& path : {c.original,siblings.front()})
            Require(SharingDenied(path,GENERIC_WRITE) && SharingDenied(path,DELETE),
                    "actual fallback released a not-yet-mapped forwarded file");
          Require(GetModuleHandleW(siblings.front().filename().c_str())==nullptr,
                  "benign forwarded dependency mapped before export lookup");
        } else Require(lib.Load(W(c.original)),"harmless unguarded forwarder original load");
        c.Events("attach\n");
        SetLastError(ERROR_SUCCESS);
        const auto proc=GetProcAddress(reinterpret_cast<HMODULE>(lib.GetLibHandle()),"forwarded_probe");
        const auto resolution_error=GetLastError();
        if(proc==nullptr) {
          std::string prepared;
          for(const auto& path : siblings) {
            if(!prepared.empty()) prepared+=", ";
            prepared+=path.filename().u8string();
          }
          Require(false,"native forwarded export resolution failed; target="+std::string(test.target)+
                  "; Win32 error="+std::to_string(resolution_error)+"; prepared filenames="+prepared);
        }
        HMODULE owner=nullptr;
        Require(GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                    reinterpret_cast<LPCWSTR>(proc),&owner)!=0,"native forwarder target owner");
        wchar_t mapped_path[32768]{};const auto n=GetModuleFileNameW(owner,mapped_path,32768);
        Require(n && n<32768,"native forwarder mapped path");
        const fs::path actual(mapped_path);
        const auto mapped_identity=FileIdentity(actual);
        bool matched=false;
        for(std::size_t i=0;i<siblings.size();++i)
          if(_wcsicmp(actual.lexically_normal().c_str(),siblings[i].lexically_normal().c_str())==0)
            matched=SameFile(mapped_identity,inspected[i]) && Hash(actual)==sibling_hashes[i];
        Require(matched,"forwarded target resolved outside exact private sibling identity");
        const auto value=reinterpret_cast<int (__cdecl*)()>(proc)();
        if(test.ambiguous)
          Require(multi ? (value==310 || value==409) : (value==209 || value==210),
                  "ambiguous native probe selected no prepared harmless symbol");
        else Require(value==209,"native simple named/ordinal forwarder returned wrong fixture value");
        std::cout<<"Native forwarder target: "<<test.target<<" -> "<<actual.u8string()
                 <<"; value="<<value<<"; volume="<<mapped_identity.dwVolumeSerialNumber
                 <<"; file-index="<<mapped_identity.nFileIndexHigh<<":"<<mapped_identity.nFileIndexLow
                 <<"; sha256="<<Hash(actual)<<"; guarded="<<guarded<<"\n";
        if(guarded) {
          for(const auto& path : {c.original,siblings.front()})
            Require(SharingDenied(path,GENERIC_WRITE) && SharingDenied(path,DELETE),
                    "forwarder inspection lock disappeared during resolution");
        }
        const bool marked=!test.denied || indirect;
        c.Events(marked ? "attach\nforward-target-attach\n" : "attach\n");
        Require(UnloadPluginModuleChecked(lib,custody),"native forwarded original unload");
        c.Events(marked ? "attach\nforward-target-attach\ndetach\nforward-target-detach\n" : "attach\ndetach\n");
      };
      if(!test.denied && !test.ambiguous) {
        exercise(true);  // Production fallback owns custody; no manually outer guard.
        Require(!custody,"checked unload retained completed import custody");
      } else exercise(false);  // Loadability/grammar comparison only; no policy relaxation.
      if(test.denied) Require(UnloadPluginModuleChecked(stub),"private forwarded stub cleanup");
      for(std::size_t i=0;i<siblings.size();++i) {
        Require(GetModuleHandleW(siblings[i].filename().c_str())==nullptr,"forwarded sibling remained mapped");
        Require(Writable(siblings[i]) && Hash(siblings[i])==sibling_hashes[i],"forwarded sibling changed or lock leaked");
      }
      Require(Writable(c.original) && Hash(c.original)==original_before,"forwarded original changed or lock leaked");
      search.Restore();Pass(test.kind);
    }
    {Case c(work,"injected-invalid-module-handle",vendor,good);Copy(good,c.original);
      PluginModuleInspectionCustody custody; wxDynamicLibrary lib;
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
      custody=std::make_unique<OriginalPluginTlsGuard>(W(c.original));
      Require(custody->Refusal().empty(),"invalid-handle control guard unavailable");
      const auto invalid=reinterpret_cast<wxDllType>(reserved);lib.Attach(invalid);
      SetLastError(ERROR_SUCCESS);
      Require(!UnloadPluginModuleChecked(lib,custody),"injected invalid HMODULE unexpectedly unloaded");
      const auto error=GetLastError();
      Require(lib.IsLoaded() && lib.GetLibHandle()==invalid,"failed unload lost ownership");
      Require(LoadPluginWithPresentationFallback(lib,W(c.original),{},custody)==PluginModuleLoadOutcome::UnloadFailed,
              "injected unload refusal reached original fallback");
      Require(lib.GetLibHandle()==invalid,"fallback refusal lost invalid-handle ownership");
      Require(custody && SharingDenied(c.original,GENERIC_WRITE) && SharingDenied(c.original,DELETE),
              "failed unload released retained inspection custody");
      c.Events("");
      std::cout<<"Injected invalid-handle FreeLibrary error: "<<error<<"\n";
      Pass("injected invalid-handle refusal retains ownership");
    }
    NeverLoadedVendor();Require(Hash(vendor)==OriginalOChartsSha256,"vendor input changed");
    std::ofstream report(argv[6]);report<<"{\"status\":\"passed\",\"checks\":"<<passed<<",\"vendorExecuted\":false,\"pluginFactoryExecuted\":false,\"invalidHandleFaultInjected\":true,\"unmappedSiblingGuardControl\":true,\"exportForwarderControls\":true,\"forwarderGrammarProbes\":true,\"applicationDirectoryClosure\":{\"legacyRefusedBeforeInitialization\":true,\"cleanSiblingCannotMaskHost\":true,\"crossRootChainRefused\":true,\"benignFirstMappingIdentity\":true,\"custodyThroughUnload\":true},\"productAcceptance\":false}\n";
    std::cout<<"PASS "<<passed<<" native loader groups; no vendor or plugin factory execution\n";return 0;
  } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<"\n";return 1;}
}

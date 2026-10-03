#include "integration/ChartModuleCheck.h"
#include <wx/jsonval.h>
#ifdef __WXMSW__
#include "integration/ChartModulePe.h"
#include "integration/OChartsModuleLoader.h"
#include "integration/PluginPresentationFallback.h"
#include "SkagerOChartsPackage.h"
#include "XNavChartResources.h"
#include "picosha2.h"
#include <wx/ffile.h>
#include <wx/filename.h>
#include <wx/thread.h>
#include <array>
#include <windows.h>
#include <tlhelp32.h>
#endif
namespace opennav::integration {
bool CheckChartModule(const wxString& original, const wxString& install,
                      wxJSONValue& report) {
  report["contract"]=wxString("SKAGER.ChartModuleCheck.1");
  report["scope"]=wxString("DLL imports/binding/unload only; no renderer or chart acceptance");
  report["factory_called"]=false;
  report["plugin_initialized"]=false;
  report["original_dll_executed"]=false;
  report["passed"]=false;
#ifdef __WXMSW__
  // This early-only process will not enter normal application startup. Enforce
  // no child creation before any private module code, including CRT startup.
  PROCESS_MITIGATION_CHILD_PROCESS_POLICY policy{};
  policy.NoChildProcessCreation=1;
  using SetMitigation=BOOL (WINAPI *)(PROCESS_MITIGATION_POLICY,PVOID,SIZE_T);
  const auto setMitigation=reinterpret_cast<SetMitigation>(GetProcAddress(
      GetModuleHandleW(L"kernel32.dll"),"SetProcessMitigationPolicy"));
  if(!setMitigation || !setMitigation(ProcessChildProcessPolicy,&policy,sizeof(policy))) {
    report["reason"]=wxString("Could not enforce child-process prohibition");return false;
  }
  report["child_process_creation_blocked"]=true;
  const auto resources=install+"/opennav/chart-style/v1";
  for(const auto& resource:opennav::chart_style::generated::resources) {
    wxFFile file(wxFileName(resources,wxString::FromUTF8(resource.name)).GetFullPath(),"rb");
    if(!file.IsOpened() || file.Length()<0 || std::uint64_t(file.Length())!=resource.bytes) {
      report["reason"]=wxString("Compiled chart resource size differs");return false;
    }
    picosha2::hash256_one_by_one hash;std::array<unsigned char,8192> bytes{};
    auto remaining=resource.bytes;
    while(remaining) {
      const auto wanted=static_cast<std::size_t>((std::min)(std::uint64_t(remaining),std::uint64_t(bytes.size())));
      if(file.Read(bytes.data(),wanted)!=wanted) {report["reason"]=wxString("Chart resource read failed");return false;}
      hash.process(bytes.begin(),bytes.begin()+wanted);remaining-=wanted;
    }
    hash.finish();if(picosha2::get_hash_hex_string(hash)!=resource.sha256) {
      report["reason"]=wxString("Compiled chart resource hash differs");return false;
    }
  }
  report["resources_verified"]=true;
  if(GetModuleHandleW(L"o-charts_pi.dll") ||
     GetModuleHandleW(L"skager-ocharts-adapter.dll") ||
     GetModuleHandleW(L"opencpn.exe")!=GetModuleHandleW(nullptr)) {
    report["reason"]=wxString("Unexpected preloaded module or host image");return false;
  }
  OChartsModuleRequest request;
  request.original=original;
  request.adapter=wxFileName(install,"skager-ocharts-adapter.dll").GetFullPath();
  request.resources=resources;
  request.package_available=skager_ocharts::available;
  request.adapter_sha256=skager_ocharts::sha256;
  request.adapter_bytes=skager_ocharts::bytes;
  request.main_thread=wxIsMainThread();request.safe_mode=false;
  std::vector<std::string> imports;
  wxDynamicLibrary library;
  const auto result=LoadOChartsModule(library,request,[&](const wxString& path) {
    wxFFile file(path,"rb");
    if(!file.IsOpened() || file.Length()<=0 || file.Length()>128ll*1024*1024)return false;
    std::vector<unsigned char> bytes(static_cast<std::size_t>(file.Length()));
    return file.Read(bytes.data(),bytes.size())==bytes.size() && ChartModulePe(bytes,imports);
  });
  report["module_loaded"]=result.loaded;
  report["original_sha256"]=wxString::FromUTF8(OriginalOChartsSha256);
  report["adapter_sha256"]=wxString::FromUTF8(skager_ocharts::sha256);
  report["adapter_bytes"]=wxString::Format("%llu",static_cast<unsigned long long>(skager_ocharts::bytes));
  bool valid=result.loaded;
  if(result.loaded) {
    std::array<wchar_t,32768> path{};
    const auto n=GetModuleFileNameW(library.GetLibHandle(),path.data(),DWORD(path.size()));
    const wxString loaded=n && n<path.size()?wxString(path.data(),n):wxString();
    report["module_path"]=loaded;
    valid=loaded.CmpNoCase(request.adapter)==0;
    auto query=reinterpret_cast<SkagerGetChartPresentationStatusV1>(library.GetSymbol(SKAGER_CHART_STATUS_EXPORT));
    SkagerChartPresentationStatusV1 state{};state.structBytes=sizeof(state);state.version=SKAGER_CHART_BINDING_VERSION;
    const bool copied=query && query(&state)==1 && ValidOChartsStatus(state);
    report["binding_state"]=int(state.state);report["binding_reason"]=int(state.reason);
    valid=valid && copied && state.state==SKAGER_CHART_BOUND_PENDING_INITIALIZATION;
    report["imports"]=wxJSONValue(wxJSONTYPE_ARRAY);
    for(const auto& dll:imports) report["imports"].Append(wxString::FromUTF8(dll));
    report["host_imports_resolved"]=true;
    // Capture actual mapped paths while the private module is still loaded.
    // The wrapper hashes these files after exit; installed inventories alone
    // cannot establish which side-by-side runtime the loader selected.
    report["loaded_modules"]=wxJSONValue(wxJSONTYPE_ARRAY);
    const HANDLE snapshot=CreateToolhelp32Snapshot(TH32CS_SNAPMODULE,GetCurrentProcessId());
    bool observed=false;
    if(snapshot!=INVALID_HANDLE_VALUE) {
      MODULEENTRY32W module{};module.dwSize=sizeof(module);
      bool next=Module32FirstW(snapshot,&module)!=0;
      unsigned count=0;
      while(next && count<512) {
        report["loaded_modules"].Append(wxString(module.szExePath));
        ++count;next=Module32NextW(snapshot,&module)!=0;
      }
      observed=count>0 && !next && GetLastError()==ERROR_NO_MORE_FILES;
      CloseHandle(snapshot);
    }
    report["loaded_modules_observed"]=observed;
    valid=valid && observed;
  } else report["reason"]=wxString::FromUTF8(result.reason.empty()?"Module request refused":result.reason);
  const bool unloaded=UnloadPluginModuleChecked(library);
  report["unload_succeeded"]=unloaded;
  valid=valid && unloaded && !GetModuleHandleW(L"skager-ocharts-adapter.dll") && !GetModuleHandleW(L"o-charts_pi.dll");
  report["passed"]=valid;
  return valid;
#else
  (void)original;(void)install;
  report["reason"]=wxString("Native Windows required");
  return false;
#endif
}
}

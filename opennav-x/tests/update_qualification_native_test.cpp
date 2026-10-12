// Explicit disposable-CI fixture only, never registered with ordinary ctest.
// A fresh fixed profile is retained on failure/success for evidence; disposable
// runner disposal cleans it. Never adopt, modify or delete an existing profile.
#include "platform/windows/UpdateQualificationNative.h"
#include <windows.h>
#include <shlobj.h>
#include <wx/fileconf.h>
#include <wx/init.h>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void Require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
template<class F> void Refused(F action,const char* message) {
  bool refused=false;
  try { action(); } catch(const std::exception&) { refused=true; }
  Require(refused,message);
}
bool EnvironmentEquals(const wchar_t* key,const wchar_t* expected) {
  wchar_t value[128]{};
  const auto count=GetEnvironmentVariableW(key,value,128);
  return count && count<128 && std::wstring(value)==expected;
}
}
int main() {
  using opennav::platform::NativeQualifiedConfigLoad;
  try {
    Require(EnvironmentEquals(L"GITHUB_ACTIONS",L"true") &&
            EnvironmentEquals(L"RUNNER_ENVIRONMENT",L"github-hosted"),
            "Refusing non-disposable native fixture; never run on boat");
    PWSTR raw=nullptr;
    Require(SUCCEEDED(SHGetKnownFolderPath(FOLDERID_ProgramData,KF_FLAG_DONT_VERIFY,nullptr,&raw)),"KnownFolder failed");
    const std::wstring directory=std::wstring(raw)+L"\\opencpn";
    CoTaskMemFree(raw);
    // CREATE_NEW directory ownership is mandatory. No cleanup/adoption fallback.
    Require(CreateDirectoryW(directory.c_str(),nullptr)!=FALSE,
            "Refusing existing fixed profile; no existing bytes were touched");
    const auto path=directory+L"\\opencpn.ini";
    const auto link=directory+L"\\fixture-hardlink.ini";
    const char original[]="[Settings]\nQualificationFixture=original\n";
    HANDLE file=CreateFileW(path.c_str(),GENERIC_WRITE,0,nullptr,CREATE_NEW,FILE_ATTRIBUTE_NORMAL,nullptr);
    Require(file!=INVALID_HANDLE_VALUE,"Fresh INI create failed");
    DWORD written=0;
    const bool created=WriteFile(file,original,sizeof(original)-1,&written,nullptr) &&
        written==sizeof(original)-1 && FlushFileBuffers(file);
    const bool closed=CloseHandle(file)!=FALSE;
    Require(created && closed,"Fresh INI durable write failed");
    Refused([&]{ NativeQualifiedConfigLoad::Open(directory+L"\\..\\opencpn\\opencpn.ini"); },"Alias path accepted");
    {
      auto missing=NativeQualifiedConfigLoad::Open(path);
      Refused([&]{ missing->Complete(path); },"Completion without actual constructor accepted");
      Refused([&]{ missing->ObserveConstructor(path); },"Failed owner retried");
    }
    Require(CreateHardLinkW(link.c_str(),path.c_str(),nullptr)!=FALSE,"Hardlink fixture create failed");
    Refused([&]{ NativeQualifiedConfigLoad::Open(path); },"Hardlinked INI accepted");
    Require(DeleteFileW(link.c_str())!=FALSE,"Own fresh hardlink removal failed");
    wxInitializer wx;
    Require(wx.IsOk(),"wx initialization failed");
    {
      auto duplicate=NativeQualifiedConfigLoad::Open(path);
      wxFileConfig config("","",wxString(path),"",wxCONFIG_USE_LOCAL_FILE);
      duplicate->ObserveConstructor(path);
      Refused([&]{ duplicate->ObserveConstructor(path); },"Duplicate constructor accepted");
      Refused([&]{ duplicate->Complete(path); },"Poisoned constructor owner completed");
    }
    {
      auto owner=NativeQualifiedConfigLoad::Open(path);
      wxFileConfig config("","",wxString(path),"",wxCONFIG_USE_LOCAL_FILE);
      wxString value;
      Require(config.Read("/Settings/QualificationFixture",&value) && value=="original",
              "Actual wx constructor did not load original INI");
      owner->ObserveConstructor(path); // Same call made by patched MyConfig.
      HANDLE write=CreateFileW(path.c_str(),GENERIC_WRITE,FILE_SHARE_READ,nullptr,OPEN_EXISTING,0,nullptr);
      if(write!=INVALID_HANDLE_VALUE) { CloseHandle(write); throw std::runtime_error("INI write allowed during config load"); }
      Require(GetLastError()==ERROR_SHARING_VIOLATION,"Wrong denywrite refusal");
      Require(!DeleteFileW(path.c_str()) && GetLastError()==ERROR_SHARING_VIOLATION,
              "INI delete allowed during config load");
      const auto observation=owner->Complete(path);
      Require(observation.ini_sha256=="b8b8b70e8f3c75c2d696cf4865546a5d497077053e6ad3c87823060a3094547e" &&
              bool(opennav::integration::EncodeQualifiedConfigObservation(observation)),
              "Original byte hash/full IDs observation differs");
      Require(config.Write("/Settings/QualificationFixture","normal-write") && config.Flush(),
              "INI denywrite was not released before normal wx Flush");
      HANDLE directory_delete=CreateFileW(directory.c_str(),DELETE,FILE_SHARE_READ|FILE_SHARE_WRITE|FILE_SHARE_DELETE,
          nullptr,OPEN_EXISTING,FILE_FLAG_BACKUP_SEMANTICS,nullptr);
      if(directory_delete!=INVALID_HANDLE_VALUE) { CloseHandle(directory_delete); throw std::runtime_error("Directory custody released too soon"); }
      Require(GetLastError()==ERROR_SHARING_VIOLATION,"Wrong retained directory refusal");
      Refused([&]{ owner->Complete(path); },"Completed native owner reused");
      std::cout << "PASS original CONFIG identity " << *opennav::integration::EncodeQualifiedConfigObservation(observation) << '\n';
    }
    std::cout << "PASS inert fixed-profile native custody + actual wx read/Flush boundary; "
                 "fresh fixture retained for disposable-runner evidence; no installed app/health/boat qualification\n";
  } catch(const std::exception& e) {
    std::cerr << "FAIL " << e.what() << "; any fresh fixture retained; no preexisting profile adopted\n";
    return 1;
  }
}

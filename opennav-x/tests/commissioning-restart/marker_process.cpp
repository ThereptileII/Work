// Standalone native process fixture. No OpenCPN, GUI, connection or marine code.
#include "platform/PlatformIntegration.h"
#include "platform/windows/CommissioningRestartNative.h"
#include <windows.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>

namespace {
std::string Utf8(const std::wstring& text) {
  const int n=WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,text.data(),static_cast<int>(text.size()),nullptr,0,nullptr,nullptr);
  std::string out(n,'\0');
  WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,text.data(),static_cast<int>(text.size()),out.data(),n,nullptr,nullptr);
  return out;
}
std::string Environment(const wchar_t* name) {
  const DWORD n=GetEnvironmentVariableW(name,nullptr,0);if(!n)return {};
  std::wstring text(n,L'\0');const DWORD used=GetEnvironmentVariableW(name,text.data(),n);text.resize(used);return Utf8(text);
}
}
int wmain(int argc,wchar_t** argv) {
  if(argc!=2)return 70;
  wchar_t path[32768];const DWORD n=GetModuleFileNameW(nullptr,path,32768);
  if(!n || n==32768)return 71;
  const auto exe=Utf8(std::wstring(path,n));const std::string mode=Utf8(argv[1]);
  if(mode=="--parent") {
    const auto original=std::filesystem::current_path();
    std::string requested;std::ifstream("target-mode.txt")>>requested;
    if(requested.empty())return 72;
    if(std::filesystem::exists("erase-environment-after-start.txt")) {
      SetEnvironmentVariableW(L"OPENNAV_COMMISSIONING_RESTART_SESSION",nullptr);
      SetEnvironmentVariableW(L"OPENNAV_COMMISSIONING_RESTART_RECORD_SHA256",nullptr);
    }
    if(std::filesystem::exists("mutate-runtime-search-path.txt")) {
      SetEnvironmentVariableW(L"PATH",L"C:\\unreviewed-runtime-search");
      SetEnvironmentVariableW(L"LOCALAPPDATA",L"C:\\unreviewed-runtime-local");
      SetEnvironmentVariableW(L"APPDATA",L"C:\\unreviewed-runtime-roaming");
      SetEnvironmentVariableW(L"OPENNAV_TEST_RUNTIME_ADDITION",L"must-not-reach-child");
      SetEnvironmentVariableW(L"OPENNAV_TEST_COLD_ENVIRONMENT",nullptr);
      std::filesystem::create_directory("changed-working-directory");
      std::filesystem::current_path(original/L"changed-working-directory");
    }
    const bool armed=opennav::platform::RestartAfterExit(exe,{requested});
    std::ofstream(original/L"parent-armed.txt")<<(armed?"yes":"no")<<'\n'<<GetCurrentProcessId()<<'\n';
    if(!std::filesystem::exists(original/L"parent-fast-exit.txt")) {
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(15);
      while(!std::filesystem::exists(original/L"parent-release.txt")) {
        if(std::chrono::steady_clock::now()>=deadline)return 76;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
    return std::filesystem::exists(original/L"parent-error.txt")?7:0;
  }
  if(!opennav::platform::commissioning::IsMode(mode))return 73;
  const auto& binding=opennav::platform::commissioning::StartupBinding();
  FILETIME c,e,k,u;if(!GetProcessTimes(GetCurrentProcess(),&c,&e,&k,&u))return 74;
  std::ofstream out(std::filesystem::path("child"+mode+".txt"),std::ios::binary);
  out<<GetCurrentProcessId()<<'\n'<<((std::uint64_t(c.dwHighDateTime)<<32)|c.dwLowDateTime)<<'\n'
     <<static_cast<int>(binding.state)<<'\n'<<binding.session<<'\n'<<binding.record_sha256<<'\n'
     <<Environment(L"PATH")<<'\n'
     <<Environment(L"LOCALAPPDATA")<<'\n'<<Environment(L"APPDATA")<<'\n'
     <<Environment(L"OPENNAV_TEST_RUNTIME_ADDITION")<<'\n'
     <<Environment(L"OPENNAV_TEST_COLD_ENVIRONMENT")<<'\n';
  out.close();if(!out)return 75;
  if(mode=="--legacy" && std::filesystem::exists("chain-without-listener.txt")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::ofstream("chain-armed.txt")<<(opennav::platform::RestartAfterExit(exe,{"--xnav"})?"yes":"no");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));
  return 0;
}

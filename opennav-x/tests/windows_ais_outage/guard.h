// Disposable GitHub-hosted Windows proof only. Never part of the product.
#pragma once
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <algorithm>
#include <cwctype>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace outage {
inline void Require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
inline std::wstring Env(const wchar_t *key) {
  wchar_t value[32768]{};
  const DWORD size=GetEnvironmentVariableW(key,value,32768);
  Require(size>0 && size<32768,"required disposable environment missing");
  return value;
}
inline std::wstring Lower(std::wstring value) {
  std::transform(value.begin(),value.end(),value.begin(),towlower);return value;
}
inline std::filesystem::path Self() {
  wchar_t path[32768]{};
  const DWORD size=GetModuleFileNameW(nullptr,path,32768);
  Require(size>0 && size<32768,"executable path unavailable");
  return std::filesystem::canonical(path);
}
inline void Guard(const wchar_t *name) {
  Require(Env(L"CI")==L"true" && Env(L"GITHUB_ACTIONS")==L"true" &&
          Env(L"RUNNER_ENVIRONMENT")==L"github-hosted" &&
          Env(L"XNAV_DISPOSABLE_AIS_OUTAGE")==L"1","disposable CI required");
  const auto self=Self(), folder=self.parent_path();
  Require(Lower(self.filename().wstring())==name,"wrong proof executable name");
  Require(Lower(folder.parent_path().wstring())==
          Lower(std::filesystem::canonical(Env(L"RUNNER_TEMP")).wstring()),
          "proof must run directly below RUNNER_TEMP");
  const auto leaf=folder.filename().wstring();
  const std::wstring prefix=L"xnav-ais-outage-";
  Require(leaf.size()==prefix.size()+32 && leaf.substr(0,prefix.size())==prefix,
          "unique disposable directory required");
  Require(std::all_of(leaf.begin()+prefix.size(),leaf.end(),[](wchar_t c) {
    return (c>=L'0' && c<=L'9') || (c>=L'a' && c<=L'f');
  }),"invalid disposable directory identity");
}
inline unsigned Number(const wchar_t *s,unsigned low,unsigned high) {
  std::wstring text(s);
  Require(!text.empty() && std::all_of(text.begin(),text.end(),[](wchar_t c) {
    return c>=L'0' && c<=L'9';
  }),"invalid numeric argument");
  const auto n=std::stoull(text);
  Require(n>=low && n<=high,"numeric argument outside proof bounds");
  return static_cast<unsigned>(n);
}
inline void Deadline(DWORD milliseconds) {
  // Independent of stdin, socket calls and BFE RPC; termination drops the
  // dynamic session. The runner additionally owns a kill-on-close job.
  std::thread([milliseconds] {Sleep(milliseconds);TerminateProcess(GetCurrentProcess(),124);}).detach();
}
inline int Error(const std::exception &e) {
  // All errors supplied here are fixed strings, never payloads or credentials.
  std::cerr<<"proof error: "<<e.what()<<'\n';return 1;
}
} // namespace outage

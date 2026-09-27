#include "platform/windows/CommissioningRestartNative.h"
#include "platform/windows/WindowsArguments.h"

#include <windows.h>
#include <bcrypt.h>
#include <algorithm>
#include <array>
#include <climits>
#include <cwchar>
#include <filesystem>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace opennav::platform::commissioning {
namespace {
constexpr wchar_t SessionVariable[] = L"OPENNAV_COMMISSIONING_RESTART_SESSION";
constexpr wchar_t RecordVariable[] = L"OPENNAV_COMMISSIONING_RESTART_RECORD_SHA256";
struct Handle {
  HANDLE value = INVALID_HANDLE_VALUE;
  explicit Handle(HANDLE h = INVALID_HANDLE_VALUE) : value(h) {}
  ~Handle() { if (value != INVALID_HANDLE_VALUE && value) CloseHandle(value); }
  Handle(const Handle&) = delete;
  Handle& operator=(const Handle&) = delete;
};
std::string Utf8(const std::wstring& s) {
  if (s.empty()) return {};
  const int n=WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,s.data(),static_cast<int>(s.size()),nullptr,0,nullptr,nullptr);
  if (!n) throw std::runtime_error("Invalid UTF-16");
  std::string out(n,'\0');
  if (!WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,s.data(),static_cast<int>(s.size()),out.data(),n,nullptr,nullptr)) throw std::runtime_error("UTF-8 conversion failed");
  return out;
}
std::wstring Wide(const std::string& s) {
  if (s.empty()) return {};
  const int n=MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,s.data(),static_cast<int>(s.size()),nullptr,0);
  if (!n) throw std::runtime_error("Invalid UTF-8");
  std::wstring out(n,L'\0');
  if (!MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,s.data(),static_cast<int>(s.size()),out.data(),n)) throw std::runtime_error("UTF-16 conversion failed");
  return out;
}
// One immutable copy of the complete cold environment. OpenCPN/plugins may
// mutate PATH or plugin-root variables later; none are inherited on restart.
const std::optional<std::vector<std::wstring>> initial_environment=[]() noexcept
    -> std::optional<std::vector<std::wstring>> {
  LPWCH block=GetEnvironmentStringsW();
  if(!block)return {};
  try {
    std::vector<std::wstring> values;
    for(const auto* p=block;*p;p+=std::wcslen(p)+1)values.emplace_back(p);
    FreeEnvironmentStringsW(block);return values;
  } catch(...) {FreeEnvironmentStringsW(block);return {};}
}();
std::optional<std::string> Environment(const wchar_t* name) {
  if(!initial_environment)throw std::runtime_error("Cold environment unavailable");
  std::optional<std::string> result;
  for(const auto& entry:*initial_environment) {
    const auto equal=entry.find(L'=',entry[0]==L'='?1:0);
    if(equal==std::wstring::npos || _wcsicmp(entry.substr(0,equal).c_str(),name))continue;
    if(result || entry.size()>32768)return std::string{};
    result=Utf8(entry.substr(equal+1));
  }
  return result;
}
Binding InitialBinding() noexcept {
  try { return ReadBinding(Environment(SessionVariable),Environment(RecordVariable)); }
  catch (...) { return {GuardState::Invalid,{},{}}; }
}
const Binding initial_binding=InitialBinding();
// OpenCPN adds plugin directories to PATH at runtime. Preserve the audited cold
// launch environment for the helper, not that later search path.
const std::optional<std::string> initial_path=[]() noexcept -> std::optional<std::string> {
  try {return Environment(L"PATH");} catch(...) {return {};}
}();
const std::wstring initial_directory=[]() noexcept {
  try {
    const DWORD n=GetCurrentDirectoryW(0,nullptr);
    if(!n || n>32768)return std::wstring{};
    std::wstring s(n,L'\0');const DWORD used=GetCurrentDirectoryW(n,s.data());
    if(!used || used>=n)return std::wstring{};
    s.resize(used);return s;
  } catch(...) {return std::wstring{};}
}();
std::uint64_t Ticks(const FILETIME& t) { return (std::uint64_t(t.dwHighDateTime)<<32)|t.dwLowDateTime; }
std::uint64_t Now() { FILETIME t;GetSystemTimeAsFileTime(&t);return Ticks(t); }
std::uint64_t Created(HANDLE process) {
  FILETIME c,e,k,u;
  if(!GetProcessTimes(process,&c,&e,&k,&u)) throw std::runtime_error("Cannot identify process creation");
  return Ticks(c);
}
std::wstring Image(HANDLE process,DWORD flags=0) {
  std::wstring s(32768,L'\0');DWORD n=static_cast<DWORD>(s.size());
  if(!QueryFullProcessImageNameW(process,flags,s.data(),&n)) throw std::runtime_error("Cannot identify process image");
  s.resize(n);return s;
}
std::wstring Cwd() {
  const DWORD n=GetCurrentDirectoryW(0,nullptr);
  if(!n || n>32768) throw std::runtime_error("Invalid working directory");
  std::wstring s(n,L'\0');const DWORD used=GetCurrentDirectoryW(n,s.data());
  if(!used || used>=n) throw std::runtime_error("Working directory changed");
  s.resize(used);return s;
}
bool SamePath(const std::wstring& a,const std::wstring& b) { return _wcsicmp(a.c_str(),b.c_str())==0; }
std::vector<unsigned char> UserSid(HANDLE process) {
  HANDLE raw=nullptr;
  if(!OpenProcessToken(process,TOKEN_QUERY,&raw)) throw std::runtime_error("Cannot identify process user");
  Handle token(raw);DWORD n=0;
  GetTokenInformation(token.value,TokenUser,nullptr,0,&n);
  if(!n || n>65536) throw std::runtime_error("Invalid process user");
  std::vector<unsigned char> data(n);
  if(!GetTokenInformation(token.value,TokenUser,data.data(),n,&n)) throw std::runtime_error("Cannot read process user");
  const auto sid=reinterpret_cast<TOKEN_USER*>(data.data())->User.Sid;
  if(!IsValidSid(sid)) throw std::runtime_error("Invalid user SID");
  std::vector<unsigned char> result(GetLengthSid(sid));
  if(!CopySid(static_cast<DWORD>(result.size()),result.data(),sid)) throw std::runtime_error("Cannot copy user SID");
  return result;
}
std::string Hex(const unsigned char* p,std::size_t n) {
  constexpr char digits[]="0123456789abcdef";std::string s;
  for(std::size_t i=0;i<n;++i) {s+=digits[p[i]>>4];s+=digits[p[i]&15];}return s;
}
class Sha256 {
  BCRYPT_ALG_HANDLE algorithm=nullptr;
  BCRYPT_HASH_HANDLE hash=nullptr;
 public:
  Sha256() {
    if(BCryptOpenAlgorithmProvider(&algorithm,BCRYPT_SHA256_ALGORITHM,nullptr,0)<0) throw std::runtime_error("SHA unavailable");
    if(BCryptCreateHash(algorithm,&hash,nullptr,0,nullptr,0,0)<0) {BCryptCloseAlgorithmProvider(algorithm,0);throw std::runtime_error("SHA unavailable");}
  }
  ~Sha256() {BCryptDestroyHash(hash);BCryptCloseAlgorithmProvider(algorithm,0);}
  void Add(const char* p,std::size_t n) {
    if(n>ULONG_MAX || BCryptHashData(hash,reinterpret_cast<PUCHAR>(const_cast<char*>(p)),static_cast<ULONG>(n),0)<0) throw std::runtime_error("SHA failed");
  }
  std::string Finish() {
    std::array<unsigned char,32> digest{};
    if(BCryptFinishHash(hash,digest.data(),static_cast<ULONG>(digest.size()),0)<0) throw std::runtime_error("SHA failed");
    return Hex(digest.data(),digest.size());
  }
};
std::string Hash(const std::string& payload) {Sha256 h;h.Add(payload.data(),payload.size());return h.Finish();}
class LockedFile {
  Handle handle;
 public:
  explicit LockedFile(const std::wstring& path) : handle(CreateFileW(path.c_str(),GENERIC_READ,FILE_SHARE_READ,nullptr,OPEN_EXISTING,FILE_FLAG_OPEN_REPARSE_POINT,nullptr)) {
    if(handle.value==INVALID_HANDLE_VALUE) throw std::runtime_error("Critical file unavailable");
    BY_HANDLE_FILE_INFORMATION i{};
    if(!GetFileInformationByHandle(handle.value,&i) || (i.dwFileAttributes&(FILE_ATTRIBUTE_DIRECTORY|FILE_ATTRIBUTE_REPARSE_POINT)) || i.nNumberOfLinks!=1) throw std::runtime_error("Ambiguous critical file");
  }
  std::wstring NativePath() const {
    std::wstring path(32768,L'\0');
    const DWORD used=GetFinalPathNameByHandleW(handle.value,path.data(),
        static_cast<DWORD>(path.size()),FILE_NAME_NORMALIZED|VOLUME_NAME_NT);
    if(!used || used>=path.size())throw std::runtime_error("Cannot identify executable file path");
    path.resize(used);return path;
  }
  std::string Digest() {
    LARGE_INTEGER zero{};
    if(!SetFilePointerEx(handle.value,zero,nullptr,FILE_BEGIN)) throw std::runtime_error("Cannot hash critical file");
    Sha256 h;std::array<char,65536> buf{};DWORD n=0;
    do {
      if(!ReadFile(handle.value,buf.data(),static_cast<DWORD>(buf.size()),&n,nullptr)) throw std::runtime_error("Critical file read failed");
      h.Add(buf.data(),n);
    } while(n);
    return h.Finish();
  }
};
std::vector<wchar_t> ChildEnvironment(const Binding& binding,const std::optional<std::string>& path={}) {
  if(!initial_environment)throw std::runtime_error("Cold environment unavailable");
  std::vector<std::wstring> values;
  for(const auto& value:*initial_environment) {
    const auto equal=value.find(L'=',value[0]==L'='?1:0);
    const auto key=value.substr(0,equal);
    if(!_wcsicmp(key.c_str(),SessionVariable) || !_wcsicmp(key.c_str(),RecordVariable) ||
       (path && !_wcsicmp(key.c_str(),L"PATH")))continue;
    values.push_back(value);
  }
  values.push_back(std::wstring(SessionVariable)+L"="+Wide(binding.session));
  values.push_back(std::wstring(RecordVariable)+L"="+Wide(binding.record_sha256));
  if(path)values.push_back(L"PATH="+Wide(*path));
  std::sort(values.begin(),values.end(),[](const auto& a,const auto& b){return _wcsicmp(a.c_str(),b.c_str())<0;});
  std::vector<wchar_t> out;
  for(const auto& value:values){out.insert(out.end(),value.begin(),value.end());out.push_back(L'\0');}
  out.push_back(L'\0');return out;
}
bool CanonicalNumber(const wchar_t* text,std::uint64_t& value) {
  if(!text || !*text || (*text==L'0' && text[1])) return false;
  value=0;
  for(const auto* p=text;*p;++p) {
    if(*p<L'0' || *p>L'9' || value>(UINT64_MAX-(*p-L'0'))/10) return false;
    value=value*10+*p-L'0';
  }return true;
}
void Io(HANDLE pipe,bool write,char* data,DWORD size,ULONGLONG deadline) {
  DWORD total=0;
  while(total<size) {
    if(GetTickCount64()>=deadline)throw std::runtime_error("Pipe deadline expired before I/O");
    Handle event(CreateEventW(nullptr,TRUE,FALSE,nullptr));
    if(!event.value) throw std::runtime_error("Pipe event unavailable");
    OVERLAPPED ov{};ov.hEvent=event.value;DWORD n=0;
    BOOL ok=write?WriteFile(pipe,data+total,size-total,&n,&ov):ReadFile(pipe,data+total,size-total,&n,&ov);
    if(!ok && GetLastError()==ERROR_IO_PENDING) {
      const auto now=GetTickCount64();
      const DWORD remaining=now>=deadline?0:static_cast<DWORD>(std::min<ULONGLONG>(deadline-now,120000));
      const DWORD wait=WaitForSingleObject(event.value,remaining);
      if(wait!=WAIT_OBJECT_0) {
        CancelIoEx(pipe,&ov);WaitForSingleObject(event.value,INFINITE);
        throw std::runtime_error("Pipe timeout");
      }
      ok=GetOverlappedResult(pipe,&ov,&n,FALSE);
    }
    if(!ok || !n) throw std::runtime_error("Pipe closed or invalid");
    if(GetTickCount64()>=deadline)throw std::runtime_error("Pipe deadline expired during I/O");
    total+=n;
  }
}
void Send(HANDLE pipe,const std::string& payload,ULONGLONG deadline) {
  if(payload.empty() || payload.size()>MaximumFrame) throw std::runtime_error("Invalid frame");
  std::array<char,4> bytes{};
  for(int i=0;i!=4;++i) bytes[i]=static_cast<char>(payload.size()>>(i*8));
  Io(pipe,true,bytes.data(),4,deadline);
  Io(pipe,true,const_cast<char*>(payload.data()),static_cast<DWORD>(payload.size()),deadline);
}
std::string Receive(HANDLE pipe,ULONGLONG deadline) {
  std::array<unsigned char,4> bytes{};Io(pipe,false,reinterpret_cast<char*>(bytes.data()),4,deadline);
  std::uint32_t n=0;for(int i=0;i!=4;++i)n|=std::uint32_t(bytes[i])<<(i*8);
  if(!n || n>MaximumFrame) throw std::runtime_error("Invalid frame size");
  std::string payload(n,'\0');Io(pipe,false,payload.data(),n,deadline);return payload;
}
std::pair<DWORD,std::uint64_t> VerifyServer(HANDLE pipe,DWORD expected_session) {
  ULONG pid=0;
  if(!GetNamedPipeServerProcessId(pipe,&pid) || !pid) throw std::runtime_error("Unknown pipe server");
  Handle server(OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION,FALSE,pid));
  if(!server.value) throw std::runtime_error("Cannot inspect pipe server");
  DWORD session=0;
  if(!ProcessIdToSessionId(pid,&session) || session!=expected_session ||
     UserSid(server.value)!=UserSid(GetCurrentProcess())) throw std::runtime_error("Pipe peer identity mismatch");
  std::array<wchar_t,32768> windows{};const UINT n=GetWindowsDirectoryW(windows.data(),static_cast<UINT>(windows.size()));
  if(!n || n>=windows.size()) throw std::runtime_error("Windows path unavailable");
  const auto expected=(std::filesystem::path(windows.data())/L"System32"/L"WindowsPowerShell"/L"v1.0"/L"powershell.exe").wstring();
  if(!SamePath(Image(server.value),expected)) throw std::runtime_error("Only fixed native PowerShell verifier permitted");
  return {pid,Created(server.value)};
}
} // namespace
int ProtocolCapability() {return 1;}
const Binding& StartupBinding() {return initial_binding;}

bool SpawnGuardedHelper(const std::wstring& helper,const std::wstring& exe,const std::vector<std::string>& args) {
  try {
    const auto& b=StartupBinding();
    if(b.state!=GuardState::Armed || args.size()!=1 || !IsMode(args[0]) ||
       !initial_path || initial_path->empty() || initial_directory.empty() ||
       !SamePath(initial_directory,std::filesystem::path(exe).parent_path().wstring())) return false;
    HANDLE inherited=nullptr;
    if(!DuplicateHandle(GetCurrentProcess(),GetCurrentProcess(),GetCurrentProcess(),&inherited,
                        SYNCHRONIZE|PROCESS_QUERY_LIMITED_INFORMATION,TRUE,0)) return false;
    Handle parent(inherited);
    SIZE_T bytes=0;InitializeProcThreadAttributeList(nullptr,1,0,&bytes);
    if(!bytes) return false;
    std::vector<unsigned char> attributes(bytes);
    auto* list=reinterpret_cast<LPPROC_THREAD_ATTRIBUTE_LIST>(attributes.data());
    if(!InitializeProcThreadAttributeList(list,1,0,&bytes)) return false;
    struct Cleanup {LPPROC_THREAD_ATTRIBUTE_LIST list;~Cleanup(){DeleteProcThreadAttributeList(list);}} cleanup{list};
    if(!UpdateProcThreadAttribute(list,0,PROC_THREAD_ATTRIBUTE_HANDLE_LIST,&inherited,sizeof(inherited),nullptr,nullptr)) return false;
    std::wstring command=QuoteWindowsArgument(helper)+L" --commissioning-v1 "+
        std::to_wstring(reinterpret_cast<std::uintptr_t>(inherited))+L" "+
        std::to_wstring(GetCurrentProcessId())+L" "+Wide(b.session)+L" "+Wide(b.record_sha256)+L" "+
        QuoteWindowsArgument(exe)+L" "+QuoteWindowsArgument(Wide(args[0]));
    auto environment=ChildEnvironment(b,initial_path);
    STARTUPINFOEXW startup{};startup.StartupInfo.cb=sizeof(startup);startup.lpAttributeList=list;
    PROCESS_INFORMATION process{};
    if(!CreateProcessW(helper.c_str(),command.data(),nullptr,nullptr,TRUE,
        CREATE_NO_WINDOW|CREATE_UNICODE_ENVIRONMENT|EXTENDED_STARTUPINFO_PRESENT,
        environment.data(),initial_directory.c_str(),&startup.StartupInfo,&process)) return false;
    CloseHandle(process.hThread);CloseHandle(process.hProcess);return true;
  } catch(...) {return false;}
}

int RunGuardedHelper(int argc,wchar_t** argv) {
  // Every failure returns without calling the unarmed path.
  try {
    const auto& binding=StartupBinding();
    if(binding.state!=GuardState::Armed || argc!=8 || std::wcscmp(argv[1],L"--commissioning-v1") ||
       Utf8(argv[4])!=binding.session || Utf8(argv[5])!=binding.record_sha256) return 20;
    std::uint64_t handle_value=0,parent_pid=0;
    if(!CanonicalNumber(argv[2],handle_value) || !handle_value || handle_value>UINTPTR_MAX ||
       !CanonicalNumber(argv[3],parent_pid) || !parent_pid || parent_pid>MAXDWORD) return 21;
    Handle parent(reinterpret_cast<HANDLE>(static_cast<std::uintptr_t>(handle_value)));
    // A retained process handle survives exit, but its Win32 image-path query
    // can depend on already-destroyed user-mode process state. Compare the
    // native process image name with the exact executable file's resolved NT
    // path. There is no fallback which skips image identity on query failure.
    {
      LockedFile parent_executable(argv[6]);
      if(GetProcessId(parent.value)!=parent_pid ||
         !SamePath(Image(parent.value,PROCESS_NAME_NATIVE),parent_executable.NativePath())) return 22;
    }
    const auto parent_created=Created(parent.value);
    if(WaitForSingleObject(parent.value,30000)!=WAIT_OBJECT_0) return 23;
    DWORD code=1;
    if(!GetExitCodeProcess(parent.value,&code) || code!=0) return 24;
    const auto helper=Image(GetCurrentProcess());
    const std::filesystem::path exe(argv[6]);
    if(!exe.is_absolute() || !SamePath(helper,(exe.parent_path()/L"opennav-restart.exe").wstring()) ||
       !SamePath(Cwd(),exe.parent_path().wstring())) return 25;
    Request request;request.binding=binding;request.parent_pid=parent_pid;request.parent_created=parent_created;
    request.helper_pid=GetCurrentProcessId();request.helper_created=Created(GetCurrentProcess());
    DWORD session=0;
    if(!ProcessIdToSessionId(GetCurrentProcessId(),&session)) return 26;
    request.windows_session=session;request.executable=Utf8(exe.wstring());request.helper=Utf8(helper);
    request.working_directory=Utf8(Cwd());request.path=Environment(L"PATH").value_or("");request.arguments={Utf8(argv[7])};
    {LockedFile target(exe.wstring()),self(helper);request.executable_sha256=target.Digest();request.helper_sha256=self.Digest();}
    std::array<unsigned char,32> random{};
    if(BCryptGenRandom(nullptr,random.data(),static_cast<ULONG>(random.size()),BCRYPT_USE_SYSTEM_PREFERRED_RNG)<0) return 27;
    request.nonce=Hex(random.data(),random.size());
    const auto payload=EncodeRequest(request);
    if(!payload) return 28;
    const auto hash=Hash(*payload);
    const auto name=L"\\\\.\\pipe\\OpenNavX-CommissioningRestart-"+Wide(binding.session);
    if(!WaitNamedPipeW(name.c_str(),5000)) return 29;
    Handle pipe(CreateFileW(name.c_str(),GENERIC_READ|GENERIC_WRITE,0,nullptr,OPEN_EXISTING,
                           FILE_FLAG_OVERLAPPED|SECURITY_SQOS_PRESENT|SECURITY_IDENTIFICATION,nullptr));
    if(pipe.value==INVALID_HANDLE_VALUE) return 30;
    const auto server=VerifyServer(pipe.value,session);
    const auto deadline=GetTickCount64()+120000;
    Send(pipe.value,*payload,deadline);
    const auto permit=DecodePermit(Receive(pipe.value,deadline));
    if(!permit || !ValidatePermit(request,*permit,hash,Now())) return 31;
    if(VerifyServer(pipe.value,session)!=server) return 35;
    auto environment=ChildEnvironment(binding,permit->path);
    std::wstring command=QuoteWindowsArgument(exe.wstring())+L" "+QuoteWindowsArgument(Wide(request.arguments[0]));
    DWORD launch_error=0;std::uint64_t child_pid=0,child_created=0;
    {
      // Hold deny-write/delete handles through launch, including the final INI.
      LockedFile target(exe.wstring()),self(helper),profile(Wide(permit->profile));
      if(target.Digest()!=permit->executable_sha256 || self.Digest()!=permit->helper_sha256 ||
         profile.Digest()!=permit->profile_sha256 || !ValidatePermit(request,*permit,hash,Now())) return 32;
      STARTUPINFOW startup{};startup.cb=sizeof(startup);PROCESS_INFORMATION process{};
      if(!CreateProcessW(exe.c_str(),command.data(),nullptr,nullptr,FALSE,CREATE_UNICODE_ENVIRONMENT,
                         environment.data(),Wide(permit->working_directory).c_str(),&startup,&process)) {
        launch_error=GetLastError();
      } else {
        Handle child(process.hProcess),thread(process.hThread);
        child_pid=process.dwProcessId;child_created=Created(child.value);
      }
    }
    // A lost receipt never causes another CreateProcess attempt.
    Send(pipe.value,EncodeReceipt(request,*permit,hash,child_pid,child_created,launch_error),GetTickCount64()+5000);
    return child_pid?0:33;
  } catch(...) {return 34;}
}
} // namespace opennav::platform::commissioning

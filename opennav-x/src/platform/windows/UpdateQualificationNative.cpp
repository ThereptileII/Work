// Full 128-bit file identity requires the Windows 8 API declarations. Keep
// this minimum local to the native custody TU; there is no legacy-ID fallback.
#if !defined(_WIN32_WINNT) || _WIN32_WINNT < 0x0602
#undef _WIN32_WINNT
#define _WIN32_WINNT 0x0602
#endif
#include "platform/windows/UpdateQualificationNative.h"
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shlobj.h>
#include <bcrypt.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cwchar>
#include <stdexcept>
#include <iterator>
#include <vector>

namespace opennav::platform {
namespace {
[[noreturn]] void Refuse() { throw std::runtime_error("Qualified config custody unavailable"); }
struct Handle {
  HANDLE value = INVALID_HANDLE_VALUE;
  Handle() = default;
  explicit Handle(HANDLE h) : value(h) { if (h == INVALID_HANDLE_VALUE) Refuse(); }
  Handle(Handle&& other) noexcept : value(other.value) { other.value = INVALID_HANDLE_VALUE; }
  Handle(const Handle&) = delete;
  Handle& operator=(const Handle&) = delete;
  ~Handle() { if (value != INVALID_HANDLE_VALUE) CloseHandle(value); }
  void CloseChecked() {
    const auto h = value;
    value = INVALID_HANDLE_VALUE; // Never retry an uncertain close.
    if (h == INVALID_HANDLE_VALUE || !CloseHandle(h)) Refuse();
  }
};
std::string Hex64(std::uint64_t value) {
  constexpr char digits[] = "0123456789abcdef";
  std::string out(16,'0');
  for (unsigned i=0;i<16;++i) { out[15-i]=digits[value&15]; value>>=4; }
  return out;
}
std::string HexBytes(const unsigned char* bytes, std::size_t count) {
  constexpr char digits[] = "0123456789abcdef";
  std::string out(count*2,'0');
  for (std::size_t i=0;i<count;++i) { out[i*2]=digits[bytes[i]>>4]; out[i*2+1]=digits[bytes[i]&15]; }
  return out;
}
struct Identity {
  FILE_ID_INFO id{};
  FILE_BASIC_INFO basic{};
  FILE_STANDARD_INFO standard{};
};
Identity Inspect(HANDLE h, bool directory) {
  Identity x;
  if (GetFileType(h) != FILE_TYPE_DISK ||
      !GetFileInformationByHandleEx(h,FileIdInfo,&x.id,sizeof(x.id)) ||
      !GetFileInformationByHandleEx(h,FileBasicInfo,&x.basic,sizeof(x.basic)) ||
      !GetFileInformationByHandleEx(h,FileStandardInfo,&x.standard,sizeof(x.standard)) ||
      !!(x.basic.FileAttributes&FILE_ATTRIBUTE_DIRECTORY) != directory ||
      (x.basic.FileAttributes&FILE_ATTRIBUTE_REPARSE_POINT) || x.standard.DeletePending ||
      x.basic.CreationTime.QuadPart <= 0 ||
      (!directory && (x.standard.NumberOfLinks != 1 || x.standard.EndOfFile.QuadPart < 1 ||
                     x.standard.EndOfFile.QuadPart > 4ll*1024*1024))) Refuse();
  return x;
}
bool Same(const Identity& a,const Identity& b) {
  return a.id.VolumeSerialNumber == b.id.VolumeSerialNumber &&
      std::equal(std::begin(a.id.FileId.Identifier),std::end(a.id.FileId.Identifier),
                 std::begin(b.id.FileId.Identifier)) &&
      a.basic.CreationTime.QuadPart == b.basic.CreationTime.QuadPart;
}
std::wstring FinalPath(HANDLE h) {
  std::wstring result(32768,L'\0');
  const DWORD n=GetFinalPathNameByHandleW(h,result.data(),static_cast<DWORD>(result.size()),FILE_NAME_NORMALIZED|VOLUME_NAME_DOS);
  if (!n || n>=result.size()) Refuse();
  result.resize(n);
  if (result.compare(0,4,L"\\\\?\\") != 0) Refuse();
  return result.substr(4);
}
bool EqualPath(const std::wstring& a,const std::wstring& b) {
  return CompareStringOrdinal(a.c_str(),-1,b.c_str(),-1,TRUE)==CSTR_EQUAL;
}
void CheckPath(const std::wstring& actual,const std::wstring& fixed) {
  // No caller path selection, aliases, dot components, relative/UNC paths or
  // alternate streams. The actual wx constructor must use this fixed spelling.
  if (!EqualPath(actual,fixed)) Refuse();
}
std::vector<unsigned char> ReadOriginal(HANDLE h) {
  const auto id=Inspect(h,false);
  LARGE_INTEGER zero{};
  if (!SetFilePointerEx(h,zero,nullptr,FILE_BEGIN)) Refuse();
  std::vector<unsigned char> out(static_cast<std::size_t>(id.standard.EndOfFile.QuadPart));
  DWORD n=0;
  if (!ReadFile(h,out.data(),static_cast<DWORD>(out.size()),&n,nullptr) || n!=out.size()) Refuse();
  unsigned char extra=0;
  if (!ReadFile(h,&extra,1,&n,nullptr) || n) Refuse();
  return out;
}
std::string Sha256(const std::vector<unsigned char>& bytes) {
  BCRYPT_ALG_HANDLE algorithm=nullptr;
  BCRYPT_HASH_HANDLE hash=nullptr;
  struct Cleanup { BCRYPT_ALG_HANDLE& a; BCRYPT_HASH_HANDLE& h;
    ~Cleanup() { if(h) BCryptDestroyHash(h); if(a) BCryptCloseAlgorithmProvider(a,0); }
  } cleanup{algorithm,hash};
  std::array<unsigned char,32> digest{};
  if (BCryptOpenAlgorithmProvider(&algorithm,BCRYPT_SHA256_ALGORITHM,nullptr,0)<0 ||
      BCryptCreateHash(algorithm,&hash,nullptr,0,nullptr,0,0)<0 ||
      BCryptHashData(hash,const_cast<PUCHAR>(bytes.data()),static_cast<ULONG>(bytes.size()),0)<0 ||
      BCryptFinishHash(hash,digest.data(),static_cast<ULONG>(digest.size()),0)<0) Refuse();
  return HexBytes(digest.data(),digest.size());
}
std::wstring FixedDirectory() {
  PWSTR raw=nullptr;
  if (FAILED(SHGetKnownFolderPath(FOLDERID_ProgramData,KF_FLAG_DONT_VERIFY,nullptr,&raw))) Refuse();
  struct Free { PWSTR p; ~Free(){ CoTaskMemFree(p); } } free{raw};
  std::wstring path(raw);
  if (path.size()<3 || path[1]!=L':' || path[2]!=L'\\' || path.find(L'/',0)!=std::wstring::npos ||
      path.find(L':',2)!=std::wstring::npos || path.back()==L'\\') Refuse();
  return path+L"\\opencpn";
}
struct Directory {
  std::wstring path;
  Handle handle;
  Identity identity;
  explicit Directory(const std::wstring& p) : path(p),
      // Attribute-only opens do not establish the read/share-delete constraint
      // needed to exclude directory renames. Retain a real directory read.
      handle(CreateFileW(p.c_str(),FILE_LIST_DIRECTORY|FILE_READ_ATTRIBUTES|SYNCHRONIZE,FILE_SHARE_READ|FILE_SHARE_WRITE,
                        nullptr,OPEN_EXISTING,FILE_FLAG_BACKUP_SEMANTICS|FILE_FLAG_OPEN_REPARSE_POINT,nullptr)),
      identity(Inspect(handle.value,true)) {
    auto final=FinalPath(handle.value);
    if (p.size()==3 && final.size()==2) final+=L'\\';
    if (!EqualPath(p,final)) Refuse();
  }
  void Validate() const {
    if (!Same(identity,Inspect(handle.value,true))) Refuse();
    Directory reopened(path);
    if (!Same(identity,reopened.identity)) Refuse();
    reopened.handle.CloseChecked();
  }
};
} // namespace

std::pair<std::optional<std::string>,bool> TakeUpdateQualificationFlag() {
  constexpr wchar_t name[]=L"SKAGER_UPDATE_QUALIFY_PROFILE";
  wchar_t raw[8]{};
  SetLastError(ERROR_SUCCESS);
  const DWORD count=GetEnvironmentVariableW(name,raw,8);
  const DWORD error=GetLastError();
  std::optional<std::string> value;
  if (count || error!=ERROR_ENVVAR_NOT_FOUND) value=(count==1 && raw[0]==L'1')?"1":"invalid";
  const auto crt=_wputenv_s(name,L"");
  const BOOL os=SetEnvironmentVariableW(name,nullptr);
  const auto removed=crt==0 && (os || GetLastError()==ERROR_ENVVAR_NOT_FOUND);
  return {std::move(value),removed};
}
struct NativeQualifiedConfigLoad::Impl {
  std::wstring directory=FixedDirectory(), path=directory+L"\\opencpn.ini";
  std::vector<Directory> parents;
  Handle file;
  Identity identity;
  std::vector<unsigned char> original;
  bool constructor=false, completed=false, poisoned=false;
  explicit Impl(const std::wstring& actual) {
    CheckPath(actual,path);
    wchar_t fs[32]{};
    const auto root=directory.substr(0,3);
    if (!GetVolumeInformationW(root.c_str(),nullptr,0,nullptr,nullptr,nullptr,fs,32) ||
        std::wcscmp(fs,L"NTFS")!=0) Refuse();
    parents.emplace_back(root);
    for (std::size_t end=directory.find(L'\\',3);;end=directory.find(L'\\',end+1)) {
      parents.emplace_back(directory.substr(0,end));
      if (end==std::wstring::npos) break;
    }
    // Sharing deliberately excludes writes and deletes through the real read.
    Handle opened(CreateFileW(path.c_str(),GENERIC_READ,FILE_SHARE_READ,nullptr,OPEN_EXISTING,
                              FILE_FLAG_OPEN_REPARSE_POINT,nullptr));
    identity=Inspect(opened.value,false);
    if (!EqualPath(FinalPath(opened.value),path) ||
        identity.id.VolumeSerialNumber!=parents.back().identity.id.VolumeSerialNumber) Refuse();
    original=ReadOriginal(opened.value);
    file.value=opened.value; opened.value=INVALID_HANDLE_VALUE;
  }
  void Validate(const std::wstring& actual) {
    CheckPath(actual,path);
    for (const auto& p:parents) p.Validate();
    if (!Same(identity,Inspect(file.value,false)) || ReadOriginal(file.value)!=original) Refuse();
    Handle reopened(CreateFileW(path.c_str(),GENERIC_READ,FILE_SHARE_READ,nullptr,OPEN_EXISTING,
                                FILE_FLAG_OPEN_REPARSE_POINT,nullptr));
    if (!Same(identity,Inspect(reopened.value,false)) || !EqualPath(FinalPath(reopened.value),path)) Refuse();
    reopened.CloseChecked();
  }
};
NativeQualifiedConfigLoad::NativeQualifiedConfigLoad(std::unique_ptr<Impl> p):impl_(std::move(p)){}
NativeQualifiedConfigLoad::~NativeQualifiedConfigLoad()=default;
std::unique_ptr<NativeQualifiedConfigLoad> NativeQualifiedConfigLoad::Open(const std::wstring& path) {
  return std::unique_ptr<NativeQualifiedConfigLoad>(new NativeQualifiedConfigLoad(std::make_unique<Impl>(path)));
}
void NativeQualifiedConfigLoad::ObserveConstructor(const std::wstring& path) {
  if (impl_->constructor || impl_->completed || impl_->poisoned) { impl_->poisoned=true; Refuse(); }
  impl_->poisoned=true;
  impl_->Validate(path);
  impl_->constructor=true;
  impl_->poisoned=false;
}
integration::QualifiedConfigObservation NativeQualifiedConfigLoad::Complete(const std::wstring& path) {
  if (!impl_->constructor || impl_->completed || impl_->poisoned) { impl_->poisoned=true; Refuse(); }
  impl_->poisoned=true;
  impl_->Validate(path);
  const auto& dir=impl_->parents.back().identity;
  const auto& file=impl_->identity;
  integration::QualifiedConfigObservation observation{
    Hex64(dir.id.VolumeSerialNumber),HexBytes(dir.id.FileId.Identifier,16),Hex64(static_cast<std::uint64_t>(dir.basic.CreationTime.QuadPart)),
    Hex64(file.id.VolumeSerialNumber),HexBytes(file.id.FileId.Identifier,16),Hex64(static_cast<std::uint64_t>(file.basic.CreationTime.QuadPart)),
    Sha256(impl_->original)};
  impl_->file.CloseChecked(); // Before any ordinary wx Flush/UpdateSettings.
  impl_->completed=true;
  impl_->poisoned=false;
  return observation;
}
} // namespace opennav::platform

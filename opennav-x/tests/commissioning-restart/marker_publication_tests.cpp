#include "AtomicMarker.h"
#include "ReadClosedMarker.h"
#include <cstddef>
#include <cstring>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#endif
namespace fs=std::filesystem;
namespace {
int checks=0;
void Require(bool condition,const std::string &message){++checks;if(!condition)throw std::runtime_error(message);}
void NoStaging(const fs::path &root) {
  for(const auto &entry:fs::directory_iterator(root))
    Require(entry.path().filename().string().find(".opennav-marker-")!=0,"staging residue after publication/failure");
}
#ifdef _WIN32
struct NativeHandle {
  HANDLE value;
  ~NativeHandle(){if(value!=INVALID_HANDLE_VALUE)CloseHandle(value);}
};
void NativeSuccess(bool ok,const fs::path &path,const char *stage) {
  const DWORD code=ok?ERROR_SUCCESS:GetLastError();
  ++checks;
  if(!ok)throw opennav::tests::MarkerReadError(path,"native sharing proof",stage,code);
}
void WriterRefused(const fs::path &path,const char *caller) {
  bool refused=false;
  try{opennav::tests::ReadClosedMarker(path,caller);}
  catch(const opennav::tests::MarkerReadError &failure) {
    if(failure.stage!="CreateFileW"||failure.code!=ERROR_SHARING_VIOLATION)throw;
    refused=true;
  }
  Require(refused,"readiness reader refuses an open WRITE handle without retry");
}
void NativeSharing(const fs::path &root,const std::string &payload) {
  const auto source=root/"native-before.txt",renamed=root/"native-after.txt";
  std::string error;
  const bool prepared=opennav::tests::PublishMarker(source,payload,error);
  Require(prepared,"closed native rename fixture prepared: "+error);
  {
    NativeHandle held{CreateFileW(source.c_str(),DELETE,
        FILE_SHARE_READ|FILE_SHARE_WRITE|FILE_SHARE_DELETE,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr)};
    NativeSuccess(held.value!=INVALID_HANDLE_VALUE,source,"open DELETE-only handle");
    // Perform the real rename using this very handle, deliberately retaining
    // it after the new name is visible. No timing or scheduler luck is needed.
    const auto name=fs::absolute(renamed).wstring();
    const auto bytes=offsetof(FILE_RENAME_INFO,FileName)+(name.size()+1)*sizeof(wchar_t);
    std::vector<std::uint64_t> storage((bytes+sizeof(std::uint64_t)-1)/sizeof(std::uint64_t),0);
    auto *rename=reinterpret_cast<FILE_RENAME_INFO *>(storage.data());
    rename->ReplaceIfExists=FALSE;rename->RootDirectory=nullptr;
    rename->FileNameLength=static_cast<DWORD>(name.size()*sizeof(wchar_t));
    std::memcpy(rename->FileName,name.c_str(),(name.size()+1)*sizeof(wchar_t));
    NativeSuccess(SetFileInformationByHandle(held.value,FileRenameInfo,rename,static_cast<DWORD>(bytes))!=0,
                  source,"SetFileInformationByHandle rename");
    Require(!fs::exists(source)&&fs::exists(renamed),"real renamed path visible while DELETE handle retained");
    {
      std::ifstream previous(renamed,std::ios::binary);
      Require(!previous,"old ifstream reader reproduces incompatible rename sharing");
    }
    NativeHandle old_share{CreateFileW(renamed.c_str(),GENERIC_READ,FILE_SHARE_READ|FILE_SHARE_WRITE,
        nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr)};
    const DWORD old_error=old_share.value==INVALID_HANDLE_VALUE?GetLastError():ERROR_SUCCESS;
    Require(old_error==ERROR_SHARING_VIOLATION,"missing DELETE sharing produces native error 32");
    Require(opennav::tests::ReadClosedMarker(renamed,"held DELETE after real rename")==payload,
            "one immediate read succeeds while real rename DELETE handle stays open");
  }
  {
    NativeHandle writer{CreateFileW(renamed.c_str(),GENERIC_WRITE,
        FILE_SHARE_READ|FILE_SHARE_WRITE|FILE_SHARE_DELETE,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr)};
    NativeSuccess(writer.value!=INVALID_HANDLE_VALUE,renamed,"open content writer");
    WriterRefused(renamed,"held content writer");
  }
  Require(opennav::tests::ReadClosedMarker(renamed,"writer closed")==payload,
          "closing the WRITE handle restores immediate complete read");
  fs::remove(renamed);
}
#endif
}
int main() {
  const auto root=fs::temp_directory_path()/
      ("OpenNav marker readiness & "+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    const auto unicode_path = fs::path(u"marker-\u00c5-\u6c34.txt");
    const std::string unicode_bytes = "marker-\xc3\x85-\xe6\xb0\xb4.txt";
    const opennav::tests::MarkerReadError unicode_error(
        unicode_path, "unicode readiness", "open", 42);
    Require(std::string(unicode_error.what()).find(unicode_bytes) != std::string::npos,
            "reader diagnostics preserve UTF-8 path bytes in C++17 and C++20");
    Require(fs::create_directory(root),"isolated publication test directory");
    const auto final=root/"child--xnav.txt";
    const std::string payload="42\n123456789\n1\nsession\nrecord\npath\nlocal\nroaming\n\nunchanged\n";
    std::string error;unsigned stages=0;
    const bool published=opennav::tests::PublishMarker(final,payload,error,[&](auto stage,const fs::path &staged){
      ++stages;Require(!fs::exists(final),"final readiness path absent until close");
      Require(fs::exists(staged),"private staging exists before publication");
#ifdef _WIN32
      if(stage==opennav::tests::MarkerStage::Written)WriterRefused(staged,"actual publisher still open");
#endif
      if(stage==opennav::tests::MarkerStage::Closed) {
        Require(opennav::tests::ReadClosedMarker(staged,"closed publisher stage")==payload,"closed staging contains every byte and empty line");
#ifdef _WIN32
        const auto handle=CreateFileW(staged.c_str(),GENERIC_READ,0,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
        NativeSuccess(handle!=INVALID_HANDLE_VALUE,staged,"closed publisher exclusive native read");
        NativeSuccess(CloseHandle(handle)!=0,staged,"close exclusive readiness probe");
#endif
      }
    });
    Require(published,"publish marker normally: "+error);
    Require(stages==2&&error.empty(),"both preparation stages complete without error");
    Require(opennav::tests::ReadClosedMarker(final,"normal first read")==payload,"published marker complete on first read");
    Require(!opennav::tests::PublishMarker(final,"replacement",error)&&!error.empty(),"existing final marker refused");
    Require(opennav::tests::ReadClosedMarker(final,"existing marker preserved")==payload,"existing marker never truncated");
    for(auto stage:{opennav::tests::MarkerStage::Written,opennav::tests::MarkerStage::Closed}) {
      const auto refused=root/(stage==opennav::tests::MarkerStage::Written?"write-failed.txt":"close-failed.txt");
      Require(!opennav::tests::PublishMarker(refused,payload,error,[&](auto at,const auto &){if(at==stage)throw std::runtime_error("injected fixture preparation failure");}),"preparation failure refuses publication");
      Require(!fs::exists(refused)&&!error.empty(),"failed preparation has no ready marker");NoStaging(root);
    }
    Require(!opennav::tests::PublishMarker(root/"missing"/"marker.txt",payload,error),"missing parent is a hard write failure");
    Require(!fs::exists(root/"missing"),"failure does not invent parent locations");
    fs::create_directory(root/"occupied");
    Require(!opennav::tests::PublishMarker(root/"occupied",payload,error)&&fs::is_directory(root/"occupied"),"directory destination preserved");
    // Mirror WaitFile -> one immediate closed-marker read, without retries.
    // The reader runs concurrently while a larger fixture payload is written.
    for(unsigned i=0;i<16;++i) {
      const auto target=root/("concurrent-"+std::to_string(i)+".txt");
      const auto complete=std::string(128*1024,char('a'+i))+"\ncomplete\n";
      auto reader=std::async(std::launch::async,[&]{
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
        while(!fs::exists(target)) {
          if(std::chrono::steady_clock::now()>=deadline)throw std::runtime_error("publication deadline");
          std::this_thread::yield();
        }
        return opennav::tests::ReadClosedMarker(target,"concurrent readiness iteration "+std::to_string(i));
      });
      const bool prepared=opennav::tests::PublishMarker(target,complete,error);
      Require(prepared,"concurrent publication iteration "+std::to_string(i)+": "+error);
      Require(reader.get()==complete,"first readiness read is complete without retry");
    }
#ifdef _WIN32
    NativeSharing(root,payload);
#endif
    bool missing_refused=false;
    try{opennav::tests::ReadClosedMarker(root/"never-published.txt","missing readiness");}
    catch(const opennav::tests::MarkerReadError &failure) {
      const std::string message=failure.what();
      missing_refused=failure.code!=0 && !failure.stage.empty() &&
          message.find("missing readiness")!=std::string::npos &&
          message.find("never-published.txt")!=std::string::npos;
    }
    Require(missing_refused,"missing reader failure records caller, path, stage and native error");
    const auto oversized=root/"oversized.txt";
    const bool size_prepared=opennav::tests::PublishMarker(oversized,std::string(1024*1024+1,'x'),error);
    Require(size_prepared,"oversized reader fixture prepared: "+error);
    bool size_refused=false;
    try{opennav::tests::ReadClosedMarker(oversized,"bounded readiness");}
    catch(const opennav::tests::MarkerReadError &failure) {
      size_refused=failure.stage=="size exceeds one MiB" && failure.code!=0;
    }
    Require(size_refused,"readiness reader refuses more than one MiB immediately");
    fs::remove(oversized);
    NoStaging(root);fs::remove_all(root);
    std::cout<<"PASS "<<checks<<" marker publication checks";
#ifdef _WIN32
    std::cout<<" (native Windows exclusive-handle proof)\n";
#else
    std::cout<<" (portable algorithm only; Windows sharing gate pending)\n";
#endif
    return 0;
  } catch(const std::exception &error) {
    std::cerr<<error.what()<<'\n';std::error_code ignored;fs::remove_all(root,ignored);return 1;
  }
}

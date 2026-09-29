#include "AtomicMarker.h"
#include <future>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <thread>
#ifdef _WIN32
#include <windows.h>
#endif
namespace fs=std::filesystem;
namespace {
int checks=0;
void Require(bool condition,const char *message){++checks;if(!condition)throw std::runtime_error(message);}
std::string Read(const fs::path &path) {
  std::ifstream input(path,std::ios::binary);
  if(!input)throw std::runtime_error("published marker cannot be opened");
  return {std::istreambuf_iterator<char>(input),std::istreambuf_iterator<char>()};
}
void NoStaging(const fs::path &root) {
  for(const auto &entry:fs::directory_iterator(root))
    Require(entry.path().filename().string().find(".opennav-marker-")!=0,"staging residue after publication/failure");
}
}
int main() {
  const auto root=fs::temp_directory_path()/
      ("OpenNav marker readiness & "+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    Require(fs::create_directory(root),"isolated publication test directory");
    const auto final=root/"child--xnav.txt";
    const std::string payload="42\n123456789\n1\nsession\nrecord\npath\nlocal\nroaming\n\nunchanged\n";
    std::string error;unsigned stages=0;
    Require(opennav::tests::PublishMarker(final,payload,error,[&](auto stage,const fs::path &staged){
      ++stages;Require(!fs::exists(final),"final readiness path absent until close");
      Require(fs::exists(staged),"private staging exists before publication");
      if(stage==opennav::tests::MarkerStage::Closed) {
        Require(Read(staged)==payload,"closed staging contains every byte and empty line");
#ifdef _WIN32
        const auto handle=CreateFileW(staged.c_str(),GENERIC_READ,0,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
        Require(handle!=INVALID_HANDLE_VALUE,"closed staged marker permits exclusive native read");
        Require(CloseHandle(handle)!=0,"exclusive readiness probe handle closed");
#endif
      }
    }),"publish marker normally");
    Require(stages==2&&error.empty(),"both preparation stages complete without error");
    Require(Read(final)==payload,"published marker complete on first read");
    Require(!opennav::tests::PublishMarker(final,"replacement",error)&&!error.empty(),"existing final marker refused");
    Require(Read(final)==payload,"existing marker never truncated");
    for(auto stage:{opennav::tests::MarkerStage::Written,opennav::tests::MarkerStage::Closed}) {
      const auto refused=root/(stage==opennav::tests::MarkerStage::Written?"write-failed.txt":"close-failed.txt");
      Require(!opennav::tests::PublishMarker(refused,payload,error,[&](auto at,const auto &){if(at==stage)throw std::runtime_error("injected fixture preparation failure");}),"preparation failure refuses publication");
      Require(!fs::exists(refused)&&!error.empty(),"failed preparation has no ready marker");NoStaging(root);
    }
    Require(!opennav::tests::PublishMarker(root/"missing"/"marker.txt",payload,error),"missing parent is a hard write failure");
    Require(!fs::exists(root/"missing"),"failure does not invent parent locations");
    fs::create_directory(root/"occupied");
    Require(!opennav::tests::PublishMarker(root/"occupied",payload,error)&&fs::is_directory(root/"occupied"),"directory destination preserved");
    // Mirror WaitFile -> one immediate ReadAllLines, without read retries.
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
        return Read(target);
      });
      Require(opennav::tests::PublishMarker(target,complete,error),"concurrent publication succeeds");
      Require(reader.get()==complete,"first readiness read is complete without retry");
    }
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

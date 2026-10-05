#include <cstdlib>
#include <new>
// Fail only after the real WX result container has been returned to the bridge.
// This proves both conversion allocations release containers and propagate OOM.
int failAllocation=0;
void* trackedQueryList=nullptr;
bool queryListDeleted=false;
void* operator new(std::size_t bytes) {
  if(failAllocation && --failAllocation==0)throw std::bad_alloc();
  if(void* p=std::malloc(bytes))return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept {
  if(p && p==trackedQueryList){queryListDeleted=true;trackedQueryList=nullptr;}
  std::free(p);
}
void operator delete(void* p,std::size_t) noexcept {::operator delete(p);}

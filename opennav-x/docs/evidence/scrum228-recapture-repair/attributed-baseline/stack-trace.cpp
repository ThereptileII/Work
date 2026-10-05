#include <wx/event.h>
#include <cstring>
static thread_local void* current_event_method=nullptr;
#include <X11/Xlib.h>
#include <dlfcn.h>
#include <execinfo.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <time.h>
static void trace(const char* name, unsigned long window, int mode) {
 const char* path=getenv("SKAGER_STACK_TRACE"); if(!path)return;
 FILE* f=fopen(path,"a");if(!f)return;
 timespec t;clock_gettime(CLOCK_MONOTONIC,&t);
 fprintf(f,"STACK %s window=%lu mode=%d pid=%d time=%ld.%09ld\n",name,window,mode,getpid(),t.tv_sec,t.tv_nsec);
 Dl_info info{};if(current_event_method && dladdr(current_event_method,&info))fprintf(f,"EVENT_HANDLER %s address=%p offset=%lx\n",info.dli_sname?info.dli_sname:"?",current_event_method,(unsigned long)current_event_method-(unsigned long)info.dli_fbase);fflush(f);
 void* bt[24];int n=backtrace(bt,24);backtrace_symbols_fd(bt,n,fileno(f));fclose(f);
}
extern "C" int XRaiseWindow(Display* d,Window w) {
 static auto real=(int(*)(Display*,Window))dlsym(RTLD_NEXT,"XRaiseWindow");trace("XRaiseWindow",w,0);return real(d,w);
}
extern "C" int XConfigureWindow(Display* d,Window w,unsigned int mask,XWindowChanges* changes) {
 static auto real=(int(*)(Display*,Window,unsigned int,XWindowChanges*))dlsym(RTLD_NEXT,"XConfigureWindow");if(mask&CWStackMode)trace("XConfigureWindow",w,changes->stack_mode);return real(d,w,mask,changes);
}
extern "C" int XRestackWindows(Display* d,Window* ws,int count) {
 static auto real=(int(*)(Display*,Window*,int))dlsym(RTLD_NEXT,"XRestackWindows");if(count)trace("XRestackWindows",ws[0],count);return real(d,ws,count);
}

bool wxEvtHandler::ProcessEventIfMatchesId(const wxEventTableEntryBase& entry,wxEvtHandler* handler,wxEvent& event) {
 static auto real=(bool(*)(const wxEventTableEntryBase&,wxEvtHandler*,wxEvent&))dlsym(RTLD_NEXT,"_ZN12wxEvtHandler23ProcessEventIfMatchesIdERK21wxEventTableEntryBasePS_R7wxEvent");
 void* previous=current_event_method;current_event_method=nullptr;
 if(entry.m_fn){auto method=entry.m_fn->GetEvtMethod();std::memcpy(&current_event_method,&method,sizeof(current_event_method));}
 bool result=real(entry,handler,event);current_event_method=previous;return result;
}
// Keep diagnostic interposition inside this process, not GTK icon-loader helpers.
__attribute__((constructor)) static void clear_child_preload() { unsetenv("LD_PRELOAD"); }

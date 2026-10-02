#include <GL/gl.h>
#include <dlfcn.h>
#include <cstdio>
#include <cstdlib>
#include <ctime>
static void trace(const char* what,int w,int h) {
 if(w<400||h<300)return;
 auto p=std::getenv("SKAGER_GL_TRACE");if(!p)return;
 if(auto f=std::fopen(p,"a")){timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);std::fprintf(f,"%ld.%09ld %s %d %d\n",t.tv_sec,t.tv_nsec,what,w,h);std::fclose(f);}
}
extern "C" void glTexImage2D(GLenum target,GLint level,GLint internal,GLsizei w,GLsizei h,GLint border,GLenum format,GLenum type,const void* pixels){
 static auto real=reinterpret_cast<decltype(&glTexImage2D)>(dlsym(RTLD_NEXT,"glTexImage2D"));
 if(!pixels&&format==GL_RGBA)trace("RGBA_ALLOCATION",w,h);
 real(target,level,internal,w,h,border,format,type,pixels);
}
extern "C" void glViewport(GLint x,GLint y,GLsizei w,GLsizei h){
 static auto real=reinterpret_cast<decltype(&glViewport)>(dlsym(RTLD_NEXT,"glViewport"));
 static int lastW=-1,lastH=-1;
 if(w!=lastW||h!=lastH){trace("VIEWPORT",w,h);lastW=w;lastH=h;}
 real(x,y,w,h);
}

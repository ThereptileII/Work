#include "plugin-adapters/ocharts/BindingState.h"
#include <cassert>
#include <cstring>
#include <iostream>
using skager::ocharts::BindingState;
SkagerChartBindingV1 Valid() {
  SkagerChartBindingV1 value{};
  value.structBytes=sizeof(value); value.version=SKAGER_CHART_BINDING_VERSION;
#ifdef _WIN32
  std::strcpy(value.resourceDirectory,"C:\\SKAGER\\charts-å");
#else
  std::strcpy(value.resourceDirectory,"/private/SKAGER/charts-å");
#endif
  return value;
}
SkagerChartPresentationStatusV1 Status(const BindingState& b) {
  SkagerChartPresentationStatusV1 s{};s.structBytes=sizeof(s);s.version=1;
  assert(b.ReadStatus(&s)); return s;
}
void Refuses(const SkagerChartBindingV1& value) {
  BindingState b;assert(!b.Bind(&value));
  assert(Status(b).state==SKAGER_CHART_UNBOUND);
  assert(b.BeginInitialization()[0]==0); // failed input never copied
}
int main() {
  auto v=Valid(); BindingState b;
  assert(!b.Bind(nullptr)); assert(Status(b).reason==SKAGER_CHART_REASON_UNBOUND);
  assert(b.Bind(&v)); v.resourceDirectory[1]='X';
  assert(Status(b).state==SKAGER_CHART_BOUND_PENDING_INITIALIZATION);
  assert(!b.Bind(&v)); // no replacement, even before initialization
  const auto copied=b.BeginInitialization();const auto expected=Valid();
  assert(!std::memcmp(copied.data(),expected.resourceDirectory,copied.size()));
  b.Complete(true,SKAGER_CHART_REASON_NONE);assert(Status(b).state==SKAGER_CHART_SELECTED);
  b.Complete(false,SKAGER_CHART_REASON_RENDERER_INITIALIZATION);
  assert(Status(b).state==SKAGER_CHART_STANDARD_FALLBACK);
  assert(Status(b).reason==SKAGER_CHART_REASON_RENDERER_INITIALIZATION);
  assert(!b.Bind(&expected));
  BindingState queried;assert(Status(queried).state==SKAGER_CHART_UNBOUND);
  assert(queried.Bind(&expected)); // query did not initialize the renderer
  BindingState unbound;unbound.BeginInitialization();assert(!unbound.Bind(&expected));
  auto bad=Valid();--bad.structBytes;Refuses(bad);
  bad=Valid();bad.version=2;Refuses(bad);
  bad=Valid();bad.reserved[7]=1;Refuses(bad);
  bad=Valid();std::memset(bad.resourceDirectory,'a',sizeof(bad.resourceDirectory));Refuses(bad);
  bad=Valid();bad.resourceDirectory[0]=0;Refuses(bad);
  bad=Valid();bad.resourceDirectory[4095]='X';Refuses(bad);
  bad=Valid();std::strcpy(bad.resourceDirectory,"relative/path");Refuses(bad);
  for(const char* bytes : {"\xc0\xaf","\xed\xa0\x80","\xf4\x90\x80\x80", "\xe2\x82", "\x80", "\x01"}) {
    bad=Valid(); const auto n=std::strlen(bad.resourceDirectory);
    std::strcpy(bad.resourceDirectory+n,bytes);Refuses(bad);
  }
  SkagerChartPresentationStatusV1 s{};s.structBytes=sizeof(s);s.version=1;s.reserved[1]=1;
  auto unchanged=s;assert(!b.ReadStatus(&s));assert(!std::memcmp(&s,&unchanged,sizeof(s)));
  s={};s.structBytes=sizeof(s);s.version=2;assert(!b.ReadStatus(&s));
  s={};s.structBytes=sizeof(s);s.version=1;s.state=1;assert(!b.ReadStatus(&s));
  std::cout<<"PASS copied binding, malformed refusal, initialization and status boundaries\n";
}

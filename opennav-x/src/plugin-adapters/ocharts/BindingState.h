#pragma once
#include "plugin-adapters/ChartPresentationBindingV1.h"
#include <array>
#include <cstring>
#include <mutex>

namespace skager::ocharts {
// No wx/plugin/chart calls. The host owns the input memory only for this call.
class BindingState {
 public:
  bool Bind(const SkagerChartBindingV1* value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (bound_ || initialized_ || !value || value->structBytes != sizeof(*value) ||
        value->version != SKAGER_CHART_BINDING_VERSION) return false;
    for (auto v : value->reserved) if (v) return false;
    const auto* end = static_cast<const char*>(
        std::memchr(value->resourceDirectory, 0, sizeof(value->resourceDirectory)));
    if (!end || end == value->resourceDirectory) return false;
    const auto length = static_cast<size_t>(end - value->resourceDirectory);
    for (size_t i=length+1; i<sizeof(value->resourceDirectory); ++i)
      if (value->resourceDirectory[i]) return false;
    if (!Utf8(value->resourceDirectory, length) ||
        !AbsolutePath(value->resourceDirectory, length)) return false;
    std::memcpy(path_.data(), value->resourceDirectory, path_.size());
    bound_ = true;
    state_ = SKAGER_CHART_BOUND_PENDING_INITIALIZATION;
    reason_ = SKAGER_CHART_REASON_NONE;
    return true;
  }
  std::array<char, SKAGER_CHART_BINDING_PATH_CAPACITY> BeginInitialization() {
    std::lock_guard<std::mutex> lock(mutex_);
    initialized_ = true;
    return path_;
  }
  void Complete(bool selected, uint32_t reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!bound_) return;
    state_ = selected ? SKAGER_CHART_SELECTED : SKAGER_CHART_STANDARD_FALLBACK;
    reason_ = selected ? uint32_t(SKAGER_CHART_REASON_NONE) : reason;
  }
  bool ReadStatus(SkagerChartPresentationStatusV1* out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!out || out->structBytes != sizeof(*out) ||
        out->version != SKAGER_CHART_BINDING_VERSION || out->state || out->reason)
      return false;
    for (auto v : out->reserved) if (v) return false;
    SkagerChartPresentationStatusV1 result{};
    result.structBytes = sizeof(result); result.version = SKAGER_CHART_BINDING_VERSION;
    result.state = state_; result.reason = reason_;
    *out = result;
    return true;
  }
 private:
  static bool AbsolutePath(const char* p, size_t n) {
#ifdef _WIN32
    // Reject drive-relative, root-relative and device namespace paths. The
    // host supplies an ordinary fully qualified drive path or UNC directory.
    const auto slash = [](char c) { return c=='/' || c=='\\'; };
    if (n>=3 && ((p[0]>='A' && p[0]<='Z') || (p[0]>='a' && p[0]<='z')) &&
        p[1]==':' && slash(p[2])) return true;
    if(n<5 || !slash(p[0]) || !slash(p[1]) || p[2]=='?' || p[2]=='.') return false;
    size_t server=2;
    while(server<n && !slash(p[server])) ++server;
    return server>2 && server+1<n && !slash(p[server+1]);
#else
    return n>0 && p[0]=='/';
#endif
  }
  static bool Utf8(const char* text, size_t n) {
    for (size_t i=0; i<n;) {
      const auto c=static_cast<unsigned char>(text[i++]);
      if (c<0x20 || c==0x7f) return false;
      if (c<0x80) continue;
      unsigned code=0, count=0, minimum=0;
      if(c>=0xc2 && c<=0xdf) {code=c&31;count=1;minimum=0x80;}
      else if(c>=0xe0 && c<=0xef) {code=c&15;count=2;minimum=0x800;}
      else if(c>=0xf0 && c<=0xf4) {code=c&7;count=3;minimum=0x10000;}
      else return false;
      if(i+count>n) return false;
      while(count--) {
        const auto d=static_cast<unsigned char>(text[i++]);
        if((d&0xc0)!=0x80) return false;
        code=(code<<6)|(d&63);
      }
      if(code<minimum || code>0x10ffff || (code>=0xd800 && code<=0xdfff)) return false;
    }
    return true;
  }
  mutable std::mutex mutex_;
  std::array<char, SKAGER_CHART_BINDING_PATH_CAPACITY> path_{};
  bool bound_=false, initialized_=false;
  uint32_t state_=SKAGER_CHART_UNBOUND, reason_=SKAGER_CHART_REASON_UNBOUND;
};
} // namespace skager::ocharts

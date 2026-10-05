#pragma once
#include "XNavChartResources.h"
#include "sha256.h"
#include <wx/ffile.h>
#include <wx/filename.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
namespace skager::ocharts {
inline bool VerifyCompiledResources(const wxString& directory) {
  if (!wxFileName(directory, "").IsAbsolute()) return false;
  for (const auto& resource : opennav::chart_style::generated::resources) {
    wxFFile file(wxFileName(directory, wxString::FromUTF8(resource.name)).GetFullPath(), "rb");
    if (!file.IsOpened() || file.Length()<0 ||
        static_cast<std::uint64_t>(file.Length()) != resource.bytes) return false;
    SHA256_CTX hash; sha256_init(&hash);
    std::array<unsigned char,8192> buffer{};
    std::uint64_t remaining=resource.bytes;
    while(remaining) {
      const auto count=static_cast<size_t>((std::min)(remaining, std::uint64_t(buffer.size())));
      if(file.Read(buffer.data(),count)!=count) return false;
      sha256_update(&hash,buffer.data(),count);remaining-=count;
    }
    std::array<unsigned char,32> digest{};sha256_final(&hash,digest.data());
    static const char hex[]="0123456789abcdef";
    std::string actual;actual.reserve(64);
    for(auto byte:digest) {actual+=hex[byte>>4];actual+=hex[byte&15];}
    if(actual!=resource.sha256) return false;
  }
  return true;
}
} // namespace skager::ocharts

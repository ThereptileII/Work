#pragma once
#include <algorithm>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace opennav::integration {
// Bounded PE32 inspection before LoadLibrary. Executable trust still comes only
// from the compiled package hash checked by LoadOChartsModule, never this parser.
inline bool ChartModulePe(const std::vector<unsigned char>& b,
                          std::vector<std::string>& imports) {
  try {
    auto u = [&](std::size_t at, std::size_t n) -> std::uint32_t {
      if (at > b.size() || n > b.size()-at) throw std::runtime_error("PE bounds");
      std::uint32_t value=0;
      for(std::size_t i=0;i<n;++i) value |= std::uint32_t(b[at+i]) << (8*i);
      return value;
    };
    if(b.size()>128*1024*1024 || u(0,2)!=0x5a4d) return false;
    const std::size_t pe=u(60,4), opt=pe+24;
    if(u(pe,4)!=0x4550 || u(pe+4,2)!=0x14c || !(u(pe+22,2)&0x2000) ||
       u(opt,2)!=0x10b || u(opt+92,4)<16) return false;
    const auto count=u(pe+6,2), optional=u(pe+20,2);
    if(count<1 || count>96 || optional<224) return false;
    auto offset = [&](std::uint32_t rva, std::size_t length) -> std::size_t {
      for(std::size_t i=0;i<count;++i) {
        const auto s=opt+optional+40*i;
        const std::uint64_t start=u(s+12,4), size=(std::min)(u(s+8,4),u(s+16,4));
        if(rva>=start && std::uint64_t(rva)-start+length<=size) {
          const std::uint64_t at=u(s+20,4)+std::uint64_t(rva)-start;
          if(at<=b.size() && length<=b.size()-at) return std::size_t(at);
        }
      }
      throw std::runtime_error("PE RVA");
    };
    auto name = [&](std::uint32_t rva) {
      std::string result;
      for(unsigned n=0;n<512;++n) {
        if(rva>UINT32_MAX-n) throw std::runtime_error("PE name overflow");
        const auto c=u(offset(rva+n,1),1);
        if(!c) return result;
        if(c<32 || c>126) throw std::runtime_error("PE non-ASCII name");
        result+=char(c);
      }
      throw std::runtime_error("PE name bounds");
    };
    // Delay imports defeat proof that every host import resolved at load time.
    if(u(opt+96+13*8,4) || u(opt+100+13*8,4)) return false;
    const auto table=u(opt+104,4), size=u(opt+108,4);
    if(!table || size<20 || size>65536) return false;
    const std::set<std::string> allowed={
      "opencpn.exe","libcurl.dll","zlib1.dll","glew32.dll","gdiplus.dll",
      "kernel32.dll","user32.dll","gdi32.dll","advapi32.dll","shell32.dll",
      "ole32.dll","oleaut32.dll","comdlg32.dll","comctl32.dll","winspool.drv",
      "ws2_32.dll","rpcrt4.dll","uuid.dll","version.dll","shlwapi.dll",
      "opengl32.dll","glu32.dll","msvcp140.dll","vcruntime140.dll","ucrtbase.dll",
      "wxbase32u_vc14x.dll","wxbase32u_net_vc14x.dll","wxbase32u_xml_vc14x.dll",
      "wxmsw32u_core_vc14x.dll","wxmsw32u_adv_vc14x.dll","wxmsw32u_aui_vc14x.dll",
      "wxmsw32u_html_vc14x.dll","wxmsw32u_stc_vc14x.dll","wxmsw32u_gl_vc14x.dll"};
    const std::set<std::string> crt={"runtime","stdio","heap","string","math","convert",
      "time","locale","environment","filesystem","utility","conio","multibyte","process"};
    imports.clear(); bool end=false;
    for(unsigned n=0;n<size/20;++n) {
      if(table>UINT32_MAX-n*20) return false;
      const auto at=offset(table+n*20,20);
      if(!(u(at,4)|u(at+4,4)|u(at+8,4)|u(at+12,4)|u(at+16,4))) {end=true;break;}
      auto dll=name(u(at+12,4));
      for(auto& c:dll) if(c>='A'&&c<='Z')c+=32;
      bool permitted=allowed.count(dll)!=0;
      for(const auto& item:crt) permitted |= dll=="api-ms-win-crt-"+item+"-l1-1-0.dll";
      if(!permitted || std::find(imports.begin(),imports.end(),dll)!=imports.end())return false;
      imports.push_back(dll);
    }
    if(!end || std::find(imports.begin(),imports.end(),"opencpn.exe")==imports.end() ||
       std::find(imports.begin(),imports.end(),"wxmsw32u_core_vc14x.dll")==imports.end())return false;
    const auto exports=u(opt+96,4), exportSize=u(opt+100,4);
    if(!exports || exportSize<40) return false;
    const auto at=offset(exports,40);
    if(u(at+20,4)!=4 || u(at+24,4)!=4)return false;
    const auto addresses=u(at+28,4), names=u(at+32,4), ordinals=u(at+36,4);
    const auto addr=offset(addresses,16), namesAt=offset(names,16), ord=offset(ordinals,8);
    std::set<std::string> found;std::set<unsigned> slots;
    for(unsigned n=0;n<4;++n) {
      found.insert(name(u(namesAt+4*n,4)));
      const auto slot=u(ord+2*n,2);if(slot>=4 || !slots.insert(slot).second)return false;
      const auto entry=u(addr+4*slot,4);
      if(!entry || (entry>=exports && std::uint64_t(entry)<std::uint64_t(exports)+exportSize))return false;
      offset(entry,1);
    }
    return found==std::set<std::string>{"create_pi","destroy_pi",
      "skager_bind_chart_presentation_v1","skager_chart_presentation_status_v1"};
  } catch(const std::exception&) { return false; }
}
} // namespace opennav::integration

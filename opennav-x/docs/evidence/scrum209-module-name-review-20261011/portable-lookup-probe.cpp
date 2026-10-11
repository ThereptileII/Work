// Portable callback seam probe; no native file, device or loader operation.
// Synthetic PE helpers copied from tests/win32_pe_imports_test.cpp.
#include "integration/Win32PluginTlsImports.h"
#include <iostream>
#include <string>
#include <vector>
using Bytes = std::vector<unsigned char>;
void Put(Bytes& b, std::size_t at, std::uint32_t value, unsigned n = 4) {
  for (unsigned i = 0; i < n; ++i) b.at(at + i) = static_cast<unsigned char>(value >> (8 * i));
}
void Name(Bytes& b, std::size_t at, const std::string& text) {
  for (const auto c : text) b.at(at++) = static_cast<unsigned char>(c);
  b.at(at) = 0;
}
Bytes Image(const std::vector<std::string>& normal = {},
            const std::vector<std::string>& delayed = {}, bool va = false) {
  Bytes b(0x1200);
  Put(b, 0, 0x5a4d, 2); Put(b, 60, 0x80); Put(b, 0x80, 0x4550);
  Put(b, 0x84, 0x14c, 2); Put(b, 0x86, 1, 2); Put(b, 0x94, 224, 2);
  Put(b, 0x96, 0x2102, 2); Put(b, 0x98, 0x10b, 2);
  Put(b, 0xb4, 0x400000); Put(b, 0xb8, 0x1000); Put(b, 0xbc, 0x200);
  Put(b, 0xd0, 0x2000); Put(b, 0xd4, 0x200); Put(b, 0xf4, 16);
  Name(b, 0x178, ".rdata");
  Put(b, 0x180, 0x1000); Put(b, 0x184, 0x1000);
  Put(b, 0x188, 0x1000); Put(b, 0x18c, 0x200); Put(b, 0x19c, 0x40000040);
  if (!normal.empty()) {
    Put(b, 0x100, 0x1100); Put(b, 0x104, static_cast<std::uint32_t>(20 * (normal.size() + 1)));
    for (std::size_t i = 0; i < normal.size(); ++i) {
      Put(b, 0x300 + 20 * i, 0x1e00); Put(b, 0x310 + 20 * i, 0x1e80);
      Put(b, 0x30c + 20 * i, static_cast<std::uint32_t>(0x1500 + 0x80 * i));
      Name(b, 0x700 + 0x80 * i, normal[i]);
    }
  }
  if (!delayed.empty()) {
    Put(b, 0x160, 0x1200); Put(b, 0x164, static_cast<std::uint32_t>(32 * (delayed.size() + 1)));
    for (std::size_t i = 0; i < delayed.size(); ++i) {
      Put(b, 0x400 + 32 * i, va ? 0 : 1);
      Put(b, 0x404 + 32 * i, static_cast<std::uint32_t>(0x1900 + 0x80 * i + (va ? 0x400000 : 0)));
      Name(b, 0xb00 + 0x80 * i, delayed[i]);
    }
  }
  return b;
}

void Probe(const char* label, const Bytes& root, const std::string& sibling = {}) {
  unsigned calls = 0;
  std::string requested;
  const auto result = opennav::integration::RefuseLegacyPluginLocalTlsClosure(
      root, [&](const std::string& name) {
        ++calls;
        requested = name;
        return !sibling.empty() && name == sibling
            ? opennav::integration::PluginTlsLocalModule{true, Image({"libeay32.dll"}), {}}
            : opennav::integration::PluginTlsLocalModule{};
      });
  std::cout << label << "|calls=" << calls << "|requested=" << requested
            << "|refusal=" << result << '\n';
}
int main() {
  Probe("reserved-normal", Image({"CoM1.dll.backup"}));
  Probe("reserved-delay", Image({}, {"NUL   .dll"}));
  Probe("mixed-normal-explicit", Image({"MiXeD.DLL"}), "MiXeD.DLL");
  Probe("mixed-delay-explicit", Image({}, {"MiXeD.DLL"}), "MiXeD.DLL");
  Probe("mixed-normal-extensionless", Image({"MiXeD"}), "MiXeD.dll");
  Probe("mixed-delay-extensionless", Image({}, {"MiXeD"}), "MiXeD.dll");
  Probe("ordinary-name", Image({"COM10.dll"}));
}

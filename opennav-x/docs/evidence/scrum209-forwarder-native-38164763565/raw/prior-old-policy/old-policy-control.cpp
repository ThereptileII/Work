#include "integration/Win32PluginTlsImports.h"
#include <iostream>
#include <map>
#include <stdexcept>

using Bytes = std::vector<unsigned char>;
using namespace opennav::integration;
namespace {
unsigned checks = 0;
void Check(bool value, const char* message) {
  ++checks;
  if (!value) throw std::runtime_error(message);
}
void Put(Bytes& b, std::size_t at, std::uint32_t value, unsigned n = 4) {
  for (unsigned i = 0; i < n; ++i) b.at(at + i) = static_cast<unsigned char>(value >> (8 * i));
}
void Name(Bytes& b, std::size_t at, const std::string& text) {
  for (const auto c : text) b.at(at++) = static_cast<unsigned char>(c);
  b.at(at) = 0;
}
Bytes Image(const std::vector<std::string>& targets = {}) {
  Bytes b(0x2200);
  Put(b, 0, 0x5a4d, 2); Put(b, 60, 0x80); Put(b, 0x80, 0x4550);
  Put(b, 0x84, 0x14c, 2); Put(b, 0x86, 1, 2); Put(b, 0x94, 224, 2);
  Put(b, 0x98, 0x10b, 2); Put(b, 0xb4, 0x400000);
  Put(b, 0xd4, 0x200); Put(b, 0xf4, 16);
  Put(b, 0x180, 0x2000); Put(b, 0x184, 0x1000);
  Put(b, 0x188, 0x2000); Put(b, 0x18c, 0x200);
  Put(b, 0xf8, 0x1100); Put(b, 0xfc, 0x800);
  Put(b, 0x310, 1); Put(b, 0x314, static_cast<std::uint32_t>(targets.size()));
  Put(b, 0x31c, targets.empty() ? 0 : 0x1140);
  for (std::size_t i = 0; i < targets.size(); ++i) {
    Put(b, 0x340 + i * 4, static_cast<std::uint32_t>(0x1180 + i * 0x80));
    Name(b, 0x380 + i * 0x80, targets[i]);
  }
  return b;
}
void Tests() {
  // Regression control: the old normal/delay-only reader admits this declared
  // TLS dependency even though no import descriptor refers to libeay32.
  const auto direct = Image({"libeay32.legacy_tls_fixture"});
  Check(RefuseLegacyPluginTlsImports(direct).find("export forwarder") != std::string::npos,
        "FAIL old-policy gap: declared libeay32 export forwarder admitted");
}
}
int main() {
  try {
    Tests();
    std::cout << "PASS: " << checks << " deterministic Win32 export-forwarder checks\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}

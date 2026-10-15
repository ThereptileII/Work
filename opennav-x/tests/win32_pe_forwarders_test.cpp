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
Bytes LargeExportImage(const std::string& target = {}) {
  // Exact directory size/EAT count observed in the retained Win32 wx core DLL.
  // Keep raw backing beyond the directory so end checks cannot rely on EOF.
  constexpr std::uint32_t directory_size = 0x11af00;
  constexpr std::uint32_t export_rva = 0x1100;
  constexpr std::uint32_t raw_size = export_rva - 0x1000 + directory_size + 0x200;
  auto b = Image();
  b.resize(0x200 + raw_size);
  Put(b, 0x180, raw_size); Put(b, 0x188, raw_size);
  Put(b, 0xfc, directory_size); Put(b, 0x314, 19522); Put(b, 0x31c, 0x1140);
  if (!target.empty()) {
    const auto rva = export_rva + directory_size - static_cast<std::uint32_t>(target.size()) - 1;
    Put(b, 0x340, rva);
    Name(b, 0x200 + rva - 0x1000, target);
  }
  return b;
}
Bytes Import(Bytes b, const std::string& name, bool delayed = false) {
  const auto directory = delayed ? 0x160 : 0x100;
  Put(b, directory, 0x1a00); Put(b, directory + 4, delayed ? 64 : 40);
  if (delayed) Put(b, 0xc00, 1);
  Put(b, delayed ? 0xc04 : 0xc0c, 0x1c00);
  Name(b, 0xe00, name);
  return b;
}
void Reject(const Bytes& b, const char* expected = "") {
  const auto r = ReadWin32PeImports(b);
  Check(!r.parsed && r.modules.empty() && r.forwarders.empty() && !r.error.empty(),
        "Malformed export metadata or partial declarations admitted");
  if (*expected) Check(r.error == expected, "Wrong export rejection boundary");
}
void Tests() {
  // Regression control: the old normal/delay-only reader admits this declared
  // TLS dependency even though no import descriptor refers to libeay32.
  const auto direct = Image({"libeay32.legacy_tls_fixture"});
  Check(RefuseLegacyPluginTlsImports(direct).find("export forwarder") != std::string::npos,
        "FAIL old-policy gap: declared libeay32 export forwarder admitted");
  for (const auto& target : {"libeay32.fn", "SSLEAY32.#209", "LiBeAy32.#65535",
                             "ssleay32 .fn", "C:libeay32.fn", "vendor/ssleay32.fn"}) {
    const auto image = Image({target}), before = image;
    const auto r = ReadWin32PeImports(image);
    Check(r.parsed && r.modules.empty() && r.forwarders == std::vector<std::string>{target},
          "Forwarder was lost, rewritten or conflated with imports");
    Check(RefuseLegacyPluginTlsImports(r).find("TLS export forwarder") != std::string::npos,
          "Legacy forwarder alias admitted");
    Check(image == before, "Forwarder inspection changed bytes");
  }
  for (const auto& target : {"helper.fn", "helper.#1", "libeay32backup.fn", "ssleay32_old.#209"})
    Check(RefuseLegacyPluginTlsImports(Image({target})).empty(), "Benign forwarder overmatched");
  for (const auto& target : {"libeay32.dll.fn", "helper.dll.fn", "helper.name.with.dots",
                             "helper", ".fn", "helper.", "helper.#", "helper.#0",
                             "helper.#65536", "helper.#99999999999999", "helper.#1x"}) {
    const auto r = ReadWin32PeImports(Image({target}));
    Check(r.parsed && r.forwarders.at(0) == target, "Reader imposed forwarder grammar policy");
    Check(RefuseLegacyPluginTlsImports(r).find("Unsupported plugin export forwarder") == 0,
          "Unsupported or ambiguous forwarder grammar admitted");
  }
  const auto closure = [](const Bytes& root, const std::map<std::string, Bytes>& files,
                          unsigned* reads = nullptr) {
    return RefuseLegacyPluginLocalTlsClosure(root, [&](const std::string& name) {
      if (reads) ++*reads;
      const auto found = files.find(name);
      return found == files.end() ? PluginTlsLocalModule{} :
          PluginTlsLocalModule{true, found->second, {}};
    });
  };
  // Only the supported extensionless module grammar reaches sibling lookup.
  // Device stems must be refused before that callback for named and ordinal exports.
  std::vector<std::string> devices{"CON", "pRn", "AuX", "nul", "ConIn$", "CONOUT$"};
  for (unsigned digit = 1; digit <= 9; ++digit) {
    devices.push_back("CoM" + std::to_string(digit));
    devices.push_back("lPt" + std::to_string(digit));
  }
  for (const auto& device : devices)
    for (const auto& symbol : {"fn", "#209"}) {
      unsigned calls = 0;
      const auto image = Image({device + "." + symbol}), before = image;
      Check(!closure(image, {}, &calls).empty() && calls == 0,
            "Reserved forwarded module reached sibling callback");
      Check(image == before, "Reserved forwarder refusal modified PE bytes");
    }
  for (const auto& target : {"CON.dll.fn", "COM1 .fn", "LPT9.one.two"}) {
    unsigned calls = 0;
    Check(!closure(Image({target}), {}, &calls).empty() && calls == 0,
          "Unsupported reserved forwarder grammar reached sibling callback");
  }
  for (const auto& name : {"CONsole", "COM0", "COM10", "LPT0", "LPT10", "ConIn$Extra"}) {
    unsigned calls = 0;
    Check(closure(Image({std::string(name) + ".fn"}),
                  {{std::string(name) + ".dll", Image()}}, &calls).empty() && calls == 1,
          "Ordinary forwarded module overmatched reserved device policy");
  }
  unsigned mixed_calls = 0;
  Check(closure(Image({"MiXeD.#209"}), {{"MiXeD.dll", Image({"libeay32.fn"})}}, &mixed_calls)
            .find("TLS export forwarder") != std::string::npos && mixed_calls == 1,
        "Original-cased forwarded sibling lookup skipped TLS-bearing file");
  for (const bool delayed : {false, true}) {
    Check(closure(Import(Image(), "helper.dll", delayed),
                  {{"helper.dll", Image({"libeay32.fn"})}}).find("TLS export forwarder") != std::string::npos,
          "Import to sibling forwarder bypassed TLS policy");
    Check(closure(Image({"helper.fn"}),
                  {{"helper.dll", Import(Image(), "ssleay32.dll", delayed)}})
                  .find(delayed ? "delay import" : "TLS import") != std::string::npos,
          "Forwarder to sibling import bypassed TLS policy");
  }
  Check(closure(Image({"helper.#209"}), {{"helper.dll", Image({"nested.fn"})},
                 {"nested.dll", Image({"ssleay32.#209"})}}).find("TLS export forwarder") != std::string::npos,
        "Two-hop ordinal/named forwarder admitted");
  unsigned reads = 0;
  Check(closure(Import(Image({"HeLpEr.#209", "helper.fn"}), "helper.dll"),
                {{"helper.dll", Image({"HELPER.#209"})}}, &reads).empty() && reads == 1,
        "Import/forwarder aliases or forwarder cycle re-read sibling");
  reads = 0;
  Check(closure(Image({"absent.fn"}), {}, &reads).empty() && reads == 1,
        "Absent forwarded sibling misrepresented as complete closure");
  Check(!closure(Image({"helper.fn"}), {{"helper.dll", Bytes(64)}}).empty(),
        "Malformed forwarded sibling admitted");
  Check(!closure(Image({"relative/helper.fn"}), {}).empty(), "Forwarder path followed outside sibling policy");
  Check(!RefuseLegacyPluginLocalTlsClosure(Image({"helper.fn"}), [](const std::string&) {
    return PluginTlsLocalModule{false, {}, "Unreadable forwarded sibling"};
  }).empty(), "Unreadable forwarded sibling admitted");
  std::map<std::string, Bytes> chain;
  for (unsigned i = 0; i < 257; ++i)
    chain.emplace("node" + std::to_string(i) + ".dll", Image({"node" + std::to_string(i + 1) + ".fn"}));
  Check(closure(Image({"node0.fn"}), chain).find("node budget") != std::string::npos,
        "Forwarder graph exceeded node budget");

  auto b = Image({"helper.fn"});
  Put(b, 0x314, 4); Put(b, 0x344, 0); Put(b, 0x348, 0x1000); Put(b, 0x34c, 0xffffffff);
  auto r = ReadWin32PeImports(b);
  Check(r.parsed && r.forwarders.size() == 1, "Holes/nonforwarded RVAs treated as dependency strings");
  b = Image(); Put(b, 0xf8, 0); Put(b, 0xfc, 0);
  Check(ReadWin32PeImports(b).parsed, "Export-free image refused");
  b = Image(); Put(b, 0xf4, 0);
  Check(ReadWin32PeImports(b).parsed, "Absent export directory refused");
  b = Image({"a.b"}); Put(b, 0x340, 0x18fc); Name(b, 0xafc, "a.b");
  Check(ReadWin32PeImports(b).parsed, "In-directory final-byte NUL refused");
  b = Image({"a.b"}); Put(b, 0x340, 0x18fd); Name(b, 0xafd, "a.b");
  Reject(b, "Export forwarder crosses directory boundary");

  const auto valid = Import(Image({"helper.fn", "libeay32.#209"}), "normal.dll");
  const auto bad = [&](std::size_t at, std::uint32_t value, const char* expected = "", unsigned n = 4) {
    auto changed = valid; Put(changed, at, value, n); Reject(changed, expected);
  };
  bad(0xf8, 0); bad(0xfc, 0); bad(0xfc, 39); bad(0xfc, 1048577);
  bad(0xf8, 0xfffffff0); bad(0xf8, 0x2fff); bad(0x31c, 0);
  bad(0x31c, 0xffffffff); bad(0x31c, 0x2fff); bad(0x31c, 0x1ff);
  bad(0x314, 65537, "Export address table budget exceeded");
  bad(0x188, 0x300); bad(0x184, 0xffffff00); bad(0x18c, 0x100);
  bad(0x380, 0, "Empty or oversized export forwarder", 1);
  bad(0x380, 0xff, "Non-ASCII export forwarder", 1);
  b = Image({"x.fn"}); Put(b, 0xfc, 0x1800);
  std::fill(b.begin() + 0x380, b.begin() + 0x1380, 'x');
  Reject(b, "Empty or oversized export forwarder");
  // Repeat one string RVA to exercise independent entry/output/name budgets.
  b = Image(); b.resize(0xa200); Put(b, 0x180, 0xa000); Put(b, 0x188, 0xa000);
  Put(b, 0xfc, 0x9000); Put(b, 0x314, 4097); Put(b, 0x31c, 0x1300);
  for (unsigned i = 0; i < 4097; ++i) Put(b, 0x500 + i * 4, 0x9000);
  Name(b, 0x8200, "helper.fn"); Reject(b, "Export forwarder budget exceeded");
  Put(b, 0x314, 257); Name(b, 0x8200, std::string(4095, 'x'));
  Reject(b, "Declaration name budget exceeded");
  // EAT size can be large without any forwarders; bound the scan itself too.
  b = Image(); b.resize(0x50000); Put(b, 0x180, 0x4fe00); Put(b, 0x188, 0x4fe00);
  Put(b, 0x314, 65536); Put(b, 0x31c, 0x2000);
  Check(ReadWin32PeImports(b).parsed, "Bounded all-hole export address table refused");

  // Large, fully backed export directories need no additional scan budget.
  b = LargeExportImage();
  r = ReadWin32PeImports(b);
  Check(r.parsed && r.forwarders.empty(), "Fully backed large export directory refused");
  b = LargeExportImage("helper.fn");
  r = ReadWin32PeImports(b);
  Check(r.parsed && r.forwarders == std::vector<std::string>{"helper.fn"},
        "Large-directory final-byte NUL or far forwarder refused");
  Check(RefuseLegacyPluginTlsImports(b).empty(), "Benign large export directory refused by TLS policy");
  const auto large_legacy = LargeExportImage("libeay32.#209");
  Check(RefuseLegacyPluginTlsImports(large_legacy).find("TLS export forwarder") != std::string::npos,
        "Large-directory legacy forwarder admitted");
  Check(closure(Image({"large.fn"}), {{"large.dll", large_legacy}})
            .find("TLS export forwarder") != std::string::npos,
        "Indirect large-directory legacy forwarder escaped closure");
  const auto large_bad = [&](std::size_t at, std::uint32_t value, const char* expected = "") {
    auto changed = b; Put(changed, at, value); Reject(changed, expected);
  };
  large_bad(0xf8, 0, "Invalid export directory extent");
  large_bad(0xfc, 39, "Invalid export directory extent");
  large_bad(0xfc, 0x11b101, "RVA has no complete raw backing");
  large_bad(0xfc, 0xffffffff, "Invalid RVA range");
  large_bad(0x188, 0x11afff, "RVA has no complete raw backing");
  large_bad(0x31c, 0x11bfff, "RVA has no complete raw backing");
  large_bad(0x314, 65537, "Export address table budget exceeded");
  // This NUL is still raw-backed, but one byte beyond the export directory.
  Put(b, 0x340, 0x11c000 - 9); Name(b, 0x11b200 - 9, "helper.fn");
  Reject(b, "Export forwarder crosses directory boundary");
  b = LargeExportImage("helper.fn"); b.resize(0x11b1ff);
  Reject(b, "Invalid section raw range");
  for (std::size_t size = 0; size < valid.size(); ++size)
    Reject(Bytes(valid.begin(), valid.begin() + size));
  for (std::size_t at = 0; at < valid.size(); ++at) {
    b = valid; b[at] ^= 0xff; r = ReadWin32PeImports(b);
    Check(r.parsed ? r.error.empty() : (!r.error.empty() && r.modules.empty() && r.forwarders.empty()),
          "Mutation returned partial export/import declarations");
  }
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

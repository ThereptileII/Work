#include "integration/Win32PeImports.h"
#include "integration/Win32PluginTlsImports.h"
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

using Bytes = std::vector<unsigned char>;
using opennav::integration::ReadWin32PeImports;
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
void Reject(const Bytes& b, const char* context, const char* expected = "") {
  const auto result = ReadWin32PeImports(b);
  Check(!result.parsed && result.modules.empty() && result.forwarders.empty() && !result.error.empty(), context);
  if (*expected) Check(result.error == expected, "Wrong rejection boundary");
}
// Both roots share a work queue and limits; the fake filesystem only supplies
// deterministic bytes. Native tests separately establish mapping and file locks.
void MultiRootTests() {
  using namespace opennav::integration;
  using Key = std::pair<std::size_t, std::string>;
  using Files = std::map<Key, Bytes>;
  const auto walk = [](const Bytes& root, const Files& files,
                       PluginTlsClosureLimits limits = {},
                       std::vector<Key>* reads = nullptr,
                       const std::vector<std::string>& identities = {"plugin", "host"}) {
    return RefuseLegacyPluginTlsClosure(root, identities,
        [&](std::size_t index, const std::string& name) {
          if (reads) reads->emplace_back(index, name);
          const auto found = files.find({index, name});
          return found == files.end() ? PluginTlsLocalModule{} :
              PluginTlsLocalModule{true, found->second, {}};
        }, [](const Bytes&) { return std::string{}; }, limits);
  };
  for (const bool delayed : {false, true})
    for (const bool va : {false, true}) {
      const auto root = delayed ? Image({}, {"helper.dll"}, va) : Image({"helper.dll"});
      const auto before = root;
      std::vector<Key> reads;
      Check(walk(root, {{{1, "helper.dll"}, Image({"libeay32.dll"})}}, {}, &reads)
                .find("TLS import") != std::string::npos,
            "Host-only normal/delay legacy dependency escaped shared graph");
      Check(reads == std::vector<Key>{{0, "helper.dll"}, {1, "helper.dll"}},
            "Declared basename did not inspect both roots");
      Check(root == before, "Shared closure modified root bytes");
      Check(walk(root, {{{1, "helper.dll"}, Image()}}).empty(),
            "Benign host-only dependency refused");
      reads.clear();
      Check(walk(root, {{{0, "helper.dll"}, Image()},
                        {{1, "helper.dll"}, Image({}, {"ssleay32.dll"}, va)}}, {}, &reads)
                .find("delay import") != std::string::npos && reads.size() == 2,
            "Clean plugin sibling masked unsafe host candidate");
    }
  // An independent per-directory walk misses hostB's pluginC. Every found
  // candidate must send its declarations back through both roots of one graph.
  const Files crossing{{{0, "pluginA.dll"}, Image({"hostB.dll"})},
                       {{1, "hostB.dll"}, Image({}, {"pluginC.dll"})},
                       {{0, "pluginC.dll"}, Image({"libeay32.dll"})}};
  Check(walk(Image({"pluginA.dll"}), crossing).find("TLS import") != std::string::npos,
        "Plugin-to-host-to-plugin legacy chain escaped shared graph");
  std::vector<Key> reads;
  const Files cyclic{{{0, "MiXeD.dll"}, Image({"MiXeD.dll"})},
                     {{1, "MiXeD.dll"}, Image({}, {"MIXED.DLL"})}};
  Check(walk(Image({"MiXeD", "MIXED.DLL"}), cyclic, {}, &reads).empty() &&
            reads == std::vector<Key>{{0, "MiXeD.dll"}, {1, "MiXeD.dll"}},
        "Case/extension aliases or cross-root cycles were not deduplicated per root");
  reads.clear();
  Check(walk(Image({"helper.dll"}), {{{0, "helper.dll"}, Image()}}, {}, &reads,
             {"same-directory", "same-directory"}).empty() && reads.size() == 1,
        "Aliases of the same directory repeated a candidate read");
  Check(!walk(Image({"helper.dll"}), {{{1, "helper.dll"}, Bytes(64)}}).empty(),
        "Malformed host candidate admitted");
  reads.clear();
  Check(walk(Image({"absent.dll"}), {}, {}, &reads).empty() && reads.size() == 2,
        "Missing candidate skipped another root or falsely failed inspection");
  const auto unreadable = RefuseLegacyPluginTlsClosure(Image({"helper.dll"}),
      {"plugin", "host"}, [](std::size_t index, const std::string&) {
        return index == 0 ? PluginTlsLocalModule{true, Image(), {}} :
            PluginTlsLocalModule{false, {}, "Unreadable host candidate"};
      }, [](const Bytes&) { return std::string{}; });
  Check(unreadable == "Unreadable host candidate", "Clean sibling masked host read error");
  // Upstream's Libdirs can resolve a declared helper outside both the selected
  // nested plugin directory and the host. All roots still share one queue.
  const std::vector<std::string> upstream_roots{"nested-plugin", "host", "user-plugin-base", "app-plugin-base"};
  Check(walk(Image({"shared.dll"}), {{{2, "shared.dll"}, Image({"libeay32.dll"})}},
             {}, nullptr, upstream_roots).find("TLS import") != std::string::npos,
        "Declared helper in upstream plugin base escaped inspection");
  Check(walk(Image({"shared.dll"}), {{{2, "shared.dll"}, Image()}},
             {}, nullptr, upstream_roots).empty(), "Benign upstream-only helper refused");
  Check(walk(Image({"shared.dll"}), {{{0, "shared.dll"}, Image()},
                                     {{2, "shared.dll"}, Image({"next.dll"})},
                                     {{3, "next.dll"}, Image({}, {"ssleay32.dll"})}},
             {}, nullptr, upstream_roots).find("delay import") != std::string::npos,
        "Clean sibling masked chain crossing two upstream plugin roots");
  reads.clear();
  Check(walk(Image({"shared.dll", "SHARED.DLL"}), {}, {}, &reads,
             {"nested-plugin", "host", "same-base", "same-base"}).empty() && reads.size() == 3,
        "Repeated upstream directory identity repeated missing-candidate probes");
  // Small injected budgets prove shared accounting without allocating 256 MiB.
  reads.clear();
  Check(walk(Image({"one.dll", "two.dll"}), {}, {3, 256u * 1024u * 1024u}, &reads)
            .find("node budget") != std::string::npos && reads.size() == 3,
        "Missing multi-root candidates escaped shared node bound");
  reads.clear();
  Check(walk(Image({"helper.dll"}), {{{0, "helper.dll"}, Image()},
                                     {{1, "helper.dll"}, Image()}},
             {256, 3 * Image().size()}, &reads).empty() && reads.size() == 2,
        "Exact aggregate byte limit refused benign candidates");
  Check(walk(Image({"helper.dll"}), {{{0, "helper.dll"}, Image()},
                                     {{1, "helper.dll"}, Image()}},
             {256, 3 * Image().size() - 1}).find("byte budget") != std::string::npos,
        "Candidate bytes from both roots escaped shared byte bound");
  reads.clear();
  Check(walk(Image(), {}, {256, Image().size() - 1}, &reads).find("byte budget") !=
            std::string::npos && reads.empty(), "Oversized root bypassed byte bound");
  Check(!walk(Image(), {}, {}, nullptr, {}).empty() &&
        !walk(Image(), {}, {}, nullptr, {"plugin", ""}).empty(),
        "Missing root identity admitted");
}
void Tests() {
  // Exercise the production refusal policy with actual normal/delay PE bytes.
  using opennav::integration::RefuseLegacyPluginTlsImports;
  for (const auto& name : {"libeay32.dll", "SSLEAY32.DLL", "LiBeAy32", "ssleay32.",
                           "C:\\vendor\\libeay32.dll", "relative/ssleay32.dll",
                           "libeay32.dll. ", "ssleay32.dll:stream", "C:libeay32.dll"}) {
    for (const bool delayed : {false, true}) {
      for (const bool va : {false, true}) {
        const auto image = delayed ? Image({}, {name}, va) : Image({name});
        const auto before = image;
        const auto refusal = RefuseLegacyPluginTlsImports(image);
        Check(refusal.find(delayed ? "delay import: " : "TLS import: ") != std::string::npos,
              "Legacy TLS import reached allow branch");
        Check(image == before, "TLS guard modified bytes");
      }
    }
  }
  Check(RefuseLegacyPluginTlsImports(Image({"libssl-3.dll", "libcrypto-3.dll", "libcurl.dll"},
                                        {"vendor.dll", "libeay32.dll.backup"})).empty(),
        "Ordinary imports refused or legacy basename overmatched");
  Check(RefuseLegacyPluginTlsImports(Image()).empty(), "Import-free plugin refused");
  auto invalid = Image(); Put(invalid, 0x84, 0x8664, 2);
  Check(!RefuseLegacyPluginTlsImports(invalid).empty(), "Foreign machine admitted by TLS policy");
  invalid = Image({"libeay32.dll"}); Put(invalid, 0x104, 19);
  Check(RefuseLegacyPluginTlsImports(invalid).find("Cannot inspect") == 0,
        "Malformed import directory admitted by TLS policy");
  // Ordinary Win32 plugin metadata: no adapter exports or fixed import allowlist.
  const std::vector<std::string> normal{"wxmsw32u_core_vc14x.dll", "libcurl.dll", "Vendor-SDK_2.dll", "KERNEL32.DLL"};
  const std::vector<std::string> delayed{"winspool.drv", "optional decoder.dll", "libcurl.dll"};
  for (const bool va : {false, true}) {
    const auto bytes = Image(normal, delayed, va), before = bytes;
    const auto result = ReadWin32PeImports(bytes);
    Check(result.parsed && result.error.empty() && result.modules.size() == 7, "Ordinary plugin imports refused");
    Check(bytes == before, "Input bytes changed");
    for (std::size_t i = 0; i < normal.size(); ++i)
      Check(result.modules[i].name == normal[i] && !result.modules[i].delayed, "Normal import changed");
    for (std::size_t i = 0; i < delayed.size(); ++i)
      Check(result.modules[i + normal.size()].name == delayed[i] && result.modules[i + normal.size()].delayed, "Delay import changed");
  }
  Check(ReadWin32PeImports(Image()).parsed, "Import-free image refused");
  // The production local-closure policy must reach indirect imports while
  // keeping absent siblings explicitly outside this narrow proof.
  using opennav::integration::PluginTlsLocalModule;
  using opennav::integration::RefuseLegacyPluginLocalTlsClosure;
  const auto closure = [](const Bytes& root, const std::map<std::string, Bytes>& files,
                          unsigned* reads = nullptr) {
    return RefuseLegacyPluginLocalTlsClosure(root, [&](const std::string& name) {
      if (reads) ++*reads;
      const auto found = files.find(name);
      return found == files.end() ? PluginTlsLocalModule{} :
             PluginTlsLocalModule{true, found->second, {}};
    });
  };
  // Reserved device basenames must never reach a callback that opens a file.
  std::vector<std::string> devices{"CON", "pRn", "AuX", "nul", "ConIn$", "CONOUT$"};
  for (unsigned digit = 1; digit <= 9; ++digit) {
    devices.push_back("CoM" + std::to_string(digit));
    devices.push_back("lPt" + std::to_string(digit));
  }
  for (const auto& device : devices)
    for (const auto& suffix : {"", ".dll", ".DLL.backup", " .dll", "   .one.two"})
      for (const bool delayed : {false, true}) {
        unsigned calls = 0;
        const auto name = device + suffix;
        const auto root = delayed ? Image({}, {name}) : Image({name});
        const auto before = root;
        Check(!closure(root, {}, &calls).empty() && calls == 0,
              "Reserved normal/delay module reached sibling callback");
        Check(root == before, "Reserved-module refusal modified PE bytes");
      }
  // Superscript DOS-device digit forms cannot cross the ASCII-only PE reader.
  for (const auto& prefix : {"COM", "LPT"})
    for (const auto& digit : {std::string(1, static_cast<char>(0xb9)),
                              std::string(1, static_cast<char>(0xb2)),
                              std::string(1, static_cast<char>(0xb3)),
                              std::string("\xc2\xb9"), std::string("\xc2\xb2"), std::string("\xc2\xb3")})
      for (const bool delayed : {false, true}) {
        unsigned calls = 0;
        const auto name = prefix + digit + ".dll";
        Check(closure(delayed ? Image({}, {name}) : Image({name}), {}, &calls)
                  .find("Non-ASCII import module name") != std::string::npos && calls == 0,
              "Superscript device alias crossed ASCII reader or reached callback");
      }
  for (const auto& name : {"CONsole.dll", "PRNter.dll", "AUXiliary.dll", "NULler.dll",
                           "COM0.dll", "COM10.dll", "LPT0.dll", "LPT10.dll",
                           "ConIn$Extra.dll", "CONOUT$Extra.dll", "optional decoder.dll"}) {
    unsigned calls = 0;
    Check(closure(Image({name}), {{name, Image()}}, &calls).empty() && calls == 1,
          "Ordinary module overmatched reserved device policy");
  }
  // A case-sensitive fake reader models only exact sibling lookup, not native
  // LoadLibrary. Lowercasing these names previously skipped the TLS-bearing file.
  for (const bool delayed : {false, true})
    for (const auto& name : {"MiXeD.DLL", "MiXeD"}) {
      unsigned calls = 0;
      Check(closure(delayed ? Image({}, {name}) : Image({name}),
                    {{name == std::string("MiXeD") ? "MiXeD.dll" : name,
                      Image({"libeay32.dll"})}}, &calls).find("TLS import") != std::string::npos && calls == 1,
            "Original-cased normal/delay sibling lookup skipped TLS-bearing file");
    }
  Check(closure(Image({"helper.dll"}), {{"helper.dll", Image({"libeay32.dll"})}})
            .find("TLS import") != std::string::npos,
        "Indirect normal legacy TLS import admitted");
  Check(closure(Image({"helper"}), {{"helper.dll", Image({"libeay32.dll"})}})
            .find("TLS import") != std::string::npos,
        "Extensionless indirect normal import bypassed local DLL");
  Check(closure(Image({}, {"helper"}), {{"helper.dll", Image({"libeay32.dll"})}})
            .find("TLS import") != std::string::npos,
        "Extensionless indirect delay import bypassed local DLL");
  Check(closure(Image({"helper.dll"}), {{"helper.dll", Image({}, {"ssleay32.dll"})}})
            .find("delay import") != std::string::npos,
        "Indirect delayed legacy TLS import admitted");
  Check(closure(Image({"helper.dll"}), {{"helper.dll", Image({"nested.dll"})},
                                         {"nested.dll", Image({"libeay32.dll"})}})
            .find("TLS import") != std::string::npos,
        "Two-hop local legacy TLS import admitted");
  Check(closure(Image({"helper.dll"}), {{"helper.dll", Image({"KERNEL32.dll"})}}).empty(),
        "Benign local dependency refused");
  unsigned reads = 0;
  Check(closure(Image({"helper.dll", "helper.dll"}),
                {{"helper.dll", Image({"helper.dll"})}}, &reads).empty() && reads == 1,
        "Repeated or cyclic local import was re-read");
  reads = 0;
  Check(closure(Image({"helper", "helper.dll"}),
                {{"helper.dll", Image({})}}, &reads).empty() && reads == 1,
        "Extensionless and explicit DLL alias was inspected twice");
  reads = 0;
  Check(closure(Image({"absent.dll"}), {}, &reads).empty() && reads == 1,
        "Missing sibling incorrectly treated as inspected closure");
  Check(!closure(Image({"helper.dll"}), {{"helper.dll", Bytes(64)}}).empty(),
        "Malformed local dependency admitted");
  Check(!RefuseLegacyPluginLocalTlsClosure(Image({"helper.dll"}),
      [](const std::string&) { return PluginTlsLocalModule{false, {}, "Unreadable sibling"}; }).empty(),
      "Unreadable local dependency admitted");
  Check(!closure(Image({"relative/helper.dll"}), {}).empty(),
        "Qualified import module admitted to sibling-only closure");
  std::map<std::string, Bytes> chain;
  for (unsigned i = 0; i < 257; ++i)
    chain.emplace("node" + std::to_string(i) + ".dll",
                  Image(i == 256 ? std::vector<std::string>{} :
                        std::vector<std::string>{"node" + std::to_string(i + 1) + ".dll"}));
  Check(closure(Image({"node0.dll"}), chain).find("node budget") != std::string::npos,
        "Unbounded local dependency walk admitted");
  auto b = Image(); Put(b, 0xf4, 0);
  Check(ReadWin32PeImports(b).parsed, "Absent directory entries refused");
  b = Image({"libeay32.dll", "libeay32.dll"}, {"ssleay32.dll"});
  auto result = ReadWin32PeImports(b);
  Check(result.parsed && result.modules.size() == 3, "Reader imposed trust policy or deduplicated imports");
  b = Image({"relative/path.dll"});
  Check(ReadWin32PeImports(b).modules.at(0).name == "relative/path.dll", "Reader imposed path policy");
  // Module-name RVAs may point into mapped headers.
  b = Image({"x.dll"}); Name(b, 0x1d0, "header.dll"); Put(b, 0x30c, 0x1d0);
  Check(ReadWin32PeImports(b).modules.at(0).name == "header.dll", "Header name RVA refused");
  // Mutations target each structure used to find names, not unrelated PE data.
  const auto valid = Image({"kernel32.dll"}, {"optional.dll"});
  const auto bad = [&](std::size_t at, std::uint32_t value, const char* message, unsigned n = 4) {
    auto changed = valid; Put(changed, at, value, n); Reject(changed, message);
  };
  bad(0, 0, "DOS signature accepted", 2);
  bad(60, 0xfffffff0, "Unbounded PE offset accepted");
  bad(0x80, 0, "PE signature accepted");
  bad(0x84, 0x8664, "Non-x86 accepted", 2);
  bad(0x98, 0x20b, "PE32+ accepted", 2);
  bad(0x86, 97, "Excessive sections accepted", 2);
  bad(0x86, 0, "No sections accepted", 2);
  bad(0x94, 95, "Short optional header accepted", 2);
  bad(0x94, 0xffff, "Truncated optional header accepted", 2);
  bad(0xf4, 17, "Directories outside optional header accepted");
  bad(0xd4, 0x180, "Header missing section table accepted");
  bad(0x188, 0x1001, "Raw section beyond file accepted");
  bad(0x18c, 0x100, "Raw section aliases header");
  bad(0x184, 0xffffff00, "Overflowing virtual section accepted");
  bad(0x184, 0x100, "Virtual section aliases header");
  bad(0x100, 0, "Missing import RVA accepted");
  bad(0x104, 0, "Missing import size accepted");
  bad(0x104, 19, "Undersized descriptor table accepted");
  bad(0x104, 1048577, "Oversized directory accepted");
  bad(0x100, 0x1fff, "Cross-section directory accepted");
  bad(0x30c, 0, "Zero name RVA accepted");
  bad(0x30c, 0xffffffff, "Unmapped name RVA accepted");
  bad(0x700, 0, "Empty name accepted", 1);
  bad(0x700, 0xff, "Non-ASCII name accepted", 1);
  bad(0x104, 20, "Missing normal terminator accepted");
  bad(0x164, 32, "Missing delay terminator accepted");
  bad(0x400, 2, "Unknown delay attributes accepted");
  b = valid; Put(b, 0x400, 0); Reject(b, "Delay VA below image base accepted");
  b = valid; Put(b, 0x30c, 0x1fff); b.back() = 'x'; Reject(b, "Name without backed terminator accepted");
  b = valid; Put(b, 0x188, 0x200); Reject(b, "Import name in zero fill accepted");
  b = valid; Put(b, 0x86, 2, 2);
  Put(b, 0x1a8, 0x100); Put(b, 0x1ac, 0x1100); Reject(b, "Overlapping virtual sections accepted");
  b = valid; Put(b, 0x86, 2, 2);
  Put(b, 0x1a8, 0x100); Put(b, 0x1ac, 0x3000);
  Put(b, 0x1b0, 0x100); Put(b, 0x1b4, 0x200); Reject(b, "Overlapping raw sections accepted");
  b = Image({"a.dll"}); b.resize(0x4000);
  Put(b, 0x180, 0x3e00); Put(b, 0x188, 0x3e00);
  std::fill(b.begin() + 0x700, b.begin() + 0x1700, 'a');
  Reject(b, "Oversized name accepted", "Unterminated or oversized import module name");
  // Repeated name RVAs cannot amplify a small image into unlimited output.
  b = Image(); b.resize(0x20000);
  Put(b, 0x180, 0x1fe00); Put(b, 0x188, 0x1fe00);
  Put(b, 0x100, 0x1100); Put(b, 0x104, 20 * 4098);
  Name(b, 0x16000, "shared.dll");
  for (unsigned i = 0; i < 4097; ++i) Put(b, 0x30c + 20 * i, 0x16e00);
  Reject(b, "Excessive descriptors accepted", "Import descriptor budget exceeded");
  std::fill(b.begin() + 0x300, b.end(), 0);
  Put(b, 0x104, 20 * 258);
  Name(b, 0x16000, std::string(4095, 'a'));
  for (unsigned i = 0; i < 257; ++i) Put(b, 0x30c + 20 * i, 0x16e00);
  Reject(b, "Excessive total name bytes accepted", "Import name budget exceeded");
  for (std::size_t size = 0; size < valid.size(); ++size)
    Reject(Bytes(valid.begin(), valid.begin() + size), "Truncation accepted");
  // Reproducible mutation safety sweep, useful under ASan/UBSan. No acceptance
  // oracle for unrelated fields; every rejection must discard partial names.
  for (std::size_t at = 0; at < valid.size(); ++at) {
    b = valid; b[at] ^= 0xff; result = ReadWin32PeImports(b);
    Check(result.parsed ? result.error.empty() : (!result.error.empty() && result.modules.empty() && result.forwarders.empty()), "Inconsistent mutation result");
  }
}
}
int main(int argc, char** argv) {
  try {
    Tests();
    MultiRootTests();
    std::cout << "PASS: " << checks << " deterministic Win32 import-reader checks\n";
    for (int i = 1; i < argc; ++i) {
      std::ifstream input(argv[i], std::ios::binary);
      if (!input) throw std::runtime_error("Cannot open optional PE control");
      Bytes bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
      const auto result = ReadWin32PeImports(bytes);
      if (!result.parsed) throw std::runtime_error(std::string(argv[i]) + ": " + result.error);
      std::cout << "PASS: read-only PE control " << argv[i] << " (" << result.modules.size() << " imports)\n";
      for (const auto& module : result.modules)
        std::cout << (module.delayed ? "delay " : "normal ") << module.name << '\n';
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}

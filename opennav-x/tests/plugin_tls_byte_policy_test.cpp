#include "integration/PluginTlsBytePolicy.h"
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
  for (unsigned i = 0; i < n; ++i)
    b.at(at + i) = static_cast<unsigned char>(value >> (8 * i));
}
void Name(Bytes& b, std::size_t at, const std::string& text) {
  for (const auto c : text) b.at(at++) = static_cast<unsigned char>(c);
  b.at(at) = 0;
}
Bytes Image(const std::string& module = {}, unsigned kind = 0) {
  Bytes b(0x2200);
  Put(b, 0, 0x5a4d, 2); Put(b, 60, 0x80); Put(b, 0x80, 0x4550);
  Put(b, 0x84, 0x14c, 2); Put(b, 0x86, 1, 2); Put(b, 0x94, 224, 2);
  Put(b, 0x98, 0x10b, 2); Put(b, 0xb4, 0x400000);
  Put(b, 0xd4, 0x200); Put(b, 0xf4, 16);
  Put(b, 0x180, 0x2000); Put(b, 0x184, 0x1000);
  Put(b, 0x188, 0x2000); Put(b, 0x18c, 0x200);
  if (module.empty()) return b;
  if (kind == 2) {
    Put(b, 0xf8, 0x1100); Put(b, 0xfc, 0x800);
    Put(b, 0x310, 1); Put(b, 0x314, 1); Put(b, 0x31c, 0x1140);
    Put(b, 0x340, 0x1180); Name(b, 0x380, module + ".fn");
  } else {
    const bool delayed = kind == 1;
    const auto directory = delayed ? 0x160 : 0x100;
    Put(b, directory, 0x1a00); Put(b, directory + 4, delayed ? 64 : 40);
    if (delayed) Put(b, 0xc00, 1);
    Put(b, delayed ? 0xc04 : 0xc0c, 0x1c00);
    Name(b, 0xe00, module);
  }
  return b;
}
void Tests() {
  // Known-answer tests exercise the exact picosha2 pipeline used in production.
  for (const auto& vector : std::vector<std::pair<Bytes, std::string>>{
      {{}, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"},
      {{'a','b','c'}, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"},
      {Bytes(1000000, 'a'), "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0"}}) {
    Check(PluginTlsBytesSha256(vector.first) == vector.second, "SHA256 known answer differs");
    Check(RefuseKnownObsoletePluginTlsBytes(vector.first).empty(), "Harmless bytes overmatched");
  }
  const std::vector<std::string> obsolete{
      "2bede6f2840cfe0a4e88d5c3fa648c170509c9ceed438c7e8440340541ca0446",
      "05752a0550349337af8b7e25362d527f6a30261dd3d1e4db61bf707c9fac589d"};
  for (const auto& digest : obsolete) {
    Check(RefuseKnownObsoletePluginTlsSha256(digest).find("Known obsolete TLS") == 0,
          "Exact known stock identity admitted");
    for (std::size_t i = 0; i < digest.size(); ++i) {
      auto other = digest; other[i] = other[i] == '0' ? '1' : '0';
      Check(RefuseKnownObsoletePluginTlsSha256(other).empty(), "Other TLS identity overmatched");
    }
  }
  const auto clean = Image();
  // No obsolete DLLs or claimed SHA256 preimages are embedded. The seam supplies
  // exact digest decisions for two inert PE fixtures; real hashing is tested above.
  for (std::size_t identity = 0; identity < obsolete.size(); ++identity) {
    auto marked = Image(); marked.back() = static_cast<unsigned char>(identity + 1);
    unsigned inspected = 0, reads = 0;
    auto policy = [&](const Bytes& bytes) {
      ++inspected;
      return bytes == marked ? RefuseKnownObsoletePluginTlsSha256(obsolete[identity])
                             : RefuseKnownObsoletePluginTlsBytes(bytes);
    };
    auto missing = [&](const std::string&) { ++reads; return PluginTlsLocalModule{}; };
    Check(!RefuseLegacyPluginLocalTlsClosure(marked, missing, policy).empty(),
          "Root known-byte policy skipped");
    Check(inspected == 1 && reads == 0, "Root refusal performed sibling reads");
    for (unsigned kind = 0; kind < 3; ++kind) {
      for (const std::string name : {"renamed", "libssl-3", "libcrypto-3", "MiXeD"}) {
        const auto root = Image(kind == 2 ? name : name + ".dll", kind);
        inspected = reads = 0;
        auto sibling = [&](const std::string& requested) {
          ++reads;
          Check(requested == name + ".dll", "Sibling spelling changed");
          return PluginTlsLocalModule{true, marked, {}};
        };
        Check(!RefuseLegacyPluginLocalTlsClosure(root, sibling, policy).empty(),
              "Normal/delay/forwarder known-byte sibling admitted");
        Check(inspected == 2 && reads == 1, "Known-byte closure policy skipped a node");
        auto harmless = [&](const std::string&) { return PluginTlsLocalModule{true, clean, {}}; };
        Check(RefuseLegacyPluginLocalTlsClosure(root, harmless,
              RefuseKnownObsoletePluginTlsBytes).empty(), "Ordinary 3.x filename refused");
      }
    }
    // A sibling's own forwarded dependency must receive the same byte policy.
    std::map<std::string, Bytes> chain{{"helper.dll", Image("renamed", 2)}, {"renamed.dll", marked}};
    inspected = reads = 0;
    auto lookup = [&](const std::string& name) {
      ++reads; return PluginTlsLocalModule{true, chain.at(name), {}};
    };
    Check(!RefuseLegacyPluginLocalTlsClosure(Image("helper.dll"), lookup, policy).empty(),
          "Transitive forwarder known bytes admitted");
    Check(inspected == 3 && reads == 2, "Transitive byte policy did not inspect every node");
    // Production's shared walk must apply this same hash decision to host
    // candidates, even when a clean plugin sibling has the identical basename.
    for (unsigned kind = 0; kind < 3; ++kind) {
      const auto root = Image(kind == 2 ? "renamed" : "renamed.dll", kind);
      inspected = reads = 0;
      auto candidates = [&](std::size_t index, const std::string& name) {
        ++reads;
        Check(name == "renamed.dll", "Multi-root normal/delay/forwarder spelling changed");
        return PluginTlsLocalModule{true, index == 0 ? clean : marked, {}};
      };
      Check(!RefuseLegacyPluginTlsClosure(root, {"plugin", "host"}, candidates, policy).empty(),
            "Clean sibling masked known obsolete host bytes");
      Check(inspected == 3 && reads == 2, "Known-byte policy skipped one root candidate");
      Check(RefuseLegacyPluginTlsClosure(root, {"plugin", "host"},
          [&](std::size_t, const std::string&) {
            return PluginTlsLocalModule{true, clean, {}};
          }, RefuseKnownObsoletePluginTlsBytes).empty(),
          "Benign multi-root normal/delay/forwarder candidates refused");
    }
    inspected = reads = 0;
    Check(!RefuseLegacyPluginTlsClosure(Image("helper.dll"),
        {"nested-plugin", "host", "upstream-plugin-base"},
        [&](std::size_t index, const std::string&) {
          ++reads;
          return PluginTlsLocalModule{true, index == 2 ? marked : clean, {}};
        }, policy).empty(), "Known obsolete upstream-root bytes escaped policy");
    Check(inspected == 4 && reads == 3, "Upstream-root hash inspection skipped a candidate");
    inspected = reads = 0;
    Check(!RefuseLegacyPluginTlsClosure(Image("helper.dll"), {"plugin", "host"},
        [&](std::size_t index, const std::string& name) {
          ++reads;
          if (index == 1 && name == "helper.dll")
            return PluginTlsLocalModule{true, Image("renamed", 2), {}};
          if (index == 0 && name == "renamed.dll")
            return PluginTlsLocalModule{true, marked, {}};
          return PluginTlsLocalModule{};
        }, policy).empty(), "Host forwarder to plugin-root known bytes escaped graph");
    Check(inspected == 3 && reads == 4, "Cross-root forwarder byte policy missed candidate");
  }
  for (unsigned kind = 0; kind < 3; ++kind) {
    const auto host_legacy = Image(kind == 2 ? "libeay32" : "libeay32.dll", kind);
    Check(!RefuseLegacyPluginTlsClosure(Image("helper.dll"), {"plugin", "host"},
        [&](std::size_t index, const std::string&) {
          return index == 0 ? PluginTlsLocalModule{} :
              PluginTlsLocalModule{true, host_legacy, {}};
        }, RefuseKnownObsoletePluginTlsBytes).empty(),
        "Host normal/delay/export-forwarder legacy declaration admitted");
  }
  unsigned reads = 0;
  const auto failed = RefuseLegacyPluginLocalTlsClosure(Image("helper.dll"),
      [&](const std::string&) { ++reads; return PluginTlsLocalModule{}; },
      [](const Bytes&) { return std::string("Injected byte-inspection failure"); });
  Check(failed == "Injected byte-inspection failure" && reads == 0,
        "Byte-inspection failure did not fail closed");
}
}
int main() {
  try { Tests(); std::cout << "PASS: " << checks << " plugin TLS byte-policy checks\n"; }
  catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}

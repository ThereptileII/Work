#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace opennav::integration {
struct Win32PeImport {
  std::string name;  // Original ASCII spelling; no filename or trust policy.
  bool delayed = false;
};
struct Win32PeImports {
  bool parsed = false;
  std::vector<Win32PeImport> modules;
  std::string error;
  // Raw ASCII module.symbol / module.#ordinal declarations, separate from
  // normal/delay imports. Spelling/grammar and trust policy are not inferred.
  std::vector<std::string> forwarders;
};

// Reads import names and declared export forwarders from supplied bytes.
// No file, loader, process,
// environment or registry access. Success is NOT image validity, dependency
// resolution, full export/thunk validation, ABI compatibility or executable trust.
// Unsupported/malformed metadata returns no partial declaration lists.
inline Win32PeImports ReadWin32PeImports(const std::vector<unsigned char>& bytes) {
  try {
    const auto require = [](bool condition, const char* message) {
      if (!condition) throw std::runtime_error(message);
    };
    require(bytes.size() >= 64 && bytes.size() <= 128u * 1024u * 1024u,
            "Unsupported PE byte size");
    const auto word = [&](std::uint64_t at, unsigned count) -> std::uint32_t {
      require(at <= bytes.size() && count <= bytes.size() - at,
              "Truncated PE field");
      std::uint32_t value = 0;
      for (unsigned i = 0; i < count; ++i)
        value |= std::uint32_t(bytes[static_cast<std::size_t>(at) + i]) << (8 * i);
      return value;
    };
    require(word(0, 2) == 0x5a4d, "Missing DOS signature");
    const std::uint64_t pe = word(60, 4), optional = pe + 24;
    require(pe >= 64 && word(pe, 4) == 0x4550, "Invalid PE signature offset");
    require(word(pe + 4, 2) == 0x14c, "Unsupported machine: Win32 x86 required");
    const auto count = word(pe + 6, 2), optional_size = word(pe + 20, 2);
    require(count > 0 && count <= 96, "Unsupported PE section count");
    require(optional_size >= 96 && optional <= bytes.size() &&
                optional_size <= bytes.size() - optional,
            "Truncated PE optional header");
    require(word(optional, 2) == 0x10b, "Unsupported optional header: PE32 required");
    const auto image_base = word(optional + 28, 4);
    const auto directory_count = word(optional + 92, 4);
    require(directory_count <= (optional_size - 96) / 8,
            "Data directories exceed optional header");
    const std::uint64_t section_table = optional + optional_size;
    const std::uint64_t section_end = section_table + 40u * count;
    const auto header_size = word(optional + 60, 4);
    require(section_end <= bytes.size() && header_size >= section_end &&
                header_size <= bytes.size(), "Invalid PE header extent");
    struct Section { std::uint64_t rva, span, raw, size; };
    std::vector<Section> sections;
    for (std::uint32_t i = 0; i < count; ++i) {
      const auto at = section_table + 40u * i;
      Section s{word(at + 12, 4),
                (std::max)(word(at + 8, 4), word(at + 16, 4)),
                word(at + 20, 4), word(at + 16, 4)};
      require(s.rva + s.span <= (std::uint64_t{1} << 32) &&
                  (!s.span || s.rva >= header_size), "Invalid section RVA range");
      require(!s.size || (s.raw >= header_size && s.raw <= bytes.size() &&
                  s.size <= bytes.size() - s.raw), "Invalid section raw range");
      for (const auto& prior : sections) {
        require(!s.span || !prior.span || s.rva >= prior.rva + prior.span ||
                    prior.rva >= s.rva + s.span, "Overlapping section RVAs");
        require(!s.size || !prior.size || s.raw >= prior.raw + prior.size ||
                    prior.raw >= s.raw + s.size, "Overlapping section raw ranges");
      }
      sections.push_back(s);
    }
    const auto offset = [&](std::uint64_t rva, std::uint64_t length) -> std::size_t {
      require(rva > 0 && length > 0 && rva < (std::uint64_t{1} << 32) &&
                  length <= (std::uint64_t{1} << 32) - rva, "Invalid RVA range");
      if (rva < header_size) {
        require(length <= header_size - rva, "RVA crosses header boundary");
        return static_cast<std::size_t>(rva);
      }
      for (const auto& s : sections) {
        if (rva >= s.rva && rva - s.rva < s.span) {
          const auto delta = rva - s.rva;
          require(delta <= s.size && length <= s.size - delta,
                  "RVA has no complete raw backing");
          return static_cast<std::size_t>(s.raw + delta);
        }
      }
      throw std::runtime_error("Unmapped RVA");
    };
    std::size_t name_bytes = 0;
    const auto module_name = [&](std::uint64_t rva) {
      std::string name;
      for (unsigned i = 0; i < 4096; ++i) {
        const auto c = bytes[offset(rva + i, 1)];
        if (!c) {
          require(!name.empty(), "Empty import module name");
          require(name_bytes <= 1024u * 1024u - name.size(),
                  "Import name budget exceeded");
          name_bytes += name.size();
          return name;
        }
        require(c >= 32 && c <= 126, "Non-ASCII import module name");
        name += static_cast<char>(c);
      }
      throw std::runtime_error("Unterminated or oversized import module name");
    };
    Win32PeImports result;
    for (const unsigned index : {1u, 13u}) {
      if (directory_count <= index) continue;
      const auto entry = optional + 96 + 8 * index;
      const std::uint64_t rva = word(entry, 4), size = word(entry + 4, 4);
      if (!rva && !size) continue;
      const unsigned descriptor_size = index == 1 ? 20 : 32;
      require(rva && size >= descriptor_size && size <= 1024u * 1024u,
              "Invalid import directory extent");
      const auto base = offset(rva, size);
      bool terminated = false;
      for (std::uint64_t used = 0; used + descriptor_size <= size;
           used += descriptor_size) {
        const auto at = base + used;
        bool zero = true;
        for (unsigned i = 0; i < descriptor_size; ++i)
          zero = zero && bytes[static_cast<std::size_t>(at) + i] == 0;
        if (zero) { terminated = true; break; }
        std::uint64_t name_rva = word(at + (index == 1 ? 12 : 4), 4);
        if (index == 13) {
          const auto attributes = word(at, 4);
          require(attributes <= 1, "Unsupported delay import attributes");
          if (!attributes) {
            require(name_rva >= image_base, "Delay import VA precedes image base");
            name_rva -= image_base;
          }
        }
        require(result.modules.size() < 4096, "Import descriptor budget exceeded");
        result.modules.push_back({module_name(name_rva), index == 13});
      }
      require(terminated, "Unterminated import directory");
    }
    if (directory_count) {
      const std::uint64_t rva = word(optional + 96, 4);
      const std::uint64_t size = word(optional + 100, 4);
      if (rva || size) {
        require(rva && size >= 40 && size <= 1024u * 1024u,
                "Invalid export directory extent");
        const auto base = offset(rva, size);
        const auto end = rva + size;  // offset already checked 32-bit RVA extent.
        const std::uint64_t entries = word(base + 20, 4);
        require(entries <= 65536, "Export address table budget exceeded");
        if (entries) {
          const auto eat = offset(word(base + 28, 4), entries * 4);
          for (std::uint64_t i = 0; i < entries; ++i) {
            const std::uint64_t target = word(eat + i * 4, 4);
            // Zero/hole and nonforwarded exports do not declare a dependency.
            // Their RVAs are deliberately not validated as executable code.
            if (target < rva || target >= end) continue;
            require(result.forwarders.size() < 4096,
                    "Export forwarder budget exceeded");
            std::string name;
            bool terminated = false;
            for (unsigned n = 0; n < 4096; ++n) {
              // Include NUL in the export-directory boundary check, even when
              // bytes beyond the directory still have valid raw backing.
              require(target + n < end, "Export forwarder crosses directory boundary");
              const auto c = bytes[offset(target + n, 1)];
              if (!c) { terminated = true; break; }
              require(c >= 32 && c <= 126, "Non-ASCII export forwarder");
              name += static_cast<char>(c);
            }
            require(terminated && !name.empty(), "Empty or oversized export forwarder");
            require(name_bytes <= 1024u * 1024u - name.size(),
                    "Declaration name budget exceeded");
            name_bytes += name.size();
            result.forwarders.push_back(std::move(name));
          }
        }
      }
    }
    result.parsed = true;
    return result;
  } catch (const std::exception& failure) {
    return {false, {}, failure.what(), {}};
  }
}
}  // namespace opennav::integration

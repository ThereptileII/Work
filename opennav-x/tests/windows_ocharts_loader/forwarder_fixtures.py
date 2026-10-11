"""Construct nine fixed inert PE forwarders; never edit production/vendor DLLs."""
import hashlib
import json
import struct

SLOT_PREFIX = b"__skager_forwarder_slot_"
RESERVATION = SLOT_PREFIX + b"X" * 96
FORWARDERS = {
    "fixture_forward_legacy_name.dll": "libeay32.legacy_tls_fixture",
    "fixture_forward_legacy_ordinal.dll": "libeay32.#209",
    "fixture_forward_ssleay_ordinal.dll": "ssleay32.#209",
    "fixture_forward_indirect.dll": "forward_helper.forwarded_probe",
    "fixture_forward_clean_name.dll": "forward_clean.fixture_value",
    "fixture_forward_clean_ordinal.dll": "forward_clean.#209",
    "fixture_forward_explicit_dll.dll": "forward_clean.dll.fixture_value",
    "fixture_forward_multi_dot.dll": "forward.clean.fixture_value",
    "forward_helper.dll": "libeay32.#209",
}


def require(value, message):
    if not value:
        raise ValueError(message)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


class Image:
    """Strict small-fixture PE32 layout, independent of the production reader."""
    def __init__(self, raw):
        self.raw = raw
        require(64 <= len(raw) <= 1024 * 1024 and raw[:2] == b"MZ", "fixture DOS bounds")
        pe = self.u32(0x3c)
        require(pe >= 64 and self.bytes(pe, 4) == b"PE\0\0", "fixture PE signature")
        require(self.u16(pe + 4) == 0x14c and self.u16(pe + 22) & 0x2000,
                "fixture must be I386 DLL")
        optional, size = pe + 24, self.u16(pe + 20)
        require(size >= 224 and self.u16(optional) == 0x10b, "fixture PE32 optional header")
        self.bytes(optional, size)
        require(self.u32(optional + 92) == 16, "fixture must have exactly sixteen data directories")
        require(self.u32(optional + 64) == 0, "fixture checksum must be unset")
        require(self.bytes(optional + 96 + 4 * 8, 8) == b"\0" * 8,
                "signed fixture must not be patched")
        self.export_rva, self.export_size = struct.unpack_from("<II", raw, optional + 96)
        require(self.export_size >= 40 and self.export_rva + self.export_size <= 0xffffffff,
                "fixture export interval")
        for index in range(1, 16):
            rva, length = struct.unpack_from("<II", raw, optional + 96 + index * 8)
            require(not length or rva + length <= self.export_rva or
                    self.export_rva + self.export_size <= rva,
                    "fixture other data directory overlaps exports")
        count = self.u16(pe + 6)
        require(1 <= count <= 16, "fixture section count")
        self.sections = []
        section_ranges = []
        headers_end = optional + size + count * 40
        header_size = self.u32(optional + 60)
        require(headers_end <= header_size <= len(raw), "fixture header extent")
        for i in range(count):
            at = optional + size + i * 40
            self.bytes(at, 40)
            virtual_size, rva, length, offset = struct.unpack_from("<IIII", raw, at + 8)
            self.bytes(offset, length)
            require(rva + max(virtual_size, length) <= 0xffffffff, "fixture section overflow")
            require(not length or offset >= header_size, "fixture section overlaps headers")
            require(not max(virtual_size, length) or rva >= header_size,
                    "fixture section RVA overlaps headers")
            for old_rva, old_size, old_offset, old_length in section_ranges:
                require(not length or not old_length or offset + length <= old_offset or
                        old_offset + old_length <= offset, "fixture raw sections overlap")
                require(rva + max(virtual_size, length) <= old_rva or
                        old_rva + old_size <= rva, "fixture virtual sections overlap")
            section_ranges.append((rva, max(virtual_size, length), offset, length))
            self.sections.append((rva, length, offset, self.u32(at + 36)))
        self.export_offset = self.map(self.export_rva, self.export_size)
        owners = [s for s in self.sections if s[0] <= self.export_rva and
                  self.export_rva + self.export_size <= s[0] + s[1]]
        require(len(owners) == 1 and owners[0][3] & 0x40000000 and
                not owners[0][3] & 0xa0000000, "fixture exports must be read-only non-code data")
        at = self.export_offset
        self.dll_name_rva = self.u32(at + 12)
        self.dll_name = self.string(self.dll_name_rva)
        self.function_count, self.name_count, self.functions_rva, names_rva, ordinals_rva = \
            struct.unpack_from("<IIIII", raw, at + 20)
        require(1 <= self.function_count <= 1024 and 2 <= self.name_count <= 32,
                "fixture export table counts")
        self.function_offset = self.export_map(self.functions_rva, self.function_count * 4)
        names = self.export_map(names_rva, self.name_count * 4)
        ordinals = self.export_map(ordinals_rva, self.name_count * 2)
        self.tables = ((at, at + 40),
                       (self.function_offset, self.function_offset + self.function_count * 4),
                       (names, names + self.name_count * 4),
                       (ordinals, ordinals + self.name_count * 2))
        for i, (start, end) in enumerate(self.tables):
            for other_start, other_end in self.tables[:i]:
                require(end <= other_start or other_end <= start, "fixture export tables overlap")
        self.entries = []
        for i in range(self.name_count):
            name_rva, index = self.u32(names + i * 4), self.u16(ordinals + i * 2)
            require(index < self.function_count, "fixture export ordinal outside EAT")
            name = self.string(name_rva)
            start = self.export_map(name_rva, len(name) + 1)
            require(all(start + len(name) + 1 <= left or right <= start
                        for left, right in self.tables), "fixture name overlaps export tables")
            self.entries.append((name, name_rva, index, self.u32(self.function_offset + index * 4)))
        ordered = [e[0] for e in self.entries]
        require(ordered == sorted(set(ordered)), "fixture export names not unique/sorted")

    def bytes(self, offset, length):
        require(0 <= offset <= len(self.raw) and 0 <= length <= len(self.raw) - offset,
                "fixture file range outside bytes")
        return self.raw[offset:offset + length]

    def u16(self, offset):
        return struct.unpack("<H", self.bytes(offset, 2))[0]

    def u32(self, offset):
        return struct.unpack("<I", self.bytes(offset, 4))[0]

    def map(self, rva, length):
        matches = [offset + rva - start for start, size, offset, _ in self.sections
                   if start <= rva and 0 <= length <= size - (rva - start)]
        require(len(matches) == 1, "fixture RVA missing/ambiguous raw section")
        self.bytes(matches[0], length)
        return matches[0]

    def export_map(self, rva, length):
        require(self.export_rva <= rva and 0 <= length <=
                self.export_size - (rva - self.export_rva), "fixture metadata outside export interval")
        return self.map(rva, length)

    def string(self, rva):
        at = self.export_map(rva, 1)
        end = self.raw.find(b"\0", at, self.export_offset + self.export_size)
        require(end >= at and end - at <= 160, "fixture export string bounds")
        value = self.raw[at:end]
        require(value and all(32 <= c <= 126 for c in value), "fixture export ASCII")
        return value


def construct(raw, filename):
    require(filename in FORWARDERS, "fixture filename is not allowlisted")
    image = Image(raw)
    probe = [e for e in image.entries if e[0] == b"forwarded_probe"]
    slot = [e for e in image.entries if e[0] == RESERVATION]
    require(len(probe) == len(slot) == 1, "exact unmodified fixture placeholder names required")
    probe, slot = probe[0], slot[0]
    require(probe[2] != slot[2] and probe[3] == slot[3] and probe[3] != 0,
            "fixture placeholder aliases differ")
    require(sum(e[2] == probe[2] for e in image.entries) == 1,
            "forwarded_probe EAT index is shared")
    for i in range(image.function_count):
        value = image.u32(image.function_offset + i * 4)
        require(not image.export_rva <= value < image.export_rva + image.export_size,
                "fixture already has forwarded exports")
    code_owners = [s for s in image.sections if s[0] <= probe[3] < s[0] + s[1]]
    require(len(code_owners) == 1 and code_owners[0][3] & 0x20000000,
            "placeholder does not reference fixture code")
    slot_offset = image.export_map(slot[1], len(RESERVATION) + 1)
    require(image.bytes(slot_offset, len(RESERVATION) + 1) == RESERVATION + b"\0",
            "fixture reservation bytes changed")
    dll_name_offset = image.export_map(image.dll_name_rva, len(image.dll_name) + 1)
    require(dll_name_offset + len(image.dll_name) + 1 <= slot_offset or
            slot_offset + len(RESERVATION) + 1 <= dll_name_offset,
            "fixture reservation overlaps DLL name")
    for entry in image.entries:
        if entry != slot:
            start = image.export_map(entry[1], len(entry[0]) + 1)
            require(start + len(entry[0]) + 1 <= slot_offset or
                    slot_offset + len(RESERVATION) + 1 <= start,
                    "fixture reservation overlaps another export name")
    target = FORWARDERS[filename].encode("ascii")
    payload = SLOT_PREFIX + b"\0" + target + b"\0"
    require(len(payload) <= len(RESERVATION) + 1, "forwarder exceeds reserved slot")
    forward_rva = slot[1] + len(SLOT_PREFIX) + 1
    image.export_map(forward_rva, len(target) + 1)
    eat_offset = image.function_offset + probe[2] * 4
    require(eat_offset + 4 <= slot_offset or slot_offset + len(RESERVATION) + 1 <= eat_offset,
            "fixture EAT and reservation overlap")
    result = bytearray(raw)
    result[slot_offset:slot_offset + len(RESERVATION) + 1] = payload.ljust(len(RESERVATION) + 1, b"\0")
    struct.pack_into("<I", result, eat_offset, forward_rva)
    result = bytes(result)
    require(len(result) == len(raw), "fixture file size changed")
    ranges = ((eat_offset, eat_offset + 4), (slot_offset, slot_offset + len(RESERVATION) + 1))
    require(all(a == b or any(start <= i < end for start, end in ranges)
                for i, (a, b) in enumerate(zip(raw, result))), "fixture changed outside reserved metadata")
    checked = Image(result)
    forwarded = [(e[0], checked.string(e[3])) for e in checked.entries
                 if checked.export_rva <= e[3] < checked.export_rva + checked.export_size]
    require(forwarded == [(b"forwarded_probe", target)], "constructed forwarder differs")
    require(checked.entries == [(SLOT_PREFIX if e[0] == RESERVATION else e[0], e[1], e[2],
                                forward_rva if e[0] == b"forwarded_probe" else e[3])
                               for e in image.entries], "other export identities changed")
    return result, {"fixture": filename, "target": target.decode(), "beforeSha256": sha(raw),
                    "afterSha256": sha(result), "bytes": len(result),
                    "eatFileOffset": eat_offset, "reservationFileOffset": slot_offset,
                    "reservationBytes": len(RESERVATION) + 1, "forwarderRva": forward_rva,
                    "forwarderNulInsideExportDirectory": True,
                    "onlyReservedMetadataChanged": True, "allOtherBytesAndFileSizeUnchanged": True}


def patch_fixtures(directory, receipt_path):
    """Only the runner's fresh, private fixture directory is a supported input.

    Validate all nine files before the first write. Persist failed/partial receipts
    if validation, a write, or its byte-exact readback fails. No native load occurs.
    """
    report = {"status": "failed", "construction": "bounded reserved export metadata",
              "nativeLoadability": "not established by construction", "fixtures": []}
    try:
        planned = []
        for name in FORWARDERS:
            path = directory / name
            require(path.is_file() and not path.is_symlink(), "missing/nonregular inert fixture: " + name)
            original = path.read_bytes()
            result, receipt = construct(original, name)
            report["fixtures"].append(dict(receipt, written=False))
            planned.append((path, original, result, report["fixtures"][-1]))
        receipt_path.write_text(json.dumps(report, indent=2) + "\n")
        for path, original, result, receipt in planned:
            require(path.read_bytes() == original, "fixture changed before construction: " + path.name)
            path.write_bytes(result)
            require(path.read_bytes() == result, "fixture construction readback differs: " + path.name)
            receipt["written"] = True
            receipt_path.write_text(json.dumps(report, indent=2) + "\n")
        report["status"] = "passed"
        return report
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        receipt_path.write_text(json.dumps(report, indent=2) + "\n")

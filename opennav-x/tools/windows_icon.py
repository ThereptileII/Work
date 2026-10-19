"""Embed the approved icon in a fresh Windows package intermediate (SCRUM-236).

The caller authenticates the ICO against branding/provenance.json and owns the
intermediate exclusively. Call before signing, hashing or sealing the package;
any exception makes the intermediate ineligible for packaging. No target code
is executed. Existing manifests and other resource types are retained.
"""
import ctypes
import importlib.util
import os
from pathlib import Path
import stat
import struct

SIZES = (16, 20, 24, 32, 40, 48, 64, 128, 256)
MAX_ICON_BYTES = 1 << 20
MAX_EXE_BYTES = 128 << 20  # Same binary bound as updater_package.MAX_BINARY_BYTES.
MAX_VERIFIER_BYTES = 64 << 10
RT_ICON, RT_GROUP_ICON = 3, 14
GROUP_ID, LANGUAGE = 0, 0


def _plain_file(path: Path, maximum: int) -> Path:
    path = Path(os.path.abspath(path))
    for component in (path, *path.parents):
        entry = component.lstat()
        if stat.S_ISLNK(entry.st_mode) or getattr(entry, 'st_file_attributes', 0) & 0x400:
            raise ValueError('Icon input/output path contains a link or reparse point')
    entry = path.stat()
    if not stat.S_ISREG(entry.st_mode) or not 0 < entry.st_size <= maximum:
        raise ValueError('Icon input/output must be a bounded regular file')
    return path


def _verifier():
    # Fixed adjacent, source-inventoried module; no caller-selected parser or
    # executable. Its adjacent product_version.py is also a packaging input.
    source = _plain_file(Path(__file__).with_name('verify-skager-pe-brand.py'),
                         MAX_VERIFIER_BYTES)
    spec = importlib.util.spec_from_file_location('_skager_pe_icon_verifier', source)
    if spec is None or spec.loader is None:
        raise RuntimeError('Cannot import the adjacent PE branding verifier')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _icon_data(icon: bytes, verifier):
    if not isinstance(icon, bytes) or not 6 <= len(icon) <= MAX_ICON_BYTES:
        raise ValueError('Approved ICO must be bounded bytes')
    expected = verifier.ico_frames(icon)
    if tuple(sorted(expected)) != SIZES:
        raise ValueError('Approved ICO requires exactly the nine branding sizes')
    # Translate the existing ICO directory to GRPICONDIR; frame IDs are fixed
    # and finite. Reject aliases, gaps and trailing bytes before opening an
    # update transaction. Payload validation/provenance remains the caller's.
    count = len(SIZES)
    end = 6 + count * 16
    group = bytearray(struct.pack('<HHH', 0, 1, count))
    payloads = {}
    for index in range(count):
        w, h, colors, reserved, planes, depth, size, offset = struct.unpack_from(
            '<BBBBHHII', icon, 6 + index * 16)
        if offset != end or colors or planes not in (0, 1):
            raise ValueError('Invalid approved ICO directory layout')
        resource_id = index + 1
        payloads[resource_id] = icon[offset:offset + size]
        group.extend(struct.pack('<BBBBHHIH', w, h, colors, reserved, planes,
                                 depth, size, resource_id))
        end += size
    if end != len(icon):
        raise ValueError('Approved ICO contains trailing bytes')
    return expected, bytes(group), payloads


def _native_file(path: Path) -> Path:
    if os.name != 'nt':
        raise RuntimeError('PE icon embedding and readback require native Windows')
    return _plain_file(path, MAX_EXE_BYTES)


def _coff_layout(path: Path):
    """Read bounded PE headers without following the possibly stale COFF pointer."""
    with _plain_file(path, MAX_EXE_BYTES).open('rb') as source:
        data = source.read(MAX_EXE_BYTES + 1)
    if not 64 <= len(data) <= MAX_EXE_BYTES or data[:2] != b'MZ':
        raise ValueError('Invalid PE DOS header during icon update')
    pe = struct.unpack_from('<I', data, 60)[0]
    if pe < 64 or pe + 26 > len(data) or data[pe:pe + 4] != b'PE\0\0':
        raise ValueError('Invalid PE header during icon update')
    count = struct.unpack_from('<H', data, pe + 6)[0]
    optional = struct.unpack_from('<H', data, pe + 20)[0]
    table = pe + 24 + optional
    if (not 1 <= count <= 96 or optional < 96 or table + 40 * count > len(data) or
            struct.unpack_from('<H', data, pe + 24)[0] not in (0x10b, 0x20b)):
        raise ValueError('Invalid PE section table during icon update')
    sections = [data[table + 40 * i:table + 40 * (i + 1)] for i in range(count)]
    spans = []
    for section in sections:
        size, offset = struct.unpack_from('<II', section, 16)
        if size:
            if offset < table + 40 * count or offset + size > len(data):
                raise ValueError('PE section exceeds file during icon update')
            spans.append((offset, offset + size))
    spans.sort()
    if any(right[0] < left[1] for left, right in zip(spans, spans[1:])):
        raise ValueError('Overlapping PE sections during icon update')
    pointer, symbols = struct.unpack_from('<II', data, pe + 12)
    return data, pe + 12, pointer, symbols, sections


def _snapshot_coff(path: Path):
    data, header, pointer, symbols, sections = _coff_layout(path)
    if pointer == 0:
        if symbols:
            raise ValueError('Missing COFF symbol table pointer')
        return None
    matches = [s for s in sections if s[:8] == b'.symtab\0']
    if len(matches) != 1:
        raise ValueError('COFF symbols require one unique .symtab section')
    section = matches[0]
    size, offset = struct.unpack_from('<II', section, 16)
    strings = pointer + 18 * symbols
    if not offset <= pointer <= strings <= offset + size - 4:
        raise ValueError('COFF symbol table exceeds .symtab section')
    string_size = struct.unpack_from('<I', data, strings)[0]
    if string_size < 4 or strings + string_size > offset + size:
        raise ValueError('COFF string table exceeds .symtab section')
    # COFF symbol section numbers are indices, not RVAs. Refuse an update that
    # would change their meaning instead of rewriting or stripping symbols.
    references = {}
    for index in range(symbols):
        record = pointer + 18 * index
        # The pinned internal Go linker emits no auxiliary records. Some other
        # linkers put additional section indices in them; do not guess those.
        if data[record + 17]:
            raise ValueError('Unsupported COFF auxiliary symbols')
        number = struct.unpack_from('<h', data, record + 12)[0]
        if number < -2 or number > len(sections):
            raise ValueError('Invalid COFF symbol section number')
        if number > 0:
            referenced = sections[number - 1]
            references[number] = referenced[:20] + referenced[24:]
    return (header, pointer, symbols, section, data[offset:offset + size], references)


def _restore_coff_pointer(path: Path, original) -> None:
    data, header, pointer, symbols, sections = _coff_layout(path)
    if original is None:
        if pointer or symbols:
            raise ValueError('Icon update unexpectedly introduced COFF symbols')
        return
    old_header, old_pointer, old_symbols, old_section, payload, references = original
    matches = [s for s in sections if s[:8] == b'.symtab\0']
    if header != old_header or symbols != old_symbols or len(matches) != 1:
        raise ValueError('Icon update changed COFF symbol identity')
    section = matches[0]
    # Windows may relocate both the section RVA and raw offset, but its other
    # metadata and all symbol/string bytes must remain exactly as compiled.
    if section[:12] + section[16:20] + section[24:] != old_section[:12] + old_section[16:20] + old_section[24:]:
        raise ValueError('Icon update changed .symtab section metadata')
    size, offset = struct.unpack_from('<II', section, 16)
    old_offset = struct.unpack_from('<I', old_section, 20)[0]
    relocated = offset + old_pointer - old_offset
    if data[offset:offset + size] != payload:
        raise ValueError('Icon update changed COFF symbol or string bytes')
    if any(number > len(sections) or sections[number - 1][:20] + sections[number - 1][24:] != identity
           for number, identity in references.items()):
        raise ValueError('Icon update changed a referenced COFF section index')
    if pointer not in (old_pointer, relocated):
        raise ValueError('Icon update produced an unexpected COFF pointer')
    if pointer != relocated:
        # EndUpdateResourceW inserts .rsrc before .reloc and moves .symtab,
        # but leaves this file offset stale (native diagnostic 39248589437).
        # Repair only that offset after proving the original tables survived.
        with path.open('r+b') as output:
            output.seek(header)
            output.write(struct.pack('<I', relocated))
            output.flush()
            os.fsync(output.fileno())
        repaired = _snapshot_coff(path)
        if repaired != (header, relocated, symbols, section, payload, references):
            raise ValueError('COFF pointer repair readback failed')


def verify_icon(path: Path, icon: bytes) -> None:
    """Read exact group and frame bytes without executing the target module."""
    path = _native_file(path)
    verifier = _verifier()
    expected, group, payloads = _icon_data(icon, verifier)
    resources = verifier.Resources(path)
    try:
        def read(kind, identifier, size):
            # Bound each allocation before the shared Resources.read copies it.
            entry = resources.k.FindResourceExW(resources.handle, kind, identifier, LANGUAGE)
            if not entry:
                raise ctypes.WinError(ctypes.get_last_error())
            if resources.k.SizeofResource(resources.handle, entry) != size:
                raise ValueError('PE icon resource size differs from the approved bytes')
            return resources.read(kind, identifier, LANGUAGE)

        observed_group = read(RT_GROUP_ICON, GROUP_ID, len(group))
        if observed_group != group:
            raise ValueError('PE icon group differs from the approved directory')
        verifier.check_group(observed_group,
                             lambda identifier: read(RT_ICON, identifier, len(payloads[identifier])),
                             expected)
    finally:
        resources.close()


def embed_icon(path: Path, icon: bytes) -> None:
    """Update only the approved icon resources, then require exact readback."""
    path = _native_file(path)
    verifier = _verifier()
    _, group, payloads = _icon_data(icon, verifier)
    original_coff = _snapshot_coff(path)
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    pointer = ctypes.c_void_p
    kernel.BeginUpdateResourceW.argtypes = [ctypes.c_wchar_p, ctypes.c_int]
    kernel.BeginUpdateResourceW.restype = pointer
    kernel.UpdateResourceW.argtypes = [pointer, pointer, pointer, ctypes.c_uint16,
                                      pointer, ctypes.c_uint32]
    kernel.UpdateResourceW.restype = ctypes.c_int
    kernel.EndUpdateResourceW.argtypes = [pointer, ctypes.c_int]
    kernel.EndUpdateResourceW.restype = ctypes.c_int
    # FALSE preserves Go manifests, version data and all unrelated resources.
    handle = kernel.BeginUpdateResourceW(str(path), False)
    if not handle:
        raise ctypes.WinError(ctypes.get_last_error())
    buffers = []
    try:
        updates = [(RT_ICON, identifier, data) for identifier, data in payloads.items()]
        updates.append((RT_GROUP_ICON, GROUP_ID, group))
        for kind, identifier, data in updates:
            buffer = ctypes.create_string_buffer(data)
            buffers.append(buffer)  # Keep all native buffers alive until End.
            if not kernel.UpdateResourceW(handle, kind, identifier, LANGUAGE, buffer, len(data)):
                raise ctypes.WinError(ctypes.get_last_error())
    except BaseException as error:
        # Updates are queued until End; a partial set must never be committed.
        if not kernel.EndUpdateResourceW(handle, True):
            discard_error = ctypes.get_last_error()
            raise RuntimeError(f'Icon update failed and discard failed (Win32 {discard_error}); '
                               'reject this package intermediate') from error
        raise
    # End consumes the update handle. A failed commit/readback is fatal to this
    # intermediate; do not retry a possibly consumed handle or claim rollback.
    if not kernel.EndUpdateResourceW(handle, False):
        raise ctypes.WinError(ctypes.get_last_error())
    _restore_coff_pointer(path, original_coff)
    verify_icon(path, icon)

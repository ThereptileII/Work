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
    verify_icon(path, icon)

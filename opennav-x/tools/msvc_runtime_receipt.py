"""Record app-local CRT provenance; this is not a redistribution license."""
import base64
import binascii
import ctypes
from ctypes import wintypes
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct


# Selected vswhere catalog facts only; never retain arbitrary instance properties
# such as user/profile data. These facts do not establish edition entitlement.
VISUAL_STUDIO_CATALOG_FIELDS = frozenset({
    'id', 'buildVersion', 'productDisplayVersion', 'productLine',
    'productLineVersion', 'productName', 'productSemanticVersion',
    'productMilestone', 'productMilestoneIsPreRelease', 'productPatchVersion',
    'productPreReleaseMilestoneSuffix', 'requiredEngineVersion',
})


def _visual_studio_catalog(catalog):
    if (type(catalog) is not dict or
            not set(catalog) <= VISUAL_STUDIO_CATALOG_FIELDS):
        raise ValueError('Visual Studio catalog identity fields invalid')
    for value in catalog.values():
        # Empty suffixes are valid catalog values; controls and nested data are not.
        if (type(value) is not str or (value != '' and not value.isprintable()) or
                len(value.encode('utf-8')) > 256):
            raise ValueError('Visual Studio catalog identity value invalid')
    return dict(catalog)


def parse_visual_studio_catalog_base64(raw):
    """Bound the non-secret JSON transport, including PS 5.1 native quoting."""
    if type(raw) is not str or len(raw) > 5464:
        raise ValueError('Visual Studio catalog transport invalid')
    def unique_object(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('Visual Studio catalog duplicate field')
            result[key] = value
        return result
    try:
        data = base64.b64decode(raw, validate=True)
        if len(data) > 4096:
            raise ValueError('Visual Studio catalog transport too large')
        catalog = json.loads(data.decode('utf-8'), object_pairs_hook=unique_object)
    except (binascii.Error, UnicodeError, json.JSONDecodeError, RecursionError) as error:
        raise ValueError('Visual Studio catalog transport invalid') from error
    return _visual_studio_catalog(catalog)


def _visual_studio_identity(visual_studio, version, product_id, catalog):
    if (type(product_id) is not str or not re.fullmatch(
            r'Microsoft\.VisualStudio\.Product\.[A-Za-z0-9][A-Za-z0-9.-]{0,63}', product_id)):
        raise ValueError('Visual Studio product identity invalid')
    return {'installationDirectory': str(visual_studio),
            'installationVersion': version, 'productId': product_id,
            'catalog': _visual_studio_catalog(catalog)}


def windows_file_versions(path):
    """Read numeric file/product versions without loading or executing the DLL."""
    if os.name != 'nt':
        raise ValueError('MSVC version inspection requires native Windows')
    api = ctypes.WinDLL('version', use_last_error=True)
    api.GetFileVersionInfoSizeW.argtypes = [wintypes.LPCWSTR, ctypes.POINTER(wintypes.DWORD)]
    api.GetFileVersionInfoSizeW.restype = wintypes.DWORD
    api.GetFileVersionInfoW.argtypes = [wintypes.LPCWSTR, wintypes.DWORD,
                                      wintypes.DWORD, ctypes.c_void_p]
    api.GetFileVersionInfoW.restype = wintypes.BOOL
    api.VerQueryValueW.argtypes = [ctypes.c_void_p, wintypes.LPCWSTR,
                                 ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(wintypes.UINT)]
    api.VerQueryValueW.restype = wintypes.BOOL
    unused = wintypes.DWORD()
    size = api.GetFileVersionInfoSizeW(str(path), ctypes.byref(unused))
    if not size:
        raise ValueError('MSVC file version missing: ' + path.name)
    data = ctypes.create_string_buffer(size)
    value, length = ctypes.c_void_p(), wintypes.UINT()
    if (not api.GetFileVersionInfoW(str(path), 0, size, data) or
            not api.VerQueryValueW(data, '\\', ctypes.byref(value), ctypes.byref(length)) or
            length.value < 52 or not value.value):
        raise ValueError('MSVC file version unreadable: ' + path.name)
    fields = struct.unpack('<13I', ctypes.string_at(value, 52))
    if fields[0] != 0xFEEF04BD:
        raise ValueError('MSVC file version signature invalid: ' + path.name)
    def version(high, low):
        return '.'.join(str(part) for part in (high >> 16, high & 0xffff, low >> 16, low & 0xffff))
    return {'fileVersion': version(fields[2], fields[3]),
            'productVersion': version(fields[4], fields[5])}


def _dll_record(path):
    if path.is_symlink() or not path.is_file():
        raise ValueError('MSVC DLL missing or redirected: ' + path.name)
    data = path.read_bytes()
    if len(data) < 64 or data[:2] != b'MZ':
        raise ValueError('MSVC DLL is not PE: ' + path.name)
    offset = struct.unpack_from('<I', data, 0x3c)[0]
    if (offset > len(data) - 6 or data[offset:offset + 4] != b'PE\0\0' or
            struct.unpack_from('<H', data, offset + 4)[0] != 0x14c):
        raise ValueError('MSVC DLL is not Win32/x86 PE: ' + path.name)
    return {'name': path.name, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
            **windows_file_versions(path)}


def copy_msvc_runtime(runtime, app, visual_studio, visual_studio_version,
                      visual_studio_product_id, visual_studio_catalog):
    """Copy the complete selected CRT set, verifying source and target equality."""
    runtime, app, visual_studio = Path(runtime).resolve(), Path(app), Path(visual_studio).resolve()
    redist_version = runtime.parent.parent.name
    if (not re.fullmatch(r'\d+(?:\.\d+){2,3}', visual_studio_version) or
            not re.fullmatch(r'\d+(?:\.\d+){2,3}', redist_version) or
            runtime != visual_studio / 'VC/Redist/MSVC' / redist_version / 'x86/Microsoft.VC143.CRT'):
        raise ValueError('MSVC runtime does not match selected Visual Studio x86 VC143 redist')
    identity = _visual_studio_identity(visual_studio, visual_studio_version,
                                       visual_studio_product_id, visual_studio_catalog)
    files = sorted((p for p in runtime.iterdir() if p.suffix.casefold() == '.dll'),
                   key=lambda p: p.name.casefold())
    names = [p.name.casefold() for p in files]
    if len(set(names)) != len(names):
        raise ValueError('MSVC runtime DLL case aliases are ambiguous')
    if not {'msvcp140.dll', 'vcruntime140.dll'} <= set(names):
        raise ValueError('MSVC runtime required DLL missing')
    records = [_dll_record(path) for path in files]
    for path, record in zip(files, records):
        target = app / path.name
        if target.is_symlink():
            raise ValueError('MSVC target DLL is redirected: ' + path.name)
        shutil.copy2(path, target)
        if _dll_record(path) != record or _dll_record(target) != record:
            raise ValueError('MSVC source/copied DLL equality failed: ' + path.name)
    return {'schemaVersion': 2, 'component': 'Microsoft Visual C++ runtime',
            'architecture': 'x86', 'family': 'Microsoft.VC143.CRT',
            'visualStudio': identity,
            'redistributable': {'directory': str(runtime), 'version': redist_version},
            'files': records, 'sourceCopyEquality': 'verified',
            'legalStatus': 'redistribution-terms-unresolved-review-required'}


def write_runtime_receipt(receipt, path):
    """Create an informational, read-only receipt covered by the package manifest."""
    with Path(path).open('x', encoding='utf-8', newline='\n') as output:
        output.write(json.dumps(receipt, indent=2, sort_keys=True) + '\n')
    Path(path).chmod(0o444)

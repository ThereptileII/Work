#!/usr/bin/env python3
"""Native, data-only PE icon and product metadata qualification (SCRUM-236)."""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import re
import struct

ROOT = Path(__file__).resolve().parents[1]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def ico_frames(data):
    if len(data) < 6 or struct.unpack_from('<HH', data) != (0, 1):
        raise ValueError('Invalid ICO header')
    count = struct.unpack_from('<H', data, 4)[0]
    frames = {}
    end = 6 + count * 16
    if not count or end > len(data):
        raise ValueError('Invalid ICO directory')
    for n in range(count):
        w, h, colors, reserved, planes, depth, size, offset = struct.unpack_from('<BBBBHHII', data, 6 + n * 16)
        w, h = w or 256, h or 256
        if w != h or depth != 32 or reserved or not size or offset < end or offset + size > len(data) or w in frames:
            raise ValueError('Invalid or duplicate approved ICO frame')
        frames[w] = data[offset:offset + size]
    return frames


def check_group(data, read_icon, expected):
    if len(data) < 6 or struct.unpack_from('<HH', data) != (0, 1):
        raise ValueError('Invalid PE group icon')
    count = struct.unpack_from('<H', data, 4)[0]
    if len(data) != 6 + count * 14:
        raise ValueError('Invalid PE group icon length')
    observed = {}
    for n in range(count):
        w, h, colors, reserved, planes, depth, size, resource_id = struct.unpack_from('<BBBBHHIH', data, 6 + n * 14)
        w, h = w or 256, h or 256
        payload = read_icon(resource_id)
        if w != h or depth != 32 or reserved or w in observed or len(payload) != size:
            raise ValueError('Invalid PE icon frame metadata')
        if w not in expected or payload != expected[w]:
            raise ValueError(f'PE icon {w}px differs from approved frame bytes')
        observed[w] = sha(payload)
    if set(observed) != set(expected):
        raise ValueError(f'PE icon sizes {sorted(observed)} differ from required {sorted(expected)}')
    return observed


class Resources:
    """LoadLibraryEx DATAFILE_EXCLUSIVE | IMAGE_RESOURCE: no imports/DllMain."""
    def __init__(self, path):
        if os.name != 'nt':
            raise RuntimeError('Actual PE resource qualification requires native Windows')
        self.k = ctypes.WinDLL('kernel32', use_last_error=True)
        self.v = ctypes.WinDLL('version', use_last_error=True)
        ptr, uint = ctypes.c_void_p, ctypes.c_uint32
        for name, restype, args in (
            ('LoadLibraryExW', ptr, [ctypes.c_wchar_p, ptr, uint]),
            ('FreeLibrary', ctypes.c_int, [ptr]),
            ('FindResourceExW', ptr, [ptr, ptr, ptr, ctypes.c_uint16]),
            ('LoadResource', ptr, [ptr, ptr]), ('LockResource', ptr, [ptr]),
            ('SizeofResource', uint, [ptr, ptr]),
            ('EnumResourceNamesW', ctypes.c_int, [ptr, ptr, ptr, ctypes.c_ssize_t]),
            ('EnumResourceLanguagesW', ctypes.c_int, [ptr, ptr, ptr, ptr, ctypes.c_ssize_t]),
        ):
            fn = getattr(self.k, name); fn.restype = restype; fn.argtypes = args
        self.v.VerQueryValueW.argtypes = [ptr, ctypes.c_wchar_p, ctypes.POINTER(ptr), ctypes.POINTER(uint)]
        self.v.VerQueryValueW.restype = ctypes.c_int
        self.handle = self.k.LoadLibraryExW(str(path.resolve()), None, 0x40 | 0x20)
        if not self.handle:
            raise ctypes.WinError(ctypes.get_last_error())

    def close(self):
        if self.handle:
            self.k.FreeLibrary(self.handle); self.handle = None

    @staticmethod
    def pointer(name):
        return name if isinstance(name, int) else ctypes.cast(ctypes.c_wchar_p(name), ctypes.c_void_p)

    def entries(self, kind):
        entries = []
        names_callback = ctypes.WINFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ssize_t)
        language_callback = ctypes.WINFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint16, ctypes.c_ssize_t)
        errors = []
        def name_cb(module, resource_type, name, parameter):
            name = name or 0  # ID 0 is valid (the application RC uses it).
            saved_name = name if name <= 65535 else ctypes.wstring_at(name)
            def language_cb(module, resource_type, name, language, parameter):
                entries.append((saved_name, language)); return 1
            cb = language_callback(language_cb)
            if not self.k.EnumResourceLanguagesW(module, resource_type, name, cb, 0):
                errors.append(ctypes.get_last_error())
            return 1
        callback = names_callback(name_cb)
        if not self.k.EnumResourceNamesW(self.handle, kind, callback, 0) or errors:
            raise RuntimeError(f'Resource enumeration failed for type {kind}: {errors or ctypes.get_last_error()}')
        return entries

    def read(self, kind, name, language):
        resource = self.k.FindResourceExW(self.handle, kind, self.pointer(name), language)
        if not resource:
            raise ctypes.WinError(ctypes.get_last_error())
        size = self.k.SizeofResource(self.handle, resource)
        pointer = self.k.LockResource(self.k.LoadResource(self.handle, resource))
        if not pointer or not size:
            raise ValueError('Empty/unreadable PE resource')
        return ctypes.string_at(pointer, size)

    def version(self, data, field):
        # Query all StringFileInfo tables, including the RC's translation-less table.
        tables = sorted(set(re.findall(rb'(?:[0-9A-Fa-f]\x00){8}\x00\x00', data)))
        values = []
        buffer = ctypes.create_string_buffer(data)
        for table in tables:
            name = table.decode('utf-16le').rstrip('\0')
            value, size = ctypes.c_void_p(), ctypes.c_uint32()
            if self.v.VerQueryValueW(buffer, f'\\StringFileInfo\\{name}\\{field}', ctypes.byref(value), ctypes.byref(size)):
                values.append(ctypes.wstring_at(value, size.value).rstrip('\0'))
        if not values:
            raise ValueError(f'Missing PE version field {field}')
        return values


def expected_metadata(text, setup=False):
    pattern = r'VIAddVersionKey\s+"(ProductName|FileDescription)"\s+"([^"]+)"' if setup else r'VALUE\s+"(ProductName|FileDescription)",\s*"([^"\\]+)\\0"'
    values = dict(re.findall(pattern, text))
    if set(values) != {'ProductName', 'FileDescription'}:
        raise ValueError('Expected source metadata is missing or ambiguous')
    return values


def inspect(path, expected, metadata):
    before = path.read_bytes()
    resources = Resources(path)
    try:
        groups = []
        for name, language in resources.entries(14):
            frames = check_group(resources.read(14, name, language), lambda ident: resources.read(3, ident, language), expected)
            groups.append({'id': name, 'language': language, 'frames': frames})
        versions = []
        for name, language in resources.entries(16):
            data = resources.read(16, name, language)
            fields = {field: resources.version(data, field) for field in metadata}
            for field, values in fields.items():
                if any(value != metadata[field] for value in values):
                    raise ValueError(f'{path.name} {field} mismatch: {values}')
            versions.append({'id': name, 'language': language, 'fields': fields})
    finally:
        resources.close()
    if not groups or not versions:
        raise ValueError('Required icon/version resources are absent')
    if before != path.read_bytes():
        raise ValueError('Executable changed during resource inspection')
    return {'name': path.name, 'bytes': len(before), 'sha256': sha(before), 'groups': groups, 'versions': versions}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--application', type=Path, required=True)
    parser.add_argument('--setup', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    provenance = json.loads((ROOT / 'resources/branding/provenance.json').read_text())
    icon = (ROOT / 'resources/branding/skager.ico').read_bytes()
    if sha(icon) != provenance['outputs']['resources/branding/skager.ico']:
        raise ValueError('Approved ICO provenance hash mismatch')
    expected = ico_frames(icon)
    if sorted(expected) != provenance['iconSizes']:
        raise ValueError('Approved ICO size declaration mismatch')
    rc = ROOT / 'src/integration/Skager.rc.in'
    nsi = ROOT / 'installer/windows/AlphaSetup.nsi'
    report = {'schema': 1, 'scope': 'native data-only final PE branding; no application/setup execution',
              'approvedIconSha256': sha(icon), 'requiredSizes': sorted(expected),
              'inputs': {str(p.relative_to(ROOT)): sha(p.read_bytes()) for p in (rc, nsi)},
              'application': inspect(args.application, expected, expected_metadata(rc.read_text())),
              'setup': inspect(args.setup, expected, expected_metadata(nsi.read_text(), setup=True))}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print('Application and Setup: exact approved icon frames and SKAGER metadata verified')


if __name__ == '__main__':
    main()

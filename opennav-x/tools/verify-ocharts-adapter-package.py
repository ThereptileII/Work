#!/usr/bin/env python3
"""Verify exact adapter bytes/source/resources before emitting the compiled trust header.

This verifies package identity, not native rendering, licensing, TLS or boat acceptance.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import struct
import shutil
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
_spec = importlib.util.spec_from_file_location('prepare_ocharts', ROOT / 'tools/prepare-ocharts-adapter.py')
prep = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(prep)
DLL = 'skager-ocharts-adapter.dll'
EXPORTS = {'create_pi', 'destroy_pi', 'skager_bind_chart_presentation_v1',
           'skager_chart_presentation_status_v1'}
# Direct imports only. The qualified application's modern libcurl supplies TLS.
IMPORTS = {'opencpn.exe', 'libcurl.dll', 'zlib1.dll', 'glew32.dll',
           'kernel32.dll', 'user32.dll', 'gdi32.dll', 'gdiplus.dll', 'advapi32.dll', 'shell32.dll',
           'ole32.dll', 'oleaut32.dll', 'comdlg32.dll', 'comctl32.dll', 'winspool.drv',
           'ws2_32.dll', 'rpcrt4.dll', 'uuid.dll', 'version.dll', 'shlwapi.dll',
           'opengl32.dll', 'glu32.dll', 'msvcp140.dll', 'vcruntime140.dll', 'ucrtbase.dll'}
IMPORTS |= {'api-ms-win-crt-' + x + '-l1-1-0.dll' for x in (
    'runtime', 'stdio', 'heap', 'string', 'math', 'convert', 'time', 'locale',
    'environment', 'filesystem', 'utility', 'conio', 'multibyte', 'process')}
IMPORTS |= {'wxbase32u' + s + '_vc14x.dll' for s in ('', '_net', '_xml')}
IMPORTS |= {'wxmsw32u_' + s + '_vc14x.dll' for s in ('core', 'adv', 'aui', 'html', 'stc', 'gl')}


def pe_contract(data):
    def read(fmt, offset):
        size = struct.calcsize(fmt)
        if offset < 0 or offset + size > len(data):
            raise ValueError('Truncated PE')
        return struct.unpack_from(fmt, data, offset)
    if data[:2] != b'MZ':
        raise ValueError('Not a PE DLL')
    pe, = read('<I', 60)
    if data[pe:pe + 4] != b'PE\0\0':
        raise ValueError('Invalid PE signature')
    machine, count, _, _, _, optional, flags = read('<HHIIIHH', pe + 4)
    if machine != 0x14c or not flags & 0x2000 or not 1 <= count <= 96 or optional < 224:
        raise ValueError('Adapter must be an I386 DLL')
    opt = pe + 24
    if read('<H', opt)[0] != 0x10b or read('<I', opt + 92)[0] < 16:
        raise ValueError('Expected PE32 data directories')
    sections = [read('<IIII', opt + optional + n * 40 + 8) for n in range(count)]
    def offset(rva, size=1):
        for virtual_size, base, raw_size, raw in sections:
            if base <= rva and rva - base + size <= min(virtual_size, raw_size):
                result = raw + rva - base
                if result + size <= len(data):
                    return result
        raise ValueError('PE address outside raw section')
    def string(rva):
        start = offset(rva)
        end = data.find(b'\0', start, min(start + 512, len(data)))
        if end < 0:
            raise ValueError('Invalid PE name')
        return data[start:end].decode('ascii')
    # Delay imports could introduce an unreviewed side-by-side dependency.
    if any(read('<II', opt + 96 + 13 * 8)):
        raise ValueError('Delay imports are not supported')
    imp, size = read('<II', opt + 96 + 8)
    imports = []
    if not imp or size < 20 or size > 65536:
        raise ValueError('Missing/unbounded imports')
    for n in range(size // 20):
        desc = read('<IIIII', offset(imp + n * 20, 20))
        if not any(desc):
            break
        name = string(desc[3]).lower()
        if name not in IMPORTS or name in imports:
            raise ValueError('Unexpected/duplicate import: ' + name)
        imports.append(name)
    else:
        raise ValueError('Unterminated import table')
    if 'opencpn.exe' not in imports:
        raise ValueError('Missing API17 host import')
    exp, exp_size = read('<II', opt + 96)
    if not exp or exp_size < 40:
        raise ValueError('Missing exports')
    base = offset(exp, 40)
    functions, names, addresses, name_table, ordinals = read('<IIIII', base + 20)
    if names != 4 or functions != 4:
        raise ValueError('Unexpected export inventory')
    exports = []
    for n in range(names):
        exports.append(string(read('<I', offset(name_table + n * 4, 4))[0]))
        ordinal = read('<H', offset(ordinals + n * 2, 2))[0]
        if ordinal >= functions:
            raise ValueError('Invalid export ordinal')
        address = read('<I', offset(addresses + ordinal * 4, 4))[0]
        if not address or exp <= address < exp + exp_size:
            raise ValueError('Forwarded/missing adapter export')
        offset(address)
    if set(exports) != EXPORTS:
        raise ValueError('Adapter export contract differs')
    return sorted(imports), sorted(exports)


def verify(package, resources):
    if {p.name for p in package.iterdir()} != {DLL, 'manifest.json', 'corresponding-source.zip'}:
        raise ValueError('Unexpected adapter package payload')
    manifest = json.loads((package / 'manifest.json').read_text())
    lock = json.loads((ROOT / prep.LOCK).read_text())
    expected_source = {'repository': lock['source']['repository'], 'commit': lock['source']['commit'],
                       'gitlinkCommit': lock['gitlink']['commit'],
                       'manifestSha256': prep.record(ROOT / prep.LOCK, text=True)['sha256']}
    if (manifest.get('schema') != 1 or manifest.get('kind') != 'skager-ocharts-adapter' or
            manifest.get('bindingVersion') != 1 or manifest.get('source') != expected_source):
        raise ValueError('Unsupported adapter source/binding identity')
    if manifest.get('inputs') != {p: prep.record(ROOT / p, text=True) for p in prep.INPUTS}:
        raise ValueError('Adapter source/patch inputs differ')
    if manifest.get('chartResourceManifest') != prep.record(resources / 'manifest.json'):
        raise ValueError('Adapter chart resource manifest differs')
    if manifest.get('chartResourceHeader') != prep.record(resources / 'XNavChartResources.h'):
        raise ValueError('Adapter chart resource header differs')
    from curl_package import SOURCES, SOURCE_KEYS, CURL_OUTPUTS, ZLIB_OUTPUTS, require_record
    deps = manifest.get('dependencies', {})
    if set(deps) != {'curl', 'zlib'}:
        raise ValueError('Missing maintained dependency receipt')
    for lib, outputs in (('curl', CURL_OUTPUTS), ('zlib', ZLIB_OUTPUTS)):
        dep = deps[lib]
        if (set(dep) != {'version', 'source', 'outputs'} or dep['version'] != SOURCES[lib]['version'] or
                dep['source'] != {key: SOURCES[lib][key] for key in SOURCE_KEYS} or
                set(dep['outputs']) != outputs):
            raise ValueError('Legacy or unexpected dependency source')
        for name, value in dep['outputs'].items():
            require_record(value, name)
    resource_manifest = json.loads((resources / 'manifest.json').read_text())
    for name, expected in resource_manifest['files'].items():
        if len(prep.safe_path(name).parts) != 1 or prep.record(resources / name) != expected:
            raise ValueError('Current chart resource bytes differ')
    actual = prep.record(package / DLL)
    if manifest.get('dll') != dict(actual, name=DLL, machine='I386'):
        raise ValueError('Adapter DLL hash/size differs')
    imports, exports = pe_contract((package / DLL).read_bytes())
    if manifest.get('imports') != imports or manifest.get('exports') != exports:
        raise ValueError('Adapter PE receipt differs')
    if manifest.get('correspondingSource') != dict(prep.record(package / 'corresponding-source.zip'),
                                                 path='corresponding-source.zip'):
        raise ValueError('Corresponding source package differs')
    with zipfile.ZipFile(package / 'corresponding-source.zip') as archive:
        names = archive.namelist()
        expected = {'product/' + p for p in prep.INPUTS}
        expected |= {'source/' + p for p in lock['source']['files']}
        expected |= {'source/opencpn-libs/' + p for p in lock['gitlink']['files']}
        if len(names) != len(set(names)) or set(names) != expected:
            raise ValueError('Corresponding source inventory/CRC differs')
        if (sum(info.file_size for info in archive.infolist()) > 32 * 1024 * 1024 or
                any(info.file_size > 16 * 1024 * 1024 or info.is_dir() for info in archive.infolist())):
            raise ValueError('Unbounded source archive')
        if archive.testzip():
            raise ValueError('Corresponding source CRC differs')
        for p in prep.INPUTS:
            if archive.read('product/' + p) != (ROOT / p).read_bytes().replace(b'\r\n', b'\n'):
                raise ValueError('Corresponding product source differs')
        for key in ('source', 'gitlink'):
            for name, blob in lock[key]['files'].items():
                path = 'source/' + ('opencpn-libs/' if key == 'gitlink' else '') + name
                if not prep.blob_ok(archive.read(path), blob):
                    raise ValueError('Corresponding upstream blob differs')
    return actual


def trust_header(result):
    return ('#pragma once\n#include <cstdint>\nnamespace skager_ocharts {\n'
            'inline constexpr bool available = true;\n'
            f'inline constexpr char sha256[] = "{result["sha256"]}";\n'
            f'inline constexpr std::uint64_t bytes = {result["bytes"]};\n}}\n')


def installed_source_bundle(app, build):
    """Verify the installed DLL/source/runtime closure, and return its source ZIP.

    No native code is loaded. The header must be the exact configure output;
    adapter-free builds reject stray adapter files rather than packaging them.
    """
    app, build = Path(app), Path(build)
    header = (build / 'include/SkagerOChartsPackage.h').read_text()
    notices = app / 'opennav/third-party/ocharts'
    present = (app / DLL).exists() or notices.exists()
    unavailable = (ROOT / 'src/integration/SkagerOChartsPackage.h.in').read_text()
    for key, value in [('available', 'false'), ('sha256', ''), ('bytes', '0')]:
        unavailable = unavailable.replace('@skager_ocharts_' + key + '@', value)
    if header == unavailable:
        if present:
            raise ValueError('Adapter payload present in an adapter-free build')
        return []
    if not present or not notices.is_dir() or notices.is_symlink():
        raise ValueError('Installed private chart adapter/source is missing')
    if {p.name for p in notices.iterdir()} != {'manifest.json', 'corresponding-source.zip'}:
        raise ValueError('Unexpected installed adapter notices')
    resources = build / 'opennav-chart-style/v1'
    with tempfile.TemporaryDirectory(prefix='skager-installed-adapter-') as raw:
        package = Path(raw)
        for source in (app / DLL, notices / 'manifest.json', notices / 'corresponding-source.zip'):
            prep.record(source)  # Reject missing files and symlinks before copying.
            shutil.copyfile(source, package / source.name)
        result = verify(package, resources)
        if header != trust_header(result):
            raise ValueError('Installed adapter differs from compiled identity')
    manifest = json.loads((notices / 'manifest.json').read_text())
    from curl_package import verify_file
    for library, filename in [('curl', 'libcurl.dll'), ('zlib', 'zlib1.dll')]:
        verify_file(app / filename, manifest['dependencies'][library]['outputs']['bin/' + filename])
    # The renderer must see exactly the resources used to build its trust header.
    installed_resources = app / 'opennav/chart-style/v1'
    for name in ('manifest.json', 'chartsymbols.xml', 'S52RAZDS.RLE',
                 'rastersymbols-day.png', 'rastersymbols-dusk.png', 'rastersymbols-dark.png'):
        if prep.record(installed_resources / name) != prep.record(resources / name):
            raise ValueError('Installed chart resources differ: ' + name)
    source = notices / 'corresponding-source.zip'
    return [{'archive': source, 'path': 'third-party-sources/skager-ocharts-source.zip',
             'sha256': prep.record(source)['sha256'], 'reference': manifest['source']}]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--resources', type=Path, required=True)
    parser.add_argument('--header', type=Path,
                        help='Emit the compiled trust header after verification; omit for validation only')
    args = parser.parse_args()
    result = verify(args.package, args.resources)
    if args.header is None:
        return
    args.header.parent.mkdir(parents=True, exist_ok=True)
    text = trust_header(result)
    if not args.header.exists() or args.header.read_text() != text:
        args.header.write_text(text)


if __name__ == '__main__':
    main()

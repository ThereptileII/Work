#!/usr/bin/env python3
"""SCRUM-259: fetch/prepare a closed, pinned source-only adapter build workspace."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import tempfile
import subprocess
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
LOCK = 'tools/ocharts-adapter-source.lock.json'
PATCHES = ('patches/ocharts-skager-presentation.patch', 'patches/ocharts-wxcurl-trust.patch')
LOCAL = ('src/plugin-adapters/ChartPresentationBindingV1.h',
         'src/plugin-adapters/ocharts/ChartPresentationAdapter.h',
         'src/plugin-adapters/ocharts/ChartPresentationAdapter.cpp',
         'src/plugin-adapters/ocharts/BindingState.h',
         'src/plugin-adapters/ocharts/PointStyleObservation.h',
         'src/plugin-adapters/ocharts/OwnedPresentationValidation.h',
         'src/plugin-adapters/ocharts/ResourceVerification.h',
         'src/integration/ChartNameTypography.h', 'src/integration/ChartNameText.h',
         'src/integration/ChartTextFace.h',
         'src/integration/ChartNameSpacing.h',
         'src/integration/ChartNameAlphaWindows.cpp',
         'src/integration/ChartLightLabel.h', 'src/integration/ChartLightSymbol.h', 'src/integration/ChartSpecialBuoySymbol.h', 'src/integration/ChartYellowBuoySymbol.h', 'src/integration/ChartSoundingFont.h',
         'src/integration/ChartCanvasInk.h', 'src/ui/Theme.h')
RECIPE = 'cmake/ocharts-adapter/CMakeLists.txt'
INPUTS = (LOCK, RECIPE, 'cmake/ocharts-adapter/PreparedPath.cmake',
          'cmake/ocharts-adapter/Targets.cmake', 'tools/prepare-ocharts-adapter.py',
          'tools/verify-ocharts-adapter-package.py', 'tools/windows-wx.lock.json',
          'tools/windows-curl.lock.json', 'tools/windows-zlib.lock.json', 'tools/windows-openssl.lock.json',
          'tools/curl_package.py', 'tools/openssl_package.py',
          'tools/test-downloader-trust-windows.ps1', 'tests/downloader_trust/CMakeLists.txt',
          'tests/downloader_trust/InputPaths.cmake', 'tests/downloader_trust/Targets.cmake',
          'tools/wxcurl-trust-probe.cpp', 'tools/downloader-trust-probe.cpp',
          'tools/downloader-trust-server.py') + PATCHES + LOCAL


def record(path, text=False):
    if path.is_symlink() or not path.is_file():
        raise ValueError('Missing or linked input: ' + str(path))
    data = path.read_bytes()
    if text:
        data = data.replace(b'\r\n', b'\n')
    return {'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}


def safe_path(name):
    p = PurePosixPath(name)
    if (not name or p.is_absolute() or '..' in p.parts or '\\' in name or ':' in name
            or p.as_posix() != name):
        raise ValueError('Unsafe source path')
    return p


def blob_ok(data, expected):
    return (len(data) == expected['bytes'] and
            hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
            == expected['gitBlob'])


def fetch_sources(destination, cache):
    lock = json.loads((ROOT / LOCK).read_text())
    jobs = []
    for key in ('source', 'gitlink'):
        part = lock[key]
        for name, expected in part['files'].items():
            safe_path(name)
            # The one open API import library is the only binary source input.
            if Path(name).suffix.lower() in ('.exe', '.dll', '.lib') and not (
                    key == 'gitlink' and name == 'api-17/msvc-wx32/opencpn.lib'):
                raise ValueError('Forbidden binary payload')
            if any(s in name.lower() for s in ('oeserverd', 'oexserverd')):
                raise ValueError('Forbidden helper payload')
            jobs.append((part, name, expected, key))
    def fetch(job):
        part, name, expected, key = job
        cached = cache / expected['gitBlob']
        if cached.exists():
            data = cached.read_bytes()
        else:
            url = f"https://raw.githubusercontent.com/{part['repository']}/{part['commit']}/{name}"
            with urllib.request.urlopen(url, timeout=60) as response:
                data = response.read(expected['bytes'] + 1)
        if not blob_ok(data, expected):
            raise ValueError('Source blob differs: ' + name)
        if not cached.exists():
            with tempfile.NamedTemporaryFile(dir=cache, delete=False) as stream:
                stream.write(data)
                temporary = Path(stream.name)
            temporary.replace(cached)
        target = destination / ('opencpn-libs' if key == 'gitlink' else '') / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    cache.mkdir(parents=True, exist_ok=True)
    with ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(fetch, jobs))
    return lock


def apply_patches(source, root):
    tracked = {p.relative_to(source).as_posix() for p in source.rglob('*') if p.is_file()}
    # Pinned upstream mixes LF and CRLF. Patch only a derived text copy;
    # original blobs/import library remain exact and are distributed unchanged.
    for name in tracked:
        if not name.endswith('.lib'):
            path = source / name
            data = path.read_bytes()
            if b'\r\n' in data:
                path.write_bytes(data.replace(b'\r\n', b'\n'))
    for name in PATCHES:
        patch = (root / name).read_bytes().replace(b'\r\n', b'\n')
        paths = re.findall(rb'^\+\+\+ b/(.+)$', patch, re.M)
        if not paths or any(p.decode() not in tracked for p in paths):
            raise ValueError('Patch adds or changes an unpinned source path')
        # A source directory nested in the product checkout otherwise causes
        # git-apply to silently skip paths relative to the parent's worktree.
        with tempfile.TemporaryDirectory(prefix='skager-patch-index-') as raw:
            metadata = Path(raw) / 'git'
            subprocess.run(['git', 'init', '--bare', '--quiet', str(metadata)], check=True)
            # This derived source is deliberately LF. Do not let the runner's
            # global Windows checkout policy re-expand patched files to CRLF.
            command = ['git', '-c', 'core.bare=false', '-c', 'core.autocrlf=false',
                       '-c', 'core.eol=lf', '--git-dir=' + str(metadata),
                       '--work-tree=' + str(source.resolve()), 'apply']
            subprocess.run(command + ['--check', '-'], input=patch, cwd=source, check=True)
            subprocess.run(command + ['-'], input=patch, cwd=source, check=True)


def verify_prepared(directory):
    receipt = json.loads((directory / 'preparation.json').read_text())
    actual = {p.relative_to(directory).as_posix(): record(p)
              for p in directory.rglob('*') if p.is_file() and p.name != 'preparation.json'}
    if actual != receipt['preparedFiles']:
        raise ValueError('Prepared input inventory/hash differs')
    if receipt['inputs'] != {p: record(ROOT / p, text=True) for p in INPUTS}:
        raise ValueError('Product source inputs changed since preparation')
    lock = json.loads((ROOT / LOCK).read_text())
    expected_original = {}
    for key in ('source', 'gitlink'):
        for name, blob in lock[key]['files'].items():
            rel = ('opencpn-libs/' if key == 'gitlink' else '') + name
            expected_original[rel] = blob
    original = directory / 'original'
    original_names = {p.relative_to(original).as_posix() for p in original.rglob('*') if p.is_file()}
    if original_names != set(expected_original):
        raise ValueError('Original source inventory differs')
    for name, blob in expected_original.items():
        if not blob_ok((original / name).read_bytes(), blob):
            raise ValueError('Original source blob differs')
    import tempfile
    with tempfile.TemporaryDirectory(prefix='skager-adapter-verify-') as temp:
        derived = Path(temp) / 'source'
        shutil.copytree(original, derived)
        apply_patches(derived, ROOT)
        expected = {p.relative_to(derived).as_posix(): record(p) for p in derived.rglob('*') if p.is_file()}
        actual_source = {p.relative_to(directory / 'source').as_posix(): record(p)
                         for p in (directory / 'source').rglob('*') if p.is_file()}
        if actual_source != expected:
            raise ValueError('Patched source does not derive from pinned originals')
    for name in LOCAL:
        if record(directory / 'local' / name, text=True) != record(ROOT / name, text=True):
            raise ValueError('Prepared product overlay differs')
    return receipt


def verify_producer_dependencies(curl_prefix, openssl_prefix, zlib_prefix):
    """Verify explicit same-job producers, not the co-located installed package."""
    from curl_package import verify_manifest, verify_file
    manifests = {
        'curl': verify_manifest(curl_prefix, 'curl', dependency_prefixes={
            'openssl': openssl_prefix, 'zlib': zlib_prefix}),
        'zlib': verify_manifest(zlib_prefix, 'zlib'),
    }
    for library, prefix in (('curl', curl_prefix), ('zlib', zlib_prefix)):
        for name, expected in manifests[library]['outputs'].items():
            verify_file(prefix / name, expected)
    return manifests


def copy_local_inputs(out):
    """Copy the exact owned include closure consumed by the native recipe."""
    for name in LOCAL:
        target = out / 'local' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((ROOT / name).read_bytes().replace(b'\r\n', b'\n'))


def prepare(args):
    manifests = verify_producer_dependencies(args.curl_prefix, args.openssl_prefix, args.zlib_prefix)
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    lock = fetch_sources(out / 'original', args.cache.resolve())
    shutil.copytree(out / 'original', out / 'source')
    apply_patches(out / 'source', ROOT)
    copy_local_inputs(out)
    # Consume current, tested dependency outputs; never fallback to plugin vendor libs.
    from curl_package import verify_file
    for library in ('curl', 'zlib'):
        prefix = args.curl_prefix if library == 'curl' else args.zlib_prefix
        manifest = manifests[library]
        for name, expected in manifest['outputs'].items():
            verify_file(prefix / name, expected)
            target = out / 'sdk' / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(prefix / name, target)
        shutil.copyfile(prefix / (library + '-build.json'),
                        out / 'sdk' / (library + '-build.json'))
    wx = out / 'sdk/wx'
    for item in json.loads((ROOT / 'tools/windows-wx.lock.json').read_text())['archives']:
        archive = args.cache.resolve() / item['file']
        if not archive.exists():
            archive.write_bytes(urllib.request.urlopen(item['url'], timeout=120).read(item['bytes'] + 1))
        verify_file(archive, {k: item[k] for k in ('sha256', 'bytes')})
        subprocess.run(['7z', 'x', '-y', '-o' + str(wx), str(archive)], check=True,
                       stdout=subprocess.DEVNULL)
    for name, expected in lock['glew']['files'].items():
        target = out / 'sdk/glew' / name
        target.parent.mkdir(parents=True, exist_ok=True)
        data = urllib.request.urlopen(expected['url'], timeout=60).read(expected['bytes'] + 1)
        target.write_bytes(data)
        verify_file(target, {k: expected[k] for k in ('sha256', 'bytes')})
    manifest = json.loads((args.resources / 'manifest.json').read_text())
    if manifest['upstreamCommit'] != json.loads((ROOT / 'upstream.lock.json').read_text())['commit']:
        raise ValueError('Chart source identity differs')
    (out / 'resources').mkdir()
    for name, expected in manifest['files'].items():
        if len(safe_path(name).parts) != 1:
            raise ValueError('Non-flat chart resource')
        verify_file(args.resources / name, expected)
        shutil.copyfile(args.resources / name, out / 'resources' / name)
    for name in ('manifest.json', 'XNavChartResources.h'):
        shutil.copyfile(args.resources / name, out / 'resources' / name)
    receipt = {'schema': 1, 'sourceCommit': lock['source']['commit'],
               'gitlinkCommit': lock['gitlink']['commit'],
               'inputs': {p: record(ROOT / p, text=True) for p in INPUTS},
               'preparedFiles': {p.relative_to(out).as_posix(): record(p)
                                 for p in sorted(out.rglob('*')) if p.is_file()}}
    (out / 'preparation.json').write_text(json.dumps(receipt, indent=2) + '\n')
    verify_prepared(out)
    print('Prepared exact adapter sources; native build and acceptance remain pending.')



def package_dll(prepared, dll, output):
    import importlib.util
    import zipfile
    spec = importlib.util.spec_from_file_location('verify_ocharts', ROOT / 'tools/verify-ocharts-adapter-package.py')
    validator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(validator)
    receipt = verify_prepared(prepared)
    imports, exports = validator.pe_contract(dll.read_bytes())
    lock = json.loads((ROOT / LOCK).read_text())
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(dll, output / validator.DLL)
    # The corresponding source includes original unmodified upstream blobs,
    # retained license notices, and the exact applied source patches/build recipe.
    originals = prepared / 'original'
    archive_path = output / 'corresponding-source.zip'
    with zipfile.ZipFile(archive_path, 'w', compression=zipfile.ZIP_STORED) as archive:
        entries = {'product/' + p: (ROOT / p).read_bytes().replace(b'\r\n', b'\n') for p in INPUTS}
        entries.update({'source/' + p.relative_to(originals).as_posix(): p.read_bytes()
                        for p in originals.rglob('*') if p.is_file()})
        for name, data in sorted(entries.items()):
            info = zipfile.ZipInfo(name, (1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_STORED
            info.external_attr = 0o100644 << 16
            archive.writestr(info, data)
    manifest = {'schema': 1, 'kind': 'skager-ocharts-adapter', 'bindingVersion': 1,
                'source': {'repository': lock['source']['repository'], 'commit': lock['source']['commit'],
                           'gitlinkCommit': lock['gitlink']['commit'],
                           'manifestSha256': record(ROOT / LOCK, text=True)['sha256']},
                'inputs': receipt['inputs'],
                'chartResourceManifest': record(prepared / 'resources/manifest.json'),
                'chartResourceHeader': record(prepared / 'resources/XNavChartResources.h'),
                'dll': dict(record(output / validator.DLL), name=validator.DLL, machine='I386'),
                'imports': imports, 'exports': exports,
                'dependencies': {lib: {key: json.loads((prepared / 'sdk' / (lib + '-build.json')).read_text())[key]
                                       for key in ('version', 'source', 'outputs')}
                                 for lib in ('curl', 'zlib')},
                'correspondingSource': dict(record(archive_path), path=archive_path.name),
                'qualification': 'identity-only; native TLS, lifecycle, rendering and licensing gates remain'}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    validator.verify(output, prepared / 'resources')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--cache', type=Path)
    parser.add_argument('--curl-prefix', type=Path)
    parser.add_argument('--openssl-prefix', type=Path)
    parser.add_argument('--zlib-prefix', type=Path)
    parser.add_argument('--resources', type=Path)
    parser.add_argument('--verify-prepared', type=Path)
    parser.add_argument('--package-dll', type=Path)
    parser.add_argument('--prepared', type=Path)
    args = parser.parse_args()
    if args.package_dll:
        if not args.prepared or not args.output:
            parser.error('Packaging requires prepared/output')
        package_dll(args.prepared, args.package_dll, args.output)
    elif args.verify_prepared:
        verify_prepared(args.verify_prepared)
    else:
        if not all((args.output, args.cache, args.curl_prefix, args.openssl_prefix, args.zlib_prefix, args.resources)):
            parser.error('Preparation requires output/cache/curl-prefix/openssl-prefix/zlib-prefix/resources')
        prepare(args)


if __name__ == '__main__':
    main()

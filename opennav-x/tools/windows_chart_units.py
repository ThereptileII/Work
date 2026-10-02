"""Short native chart compile preflight; never a product/visual/link acceptance gate."""
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import struct
import subprocess
import sys
import tarfile

LOCAL_UNITS = tuple('src/integration/' + name + '.cpp' for name in (
    'ChartPresentation', 'ChartRouteWaypoint', 'ChartRouteUnderlay',
    'ChartRouteUnderlayGeometry', 'OnboardAisPresentation', 'OnlineAisOverlay'))
UPSTREAM_UNITS = tuple('gui/src/' + name + '.cpp' for name in (
    'chcanv', 'route_gui', 'route_point_gui', 'waypointman_gui', 'ais', 'piano')) + (
    'libs/s52plib/src/s52plib.cpp', 'libs/s52plib/src/chartsymbols.cpp')


def extract_locked_tree(archive, destination, archive_root):
    """Extract regular source files only, with bounds and Windows path guards."""
    destination.mkdir(parents=True, exist_ok=False)
    seen, total = set(), 0
    with tarfile.open(archive) as tar:
        for member in tar:
            path = PurePosixPath(member.name)
            if (path.is_absolute() or '..' in path.parts or not path.parts or
                    path.parts[0] != archive_root or '\\' in member.name or ':' in member.name):
                raise ValueError('Unsafe locked header archive path')
            if member.isdir():
                continue
            if not member.isfile() or len(path.parts) < 2 or member.size > 16 * 1024 * 1024:
                raise ValueError('Unexpected locked header archive member')
            relative = Path(*path.parts[1:])
            key = relative.as_posix().casefold()
            total += member.size
            if key in seen or total > 80 * 1024 * 1024 or len(seen) >= 10000:
                raise ValueError('Duplicate or oversized header archive')
            seen.add(key)
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(tar.extractfile(member).read())


def apply_dependency_patches(source, destination, library, api, evidence):
    # Patch the copied source using the exact ordered series in upstream CMake.
    # Use production's GNU-patch runner (including its whitespace policy), not
    # git-apply's different EOF handling on the bundled ShapeFileCpp tests.
    cmake = source / 'libs' / library / 'CMakeLists.txt'
    names = re.findall(r'^\s+([0-9][^\s]+\.patch)\s*$', cmake.read_text(), re.M)
    actual = {p.name for p in (cmake.parent / 'patches').glob('*.patch')}
    if not names or len(names) != len(set(names)) or set(names) != actual:
        raise ValueError('Unexpected upstream header patch series: ' + library)
    for name in names:
        patch = cmake.parent / 'patches' / name
        canonical = evidence / 'header-patches' / library / name
        canonical.parent.mkdir(parents=True, exist_ok=True)
        canonical.write_bytes(patch.read_bytes().replace(b'\r\n', b'\n'))
        api.run(['cmake', '-Dpatch_file=' + canonical.as_posix(),
                 '-Dpatch_dir=' + destination.as_posix(), '-P',
                 cmake.parent / 'cmake/PatchFile.cmake'],
                evidence / (library + '-' + name + '.log'), cwd=evidence)


def header_sources(source, sdk, api, evidence):
    locks = json.loads((api.ROOT / 'tools/windows-chart-headers.lock.json').read_text())
    for item in locks['archives']:
        archive = sdk / item['file']
        api.fetch(item, archive)
        destination = sdk / item['name']
        extract_locked_tree(archive, destination, item['archiveRoot'])
        apply_dependency_patches(source, destination, item['name'], api, evidence)
    shape = sdk / 'shapefile'
    shutil.copytree(source / 'libs/ShapefileCpp/ShapeFileCpp-b2681f8', shape)
    apply_dependency_patches(source, shape, 'ShapefileCpp', api, evidence)
    lock = json.loads((api.ROOT / 'tools/windows-prototype-headers.lock.json').read_text())
    for item in lock['files']:
        api.fetch(item, sdk / 'glew' / item['file'])
    # Headers only; no curl producer configure/build/test or OpenSSL download.
    curl = json.loads((api.ROOT / 'tools/windows-curl.lock.json').read_text())
    archive = sdk / curl['archive']
    api.fetch(curl, archive)
    with tarfile.open(archive) as tar:
        for member in tar:
            if not re.fullmatch(r'curl-8\.22\.0/include/curl/[a-z0-9_-]+\.h', member.name):
                continue
            if not member.isfile() or member.size > 1024 * 1024:
                raise ValueError('Unexpected curl header member')
            dest = sdk / 'curl/curl' / Path(member.name).name
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(tar.extractfile(member).read())


def verify_objects(build, units, record):
    expected = {'check_chart_' + Path(p).stem: Path(p).stem + '.obj' for p in units}
    objects = list(build.glob('check_chart_*.dir/Release/*.obj'))
    actual = {}
    for path in objects:
        target = path.parent.parent.name.removesuffix('.dir')
        if target not in expected or path.name != expected[target] or target in actual:
            raise ValueError('Unexpected/duplicate chart object')
        data = path.read_bytes()
        # Ordinary COFF or Microsoft's bigobj header, both native x86.
        machine = struct.unpack_from('<H', data, 0)[0] if len(data) >= 20 else 0
        if data[:4] == b'\0\0\xff\xff' and len(data) >= 56:
            machine = struct.unpack_from('<H', data, 6)[0]
        if machine != 0x14c:
            raise ValueError('Missing/non-x86 chart object: ' + path.name)
        actual[target] = record(path)
    if set(actual) != set(expected):
        raise ValueError('Missing native chart translation-unit objects')
    return actual


def compile_chart_units(args, evidence, api):
    if sys.platform != 'win32' or os.environ.get('GITHUB_ACTIONS') != 'true':
        raise ValueError('Chart preflight requires disposable native Windows CI, never the boat')
    source = api.ROOT / 'build/integration-source'
    upstream = api.ROOT / 'upstream/OpenCPN'
    lock = json.loads((api.ROOT / 'upstream.lock.json').read_text())
    if args.upstream.resolve() != upstream.resolve():
        if upstream.exists():
            raise ValueError('Default upstream exists; refusing to replace it')
        upstream.parent.mkdir(parents=True, exist_ok=True)
        api.run(['git', 'clone', '--no-hardlinks', args.upstream, upstream], evidence / 'clone.log')
        api.run(['git', '-C', upstream, 'checkout', '--detach', lock['commit']], evidence / 'checkout.log')
    api.run([sys.executable, api.ROOT / 'tools/prepare-integration.py'], evidence / 'prepare.log')
    # Record all production local/header/patch/generator inputs before configuring.
    tracked = subprocess.check_output(['git', 'ls-files'], cwd=api.ROOT, text=True).splitlines()
    roots = ('src/', 'cmake/', 'resources/chart-style/', 'patches/', 'tests/windows_changed_units/')
    names = {p for p in tracked if p.startswith(roots)} | {
        '.gitattributes', 'CMakeLists.txt', 'upstream.lock.json',
        'tests/windows_chart_units_tests.py', 'tests/windows_changed_units_runtime_tests.py',
        'tools/test-windows-changed-units.py', 'tools/windows_chart_units.py',
        'tools/prepare-integration.py', 'tools/verify-upstream.py',
        'tools/windows-chart-headers.lock.json', 'tools/windows-prototype-headers.lock.json',
        'tools/windows-wx.lock.json', 'tools/windows-curl.lock.json',
        'tools/generate-xnav-chart-style.py', 'tools/chart_raster_ink.py', 'tools/chart_anchor_art.py',
        'docs/design/prototype-tokens.json', 'docs/design/prototype/src/chart-marker-art.js',
        'docs/design/prototype/src/chart-symbols.js', 'docs/design/prototype/src/chart-symbols.css',
        'docs/design/prototype/src/style.css'}
    inputs = {p: api.record(api.ROOT / p) for p in sorted(names)}
    workflow_key, workflow = api.floating_workflow_input(api.ROOT)
    inputs[workflow_key] = workflow
    upstream_inputs = {p.relative_to(source).as_posix(): api.record(p)
        for p in source.rglob('*') if p.is_file() and
        (p.suffix in ('.h', '.hpp', '.in', '.cmake', '.patch') or p.name == 'CMakeLists.txt')}
    upstream_inputs.update({p: api.record(source / p) for p in UPSTREAM_UNITS})
    report = {'upstream': lock['commit'], 'localUnits': LOCAL_UNITS, 'upstreamUnits': UPSTREAM_UNITS,
              'localInputs': inputs, 'upstreamInputs': upstream_inputs,
              'policy': {'testFixtures': False, 'pilotLoopback': False, 'openglCompiled': True,
                         'dependencyBuilds': False, 'applicationLinked': False},
              'nativeProductAcceptance': False}
    def save():
        (evidence / 'chart-inputs.json').write_text(json.dumps(report, indent=2) + '\n')
    save()
    for path in UPSTREAM_UNITS:
        dest = evidence / 'source' / path
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / path, dest)
    # Keep unbuilt SDK libraries/archives out of the uploaded focused artifact.
    # Their locked archive identities and every consumed header hash are retained.
    for path in LOCAL_UNITS:
        dest = evidence / 'source/product' / path
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(api.ROOT / path, dest)
    sdk = api.ROOT / 'build/windows-chart-unit-sdk'
    sdk.mkdir(parents=True, exist_ok=False)
    wx = sdk / 'wx'
    for item in json.loads((api.ROOT / 'tools/windows-wx.lock.json').read_text())['archives']:
        archive = sdk / item['file']
        api.fetch(item, archive)
        api.run(['7z', 'x', '-y', '-o' + str(wx), archive], evidence / (item['file'] + '.log'))
    header_sources(source, sdk, api, evidence)
    resources = evidence / 'chart-resources'
    api.run([sys.executable, api.ROOT / 'tools/generate-xnav-chart-style.py',
             '--source', source / 'data/s57data', '--output', resources], evidence / 'resources.log')
    sdk_inputs = {p.relative_to(sdk).as_posix(): api.record(p) for p in sdk.rglob('*')
                  if p.is_file() and p.suffix in ('.h', '.hpp') and '.git' not in p.parts}
    generated = {p.name: api.record(p) for p in resources.iterdir() if p.is_file()}
    manifest = json.loads((resources / 'manifest.json').read_text())
    if manifest['upstreamCommit'] != lock['commit'] or any(
            generated[name] != item for name, item in manifest['files'].items()):
        raise ValueError('Generated chart resources do not match manifest')
    report.update(sdkHeaders=sdk_inputs, generatedResources=generated)
    save()
    build = evidence / 'build'
    api.run(['cmake', '-S', api.ROOT / 'tests/windows_changed_units', '-B', build,
             '-G', 'Visual Studio 17 2022', '-A', 'Win32', '-DOPENNAV_CHECK_CHART_UNITS=ON',
             '-DOPENNAV_SOURCE_DIR:PATH=' + source.as_posix(),
             '-DOPENNAV_CHART_LOCAL=' + ';'.join(LOCAL_UNITS),
             '-DOPENNAV_CHART_UPSTREAM=' + ';'.join(UPSTREAM_UNITS),
             '-DOPENNAV_CHART_SDK:PATH=' + sdk.as_posix(),
             '-DOPENNAV_CHART_RESOURCES:PATH=' + resources.as_posix(),
             '-DwxWidgets_ROOT_DIR:PATH=' + wx.as_posix(),
             '-DwxWidgets_LIB_DIR:PATH=' + (wx / 'lib/vc14x_dll').as_posix(),
             '-DwxWidgets_CONFIGURATION=mswu'], evidence / 'configure.log')
    targets = ['check_chart_' + Path(p).stem for p in LOCAL_UNITS + UPSTREAM_UNITS]
    projects = {p.stem: p for p in build.glob('check_chart_*.vcxproj')}
    if set(projects) != set(targets):
        raise ValueError('Unexpected chart compile project inventory')
    for path in projects.values():
        text = path.read_text()
        if re.search(r'NOMINMAX|OPENNAV_\w+_TEST|ForcedIncludeFiles|PrecompiledHeader>Use', text):
            raise ValueError('Chart project masks production headers/macros')
        if not all(value in text for value in ('XNAV_ENABLE_TEST_FIXTURES=0',
                'XNAV_ENABLE_PILOT_LOOPBACK_TESTS=0', 'ocpnUSE_GL', 'OPENNAV_X=1')):
            raise ValueError('Chart project lacks production integration policy')
    report['generatedConfig'] = api.record(build / 'include/config.h')
    report['projects'] = {name: api.record(path) for name, path in projects.items()}
    save()
    api.run(['cmake', '--build', build, '--config', 'Release', '--target', *targets,
             '--parallel', '2', '--', '/verbosity:normal'], evidence / 'compile.log', timeout=600)
    report['objects'] = verify_objects(build, LOCAL_UNITS + UPSTREAM_UNITS, api.record)
    # Revalidate actual source and generation identity, not just reported status.
    for directory, records in ((api.ROOT, inputs), (source, upstream_inputs),
                               (sdk, sdk_inputs), (resources, generated)):
        if any(api.record(directory / p) != item for p, item in records.items()):
            raise ValueError('Chart compile input drift')
    if api.floating_workflow_input(api.ROOT) != (workflow_key, workflow):
        raise ValueError('Chart workflow identity changed')
    api.run([sys.executable, api.ROOT / 'tools/prepare-integration.py'], evidence / 'verify-source.log')
    report['compileOnlyPassed'] = True
    save()
    return {'chartPreflight': report, 'nativeProductAcceptance': False}

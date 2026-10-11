#!/usr/bin/env python3
"""Build an inert SCM broker beside an authenticated same-commit launcher closure.

The caller authenticates the original native launcher producer/transfer. Local
hash checks do not authenticate remote provenance or provision/activate a broker.
Use a separate install copy; ordinary launcher-only producer outputs stay intact.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import zipfile

import updater_package as updater

ROOT = Path(__file__).resolve().parent.parent
_spec = importlib.util.spec_from_file_location('_broker_launcher_producer', ROOT / 'tools/package-update-launcher.py')
launcher = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(launcher)
BINARY = 'skager-update-broker.exe'
RECORD_RELATIVE = 'opennav/third-party/update-broker/build.json'
BUILD_RECIPE = ('CGO_ENABLED=0 GOOS=windows GOARCH=386 GO386=sse2 go build '
                '-mod=readonly -trimpath -buildvcs=true -ldflags=-H=windowsgui ./cmd/skager-update-broker')
MAX_REFERENCE_BYTES = 16 << 20


def prepare_outputs(install):
    install = launcher.plain_path(install, 'Broker install copy')
    if not install.is_dir():
        raise ValueError('Existing verified launcher install copy required')
    binary = launcher.plain_path(install / BINARY, 'Broker binary output')
    record = launcher.plain_path(install / RECORD_RELATIVE, 'Broker build record output')
    if binary.exists() or record.parent.exists():
        raise ValueError('Refusing to overwrite existing broker outputs')
    return install, binary, record


def source_inventory(module, goroot, cache, modules, reference):
    """Reuse the launcher inventory rules, without downloading or archiving again."""
    inventory = launcher.Inventory()
    tracked = launcher.run(['git', 'ls-files', '-z', '--', 'tools/update-verifier',
                            'tools/package-update-launcher.py', 'tools/package-update-broker.py',
                            'tools/broker_package.py', 'LICENSE'], ROOT).split('\0')
    tracked = [name for name in tracked if name]
    required = {'LICENSE', 'tools/package-update-launcher.py', 'tools/package-update-broker.py',
                'tools/broker_package.py', 'tools/update-verifier/go.mod',
                'tools/update-verifier/go.sum', 'tools/update-verifier/cmd/skager-update-broker/main.go'}
    if not required.issubset(tracked):
        raise ValueError('Committed complete broker source and dependency locks required')
    for name in tracked:
        if Path(name).suffix.lower() in ('.exe', '.dll', '.pdb', '.test', '.log') or Path(name).name == 'verifier-tests.json':
            raise ValueError('Generated output is not broker corresponding source')
        inventory.add(ROOT / name, 'project/' + name)
    declared = {item['path']: item for item in reference['modules']}
    seen = set()
    mains = 0
    for item in modules:
        if item.get('Replace') or item.get('Error'):
            raise ValueError('Replaced or unresolved broker module refused')
        if item.get('Main'):
            mains += 1
            if item['Path'] != launcher.MAIN_MODULE or Path(item.get('Dir', '')).resolve() != module.resolve():
                raise ValueError('Unexpected broker main module')
            continue
        path = item.get('Path')
        expected = declared.get(path)
        if (path in seen or expected is None or item.get('Version') != expected['version'] or
                item.get('Sum') != expected['sum'] or item.get('GoModSum') != expected['goModSum']):
            raise ValueError('Broker module graph differs from retained launcher source')
        seen.add(path)
        inventory.tree(launcher.module_cache_source(item['Dir'], cache), expected['sourcePrefix'])
        inventory.add(launcher.module_cache_source(item['GoMod'], cache),
                      'module-locks/' + path + '@' + item['Version'] + '.mod')
    if mains != 1 or seen != set(declared):
        raise ValueError('Complete retained launcher module graph required')
    inventory.tree(goroot / 'src', launcher.GO_VERSION + '/src')
    for name in ('LICENSE', 'VERSION', 'PATENTS', 'AUTHORS', 'CONTRIBUTORS'):
        if name in ('LICENSE', 'VERSION') or (goroot / name).is_file():
            inventory.add(goroot / name, launcher.GO_VERSION + '/' + name)
    return inventory


def check_source(inventory, descriptor):
    """Bind the existing complete archive to actual current build inputs."""
    archive = launcher.plain_path(descriptor['archive'], 'Retained updater source archive')
    if launcher.digest(archive) != descriptor['sha256']:
        raise ValueError('Retained updater source archive changed')
    with zipfile.ZipFile(archive) as source:
        entries = source.infolist()
        names = [item.filename for item in entries]
        if (len(names) != len(set(names)) or len(names) > launcher.MAX_FILES + 1 or
                set(names) != set(inventory.files) | {'SOURCE_REFERENCE.json', 'provisioning/go.sum'} or
                any(item.file_size > (MAX_REFERENCE_BYTES if item.filename == 'SOURCE_REFERENCE.json' else launcher.MAX_FILE_BYTES)
                    or item.flag_bits & 1 for item in entries) or
                sum(item.file_size for item in entries) > launcher.MAX_SOURCE_BYTES + MAX_REFERENCE_BYTES):
            raise ValueError('Retained updater source archive inventory differs')
        reference = json.loads(source.read('SOURCE_REFERENCE.json').decode('utf-8'), object_pairs_hook=updater._unique)
        updater._keys(reference, (*descriptor['reference'].keys(), 'files'))
        if {key: value for key, value in reference.items() if key != 'files'} != descriptor['reference']:
            raise ValueError('Retained source reference differs from launcher descriptor')
        files = reference['files']
        if not isinstance(files, dict) or set(files) != set(names) - {'SOURCE_REFERENCE.json'}:
            raise ValueError('Retained source file declarations differ')
        for name, record in files.items():
            updater._keys(record, ('bytes', 'sha256'))
            if type(record['bytes']) is not int or record['bytes'] != source.getinfo(name).file_size:
                raise ValueError('Retained source size differs')
            updater._digest(record['sha256'])
            with source.open(name) as content:
                if hashlib.file_digest(content, 'sha256').hexdigest() != record['sha256']:
                    raise ValueError('Retained source member hash differs')
            if name in inventory.files:
                item = inventory.files[name]
                current = launcher.plain_path(item['source'], 'Broker corresponding-source input')
                if current.stat().st_size != record['bytes'] or launcher.digest(current) != record['sha256']:
                    raise ValueError('Actual broker build input differs from retained source')


def build_info(text, commit):
    if not isinstance(text, str) or len(text) > 65536:
        raise ValueError('Broker Go build identity unavailable')
    lines = text.splitlines()
    if not lines or not lines[0].endswith(': ' + launcher.GO_VERSION):
        raise ValueError('Broker compiler build version differs')
    paths, settings = [], {}
    for line in lines[1:]:
        fields = line.strip().split('\t')
        if fields[0] == 'path':
            if len(fields) != 2:
                raise ValueError('Malformed broker Go path')
            paths.append(fields[1])
        if fields[0] == '=>':
            raise ValueError('Replaced broker build dependency refused')
        if fields[0] == 'build':
            if len(fields) != 2 or '=' not in fields[1]:
                raise ValueError('Malformed broker Go build setting')
            key, value = fields[1].split('=', 1)
            if key in settings:
                raise ValueError('Duplicate broker Go build setting')
            settings[key] = value
    required = {'-buildmode': 'exe', '-compiler': 'gc', '-trimpath': 'true', '-ldflags': '-H=windowsgui',
                'CGO_ENABLED': '0', 'GOARCH': '386', 'GOOS': 'windows', 'GO386': 'sse2',
                'vcs': 'git', 'vcs.revision': commit, 'vcs.modified': 'false'}
    if (paths != [launcher.MAIN_MODULE + '/cmd/skager-update-broker'] or
            any(settings.get(key) != value for key, value in required.items()) or
            not re.fullmatch(r'\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z', settings.get('vcs.time', ''))):
        raise ValueError('Actual broker Go build identity differs from selected source')
    return BINARY + ': ' + launcher.GO_VERSION + '\n' + '\n'.join(lines[1:]) + '\n'


def package(install, commit, go):
    if not launcher.is_native_windows():
        raise ValueError('Broker packaging requires native Windows')
    install, final_binary, final_record = prepare_outputs(install)
    descriptor = updater.verify_updater_package(install, commit)
    if launcher.run(['git', 'rev-parse', 'HEAD'], ROOT).strip() != commit:
        raise ValueError('Broker source is not the selected product commit')
    launcher.require_clean_sources('Before broker packaging')
    module = ROOT / 'tools/update-verifier'
    compiler = launcher.provisioned_source_root(shutil.which(go) or go, 'Go compiler')
    go = str(compiler)
    environment = dict(os.environ, GOOS='windows', GOARCH='386', GO386='sse2', CGO_ENABLED='0',
                       GOTOOLCHAIN='local', GOWORK='off', GOFLAGS='-mod=readonly', GOEXPERIMENT='')
    tool = json.loads(launcher.run([go, 'env', '-json', 'GOVERSION', 'GOROOT', 'GOHOSTOS', 'GOMOD', 'GOMODCACHE'], module, environment))
    if (tool['GOVERSION'] != launcher.GO_VERSION or tool['GOHOSTOS'] != 'windows' or
            Path(tool['GOMOD']).resolve() != (module / 'go.mod').resolve()):
        raise ValueError('Pinned native Go 1.27.1 and exact broker module required')
    goroot = launcher.provisioned_source_root(tool['GOROOT'], 'GOROOT')
    cache = launcher.provisioned_source_root(tool['GOMODCACHE'], 'GOMODCACHE')
    if compiler != launcher.plain_path(goroot / 'bin/go.exe', 'Pinned broker compiler'):
        raise ValueError('Selected broker compiler is outside pinned GOROOT')
    if (goroot / 'VERSION').read_text(encoding='utf-8').splitlines()[0] != launcher.GO_VERSION:
        raise ValueError('Broker compiler and source version differ')
    environment.update(GOROOT=str(goroot), GOMODCACHE=str(cache))
    launcher.run([go, 'mod', 'verify'], module, environment)
    modules = launcher.json_stream(launcher.run([go, 'list', '-mod=readonly', '-m', '-json', 'all'], module, environment))
    inventory = source_inventory(module, goroot, cache, modules, descriptor['reference'])
    check_source(inventory, descriptor)
    with tempfile.TemporaryDirectory(prefix='.broker-build-', dir=install) as temporary:
        staging = Path(temporary)
        binary = staging / BINARY
        launcher.run([go, 'build', '-mod=readonly', '-trimpath', '-buildvcs=true', '-ldflags=-H=windowsgui',
                      '-o', str(binary), './cmd/skager-update-broker'], module, environment)
        launcher.run([go, 'mod', 'verify'], module, environment)
        launcher.require_clean_sources('Product sources changed during broker packaging')
        check_source(inventory, descriptor)
        if updater.verify_updater_package(install, commit) != descriptor:
            raise ValueError('Launcher source closure changed during broker build')
        launcher.validate_pe32(binary)
        info = build_info(launcher.run([go, 'version', '-m', str(binary)], module, environment), commit)
        source_bundle = {'archive': updater.SOURCE_RELATIVE, 'path': descriptor['path'],
                         'sha256': descriptor['sha256'], 'reference': descriptor['reference']}
        record = {'schema': 1, 'productCommit': commit, 'goVersion': launcher.GO_VERSION,
                  'binary': {'path': BINARY, 'sha256': launcher.digest(binary)},
                  'sourceBundle': source_bundle, 'buildInfo': info, 'build': BUILD_RECIPE}
        record_file = staging / 'build.json'
        record_file.write_text(json.dumps(record, indent=2) + '\n', encoding='utf-8')
        prepare_outputs(install)
        # Exclusive hard links cannot replace racing outputs. Preserve any
        # partial publication as refusal evidence rather than silently retrying.
        launcher.plain_path(final_record.parent.parent, 'Broker record parent').mkdir(parents=True, exist_ok=True)
        final_record.parent.mkdir()
        os.link(record_file, final_record)
        os.link(binary, final_binary)
        if launcher.digest(final_binary) != record['binary']['sha256'] or final_record.read_bytes() != record_file.read_bytes():
            raise ValueError('Published broker bytes changed')
        return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install', type=Path, required=True)
    parser.add_argument('--commit', default=os.environ.get('GITHUB_SHA', ''))
    parser.add_argument('--go', default='go')
    args = parser.parse_args()
    try:
        result = package(args.install, args.commit, args.go)
    except (ValueError, OSError, KeyError, zipfile.BadZipFile, subprocess.SubprocessError) as error:
        raise SystemExit(str(error)) from error
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

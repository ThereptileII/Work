#!/usr/bin/env python3
"""Resource-only Python/CMake identity proof; no app, SDK or dependency build."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import urllib.request
import zlib

ROOT = Path(__file__).resolve().parents[1]


def record(path):
    data = path.read_bytes()
    return {'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}


def inventory(directory):
    result = {p.name: record(p) for p in sorted(directory.iterdir()) if p.is_file()}
    expected = {'manifest.json', 'XNavChartResources.h', 'chartsymbols.xml',
                'S52RAZDS.RLE', 'rastersymbols-day.png', 'rastersymbols-dusk.png',
                'rastersymbols-dark.png'}
    assert set(result) == expected, 'Unexpected generated resource inventory'
    manifest = json.loads((directory / 'manifest.json').read_text())
    for name, value in manifest['files'].items():
        assert result[name] == value, 'Generated resource seal is inconsistent'
    return result


def exact(before, after):
    assert before == after, 'Generated resource byte identities differ'


def run(command, log):
    with log.open('w') as stream:
        result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
    assert result.returncode == 0, f'Command failed; retained {log}'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--source', type=Path, help='Optional existing hash-locked upstream s57data')
    parser.add_argument('--entry-version', default='3.12.10')
    parser.add_argument('--contract-only', action='store_true', help='Cheap CMake identity and byte-refusal checks only')
    args = parser.parse_args()
    out = args.evidence.resolve()
    out.mkdir(parents=True, exist_ok=True)
    assert not (out / 'result.json').exists(), 'Fresh evidence required'
    entry = Path(sys.executable).resolve()
    facts = {'executable': str(entry), 'binary': record(entry), 'version': sys.version,
             'zlibCompile': zlib.ZLIB_VERSION, 'zlibRuntime': zlib.ZLIB_RUNTIME_VERSION,
             'zlibNg': getattr(zlib, 'ZLIBNG_VERSION', None)}
    receipt = {'entry': facts, 'passed': False, 'contractOnly': args.contract_only,
               'sourceCommit': os.environ.get('GITHUB_SHA') or subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
               'collector': record(Path(__file__)), 'generator': record(ROOT / 'tools/generate-xnav-chart-style.py'),
               'sourceLock': record(ROOT / 'resources/chart-style/v1/source-lock.json'),
               'sourceFiles': {}, 'runs': {}, 'scope': 'Resources and CMake discovery only; no native module or app build/load'}
    def save():
        (out / 'result.json').write_text(json.dumps(receipt, indent=2) + '\n')
    save()
    try:
        if not args.contract_only:
            assert sys.platform == 'win32' and sys.version.split()[0] == args.entry_version, 'Expected failed production entry interpreter'
        # The exact raw-byte comparator must refuse equal-sized changed payloads.
        try:
            exact({'file': {'sha256': 'a', 'bytes': 10}}, {'file': {'sha256': 'b', 'bytes': 10}})
        except AssertionError:
            receipt['equalSizeDifferentBytesRejected'] = True
        else:
            raise AssertionError('Byte identity negative control accepted')
        source = args.source.resolve() if args.source else out / 'source'
        lock = json.loads((ROOT / 'resources/chart-style/v1/source-lock.json').read_text())
        if not args.contract_only:
            source.mkdir(parents=True, exist_ok=True)
            for name, wanted in lock['files'].items():
                path = source / name
                if not path.exists():
                    url = f"https://raw.githubusercontent.com/OpenCPN/OpenCPN/{lock['upstreamCommit']}/data/s57data/{name}"
                    with urllib.request.urlopen(url, timeout=60) as response:
                        data = response.read(wanted['bytes'] + 1)
                    assert len(data) == wanted['bytes'] and hashlib.sha256(data).hexdigest() == wanted['sha256']
                    path.write_bytes(data)
                assert record(path) == wanted, 'Pinned input resource differs'
                receipt['sourceFiles'][name] = wanted
                save()
        cmake = out / 'cmake-source'
        cmake.mkdir()
        identity_code = 'import json,sys,zlib,pathlib,hashlib; p=pathlib.Path(sys.executable).resolve(); pathlib.Path(sys.argv[1]).write_text(json.dumps({"executable":str(p),"version":sys.version,"sha256":hashlib.sha256(p.read_bytes()).hexdigest(),"zlibCompile":zlib.ZLIB_VERSION,"zlibRuntime":zlib.ZLIB_RUNTIME_VERSION,"zlibNg":getattr(zlib,"ZLIBNG_VERSION",None)},indent=2))'
        (cmake / 'identity.py').write_text(identity_code)
        (cmake / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.20)
project(resource_python NONE)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/identity.py" "${CMAKE_BINARY_DIR}/python.json" COMMAND_ERROR_IS_FATAL ANY)
if(GENERATOR_SCRIPT)
  execute_process(COMMAND "${Python3_EXECUTABLE}" "${GENERATOR_SCRIPT}" --source "${RESOURCE_SOURCE}" --output "${CMAKE_BINARY_DIR}/resources" COMMAND_ERROR_IS_FATAL ANY)
endif()
''')
        if not args.contract_only:
            run([str(entry), str(ROOT / 'tools/generate-xnav-chart-style.py'), '--source', str(source), '--output', str(out / 'entry-resources')], out / 'entry-generation.log')
            receipt['entryResources'] = inventory(out / 'entry-resources')
            save()
        for label, pinned in [('unpinned', False), ('development', True), ('production', True)]:
            build = out / label
            command = ['cmake', '-S', str(cmake), '-B', str(build)]
            if pinned:
                command.append('-DPython3_EXECUTABLE:FILEPATH=' + str(entry))
            if not args.contract_only:
                command += ['-DGENERATOR_SCRIPT:FILEPATH=' + str(ROOT / 'tools/generate-xnav-chart-style.py'),
                            '-DRESOURCE_SOURCE:PATH=' + str(source)]
            receipt['runs'][label] = {'command': command}
            save()
            run(command, out / (label + '.log'))
            chosen = json.loads((build / 'python.json').read_text())
            receipt['runs'][label]['python'] = chosen
            if pinned:
                assert os.path.samefile(chosen['executable'], entry), 'CMake interpreter drifted despite explicit pin'
                assert chosen['sha256'] == facts['binary']['sha256']
            if not args.contract_only:
                values = inventory(build / 'resources')
                receipt['runs'][label]['resources'] = values
                receipt['runs'][label]['differentFiles'] = [n for n in values if values[n] != receipt['entryResources'][n]]
                save()
                if pinned:
                    exact(values, receipt['entryResources'])
                else:
                    # Record actual compressed bytes and independently compare decoded
                    # pixels. Pixel equality is diagnostic only, never acceptance.
                    sys.path.insert(0, str(ROOT / 'tools'))
                    from chart_raster_ink import decode
                    receipt['unpinnedDecodedPixelEquality'] = {n: decode((build / 'resources' / n).read_bytes())[1] == decode((out / 'entry-resources' / n).read_bytes())[1] for n in values if n.endswith('.png')}
                    receipt['unpinnedSemanticManifestEquality'] = {k:v for k,v in json.loads((build/'resources/manifest.json').read_text()).items() if k != 'files'} == {k:v for k,v in json.loads((out/'entry-resources/manifest.json').read_text()).items() if k != 'files'}
            save()
        if not args.contract_only:
            receipt['unboundFailureReproduced'] = bool(receipt['runs']['unpinned']['differentFiles'])
            assert receipt['unboundFailureReproduced'], 'No unpinned byte drift reproduced; retain facts before diagnosing further'
        receipt['passed'] = True
        print('CMake interpreter and byte-refusal contract passed; no resource generation' if args.contract_only else 'Resource Python identity proof passed; see exact interpreter and seven-file receipts')
    finally:
        save()


if __name__ == '__main__':
    main()

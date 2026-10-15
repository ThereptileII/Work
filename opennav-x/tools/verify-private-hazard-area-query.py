#!/usr/bin/env python3
"""SCRUM-283 actual private area query; focused Linux or authenticated MSVC x86 wx fixture.

No app build, chart ingestion, GL tessellation/rendering, vendor helper or boat I/O.
The native caller must authenticate wx using the existing SDK workflow first.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
CORE_FILES = ('model/src/georef.cpp', 'model/include/model/georef.h',
              'gui/src/ocpn_plugin_gui.cpp')
PRIVATE_BODIES = {
    'src/eSENCChart.cpp': ('void S57Obj::Init()', 'int S57Obj::GetAttributeIndex( ',
                         'bool eSENCChart::IsPointInObjArea(',
                         'ListOfS57Obj *eSENCChart::GetAssociatedObjects('),
    'libs/geoprim/src/bbox.cpp': ('void LLBBox::Set(', 'bool LLBBox::Contains('),
    'libs/geoprim/src/poly_math.cpp': ('int CCW(MyPoint p0,', 'int Intersect(MyPoint p1,',
                                      'int G_PtInPolygon(MyPoint *rgpts,'),
    'libs/s52plib/src/mygeomOC.cpp': ('Extended_Geometry::~Extended_Geometry()',
                                    'PolyTessGeo::PolyTessGeo()', 'PolyTessGeo::~PolyTessGeo()',
                                    'PolyTriGroup::PolyTriGroup()', 'PolyTriGroup::~PolyTriGroup()',
                                    'TriPrim::TriPrim()', 'TriPrim::~TriPrim()', 'void TriPrim::FreeMem()'),
    'libs/s52plib/src/s52cnsy.cpp': ('bool GetIntAttr(S57Obj *obj,',
                                   'bool GetDoubleAttr(S57Obj *obj,', 'static wxString *_UDWHAZ03('),
}
CORE_BODIES = {
    'model/src/georef.cpp': ('void toSM(double lat,', 'void fromSM(double x,'),
    'gui/src/ocpn_plugin_gui.cpp': ('void toSM_Plugin(double lat,', 'void fromSM_Plugin(double x,'),
}


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def digest(text):
    return hashlib.sha256(text.encode()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    if not __debug__:
        raise ValueError('Run without -O: fixture assertions are required')
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('core', 'private-original', 'private-patched', 'wx-prefix', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--native-sdk-receipt', type=Path)
    args = parser.parse_args()
    binding = load('area_query_binding', ROOT / 'tools/verify-private-hazard-association.py')
    extract = binding.load('area_query_extract', ROOT / 'tools/verify-anchor-loader.py').function
    record, inventory = binding.record, binding.inventory
    args.core, args.private_original, args.private_patched = map(
        binding.plain_path, (args.core, args.private_original, args.private_patched))
    out = args.output.absolute()
    out.mkdir(parents=True, exist_ok=False)
    binding.plain_path(out)
    # Bind helpers/locks before executing production patch composition.
    harness_names = (
        'tools/verify-private-hazard-area-query.py', 'tests/private_hazard_area_query_test.cpp',
        'tools/verify-private-hazard-association.py', 'tools/verify-anchor-loader.py',
        'tools/prepare-ocharts-adapter.py', 'tools/ocharts-adapter-source.lock.json',
        'upstream.lock.json', 'tools/windows-wx.lock.json') + binding.LOCAL_HEADERS
    harness = {name: record(ROOT / name) for name in harness_names}
    sources = binding.bind_sources(args.core, args.private_original, args.private_patched)
    for name in CORE_FILES:
        actual = record(args.core / name)
        pinned = subprocess.check_output(['git', '-C', str(args.core), 'show', sources['coreCommit'] + ':' + name])
        if (args.core / name).read_bytes().replace(b'\r\n', b'\n') != pinned.replace(b'\r\n', b'\n'):
            raise ValueError('Core geometry differs from pin: ' + name)
        sources['core'][name] = actual
    write_json(out / 'source-inputs.json', sources)

    native = sys.platform == 'win32'
    env = dict(os.environ)
    for key in ('CL', '_CL_', 'CXXFLAGS', 'CFLAGS', 'CPATH', 'CPLUS_INCLUDE_PATH', 'C_INCLUDE_PATH'):
        if env.get(key):
            raise ValueError('Inherited compiler override refused: ' + key)
    wx_prefix = args.wx_prefix.resolve()
    compiler = shutil.which('cl' if native else 'g++')
    if not compiler:
        raise ValueError('Required fixture compiler not found')
    compiler = Path(compiler).resolve()
    toolchain = {'compilerPath': str(compiler), 'compiler': record(compiler),
                 'python': sys.version, 'qualification': 'Fixture compiler identity only, not a complete SDK/toolchain qualification'}
    sdk = None
    if native:
        if env.get('VSCMD_ARG_TGT_ARCH') != 'x86' or not args.native_sdk_receipt:
            raise ValueError('Initialized MSVC x86 and existing authenticated SDK receipt required')
        json.loads(args.native_sdk_receipt.read_text(encoding='utf-8-sig'))
        sdk = {'receipt': record(args.native_sdk_receipt),
               'authentication': 'Existing workflow caller prerequisite; no new SDK qualification',
               'wxFiles': inventory(wx_prefix)}
        wxlib = wx_prefix / 'lib/vc14x_dll'
        for name in ('wxbase32u.lib', 'wxmsw32u_core.lib', 'wxbase32u_vc14x.dll', 'wxmsw32u_core_vc14x.dll'):
            record(wxlib / name)
        flags = ['/DWXUSINGDLL', '/D_UNICODE', '/DUNICODE', '/D__WXMSW__',
                 '/I' + str(wx_prefix / 'include'), '/I' + str(wxlib / 'mswu')]
        libs = ['/link', '/MACHINE:X86', '/LIBPATH:' + str(wxlib), 'wxbase32u.lib', 'wxmsw32u_core.lib']
        env['PATH'] = str(wxlib) + os.pathsep + env.get('PATH', '')
    else:
        wx = [str(wx_prefix / 'bin/wx-config'), '--prefix=' + str(wx_prefix)]
        flags = shlex.split(subprocess.check_output(wx + ['--cxxflags'], text=True))
        libs = shlex.split(subprocess.check_output(wx + ['--libs', 'core,base'], text=True))
        env['LD_LIBRARY_PATH'] = str(wx_prefix / 'lib')
        toolchain.update(compilerVersion=subprocess.check_output([str(compiler), '--version'], text=True),
                         wxVersion=subprocess.check_output(wx + ['--version'], text=True).strip(),
                         wxConfig=record((wx_prefix / 'bin/wx-config').resolve()))
    write_json(out / 'harness.json', {
        'commit': subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
        'files': harness, 'platform': sys.platform, 'nativeSdk': sdk, 'toolchain': toolchain,
        'scope': 'Extracted actual methods; minimal chart state class and synthetic already-tessellated geometry. No application ABI, real chart, software/GL or boat acceptance.'})

    receipts, logs, query_bodies = [], {}, {}
    for kind, source, patched, mutant in (
            ('original-private', args.private_original, False, False),
            ('repaired-private', args.private_patched, True, False),
            ('polygon-bypass-negative', args.private_patched, True, True)):
        folder = out / kind
        folder.mkdir()
        fixture = folder / 'fixture'
        fixture.mkdir()
        source_group = 'patched' if patched else 'original'
        staged = [(source / name, Path(name).name, sources[source_group][name])
                  for name in (*binding.SOURCE_HEADERS, 'libs/geoprim/src/poly_math.h')]
        staged += [(args.core / 'model/include/model/georef.h', 'georef.h', sources['core']['model/include/model/georef.h'])]
        staged += [(ROOT / name, name.removeprefix('src/'), harness[name]) for name in binding.LOCAL_HEADERS]
        staged += [(ROOT / 'tests/private_hazard_area_query_test.cpp', 'private_hazard_area_query_test.cpp',
                    harness['tests/private_hazard_area_query_test.cpp'])]
        for path, name, identity in staged:
            target = fixture / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(path.read_bytes())
            if record(target) != identity:
                raise ValueError('Input changed before staging: ' + str(path))

        # Exact declaration slice avoids unused GL headers/libraries. No geometry
        # algorithm or data structure is substituted. GL tessellation is not run.
        geometry = (source / 'libs/s52plib/src/mygeom.h').read_text()
        declarations = geometry[geometry.index('class OGRGeometry;'):geometry.rindex('#endif')]
        (fixture / 'geometry-types.inc').write_text(declarations)
        chart_header = (source / 'src/eSENCChart.h').read_text()
        for declaration in ('ObjRazRules *razRules[PRIO_NUM][LUPNAME_NUM];',
                            'double      m_ref_lat;', 'double      m_ref_lon;',
                            'ListOfS57Obj *GetAssociatedObjects( S57Obj *obj );',
                            'bool IsPointInObjArea( float lat, float lon, float select_radius, S57Obj *obj );'):
            if declaration not in chart_header:
                raise ValueError('Fixture chart declaration no longer matches locked source: ' + declaration)
        bodies = {}
        # Dependency order: coordinate and polygon functions precede query.
        for root, mapping in ((args.core, CORE_BODIES), (source, {
                name: PRIVATE_BODIES[name] for name in (
                    'libs/geoprim/src/bbox.cpp', 'libs/geoprim/src/poly_math.cpp',
                    'libs/s52plib/src/mygeomOC.cpp', 'src/eSENCChart.cpp',
                    'libs/s52plib/src/s52cnsy.cpp')})):
            for name, signatures in mapping.items():
                text = (root / name).read_text()
                for signature in signatures:
                    bodies[name + ':' + signature] = extract(text, signature)
        area_key = 'src/eSENCChart.cpp:bool eSENCChart::IsPointInObjArea('
        query_key = 'src/eSENCChart.cpp:ListOfS57Obj *eSENCChart::GetAssociatedObjects('
        query_bodies[kind] = {key: digest(bodies[key]) for key in (area_key, query_key)}
        unmodified = {key: digest(value) for key, value in bodies.items()}
        if mutant:
            assert bodies[area_key].count('return ret;') == 1
            bodies[area_key] = bodies[area_key].replace('return ret;', 'return true;')
        (fixture / 'actual-bodies.inc').write_text(''.join(bodies.values()))
        if patched:
            bridge = extract((source / 'src/eSENCChart.cpp').read_text(),
                             'static std::list<S57Obj *> *SkagerAssociatedObjects(')
            (fixture / 'bridge.inc').write_text(bridge)
        compiled = inventory(fixture)
        binary = folder / ('test.exe' if native else 'test')
        defines = ['PATCHED_PRIVATE', 'SKAGER_OCHARTS_ADAPTER'] if patched else []
        if native:
            command = [str(compiler), '/nologo', '/std:c++17', '/utf-8', '/O2', '/EHsc', '/MD',
                       '/D_CRT_SECURE_NO_WARNINGS', *('/D' + d for d in defines), *flags,
                       '/I' + str(fixture), str(fixture / 'private_hazard_area_query_test.cpp'),
                       '/Fo' + str(folder) + os.sep, '/Fe' + str(binary), *libs]
        else:
            command = [str(compiler), '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                       '-Wno-unused-parameter', '-Wno-deprecated-copy', '-Wno-unused-but-set-variable',
                       *('-D' + d for d in defines), *flags, '-I' + str(fixture),
                       str(fixture / 'private_hazard_area_query_test.cpp'), *libs, '-o', str(binary)]
        result = subprocess.run(command, env=env, text=True, capture_output=True, timeout=120)
        (folder / 'compile.log').write_text(result.stdout + result.stderr)
        entry = {'kind': kind, 'command': command, 'compiledInputs': compiled,
                 'unmodifiedBodySha256': unmodified, 'compileExit': result.returncode,
                 'mutation': 'IsPointInObjArea final return true instead of ret' if mutant else None}
        receipts.append(entry)
        write_json(out / 'receipt.json', receipts)
        result.check_returncode()
        if native:
            pe = binary.read_bytes()
            offset = int.from_bytes(pe[60:64], 'little')
            assert pe[:2] == b'MZ' and pe[offset:offset + 4] == b'PE\0\0' and int.from_bytes(pe[offset + 4:offset + 6], 'little') == 0x14c
            entry['machine'] = 'I386'
        result = subprocess.run([str(binary)], env=env, text=True, capture_output=True, timeout=30)
        (folder / 'run.log').write_text(result.stdout + result.stderr)
        entry.update(runExit=result.returncode, binary=record(binary))
        logs[kind] = result.stdout
        write_json(out / 'receipt.json', receipts)
        if mutant:
            assert result.returncode == 1 and 'point-outside-polygon-inside-bounds' in result.stderr
        else:
            result.check_returncode()
        if kind == 'original-private':
            negative = subprocess.run([str(binary), 'require-repair'], env=env, text=True, capture_output=True, timeout=30)
            (folder / 'required-repair-negative.log').write_text(negative.stdout + negative.stderr)
            assert negative.returncode == 1 and 'original-missing-query-negative' in negative.stderr
            entry['requiredRepairNegativeExit'] = negative.returncode
        binding.verify_identity(fixture, compiled, exact=True)
        print(kind, result.returncode, result.stdout.splitlines()[-1:], flush=True)
    assert query_bodies['original-private'] == query_bodies['repaired-private']
    queries = lambda text: [line for line in text.splitlines() if line.startswith('query ')]
    assert queries(logs['original-private']) == queries(logs['repaired-private'])
    assert len(queries(logs['repaired-private'])) == 29
    for root, key in ((args.core, 'core'), (args.private_original, 'original'),
                      (args.private_patched, 'patched'), (ROOT, 'patches')):
        binding.verify_identity(root, sources[key], exact=key in ('original', 'patched'))
    binding.verify_identity(ROOT, harness)
    assert record(compiler) == toolchain['compiler'], 'Compiler changed during fixture'
    if sdk:
        binding.verify_identity(wx_prefix, sdk['wxFiles'], exact=True)
        assert record(args.native_sdk_receipt) == sdk['receipt']
    write_json(out / 'receipt.json', receipts)
    write_json(out / 'parity.json', {
        'queryCases': 29, 'originalRepairedQueryBodiesIdentical': True,
        'originalRepairedQueryResultsIdentical': True, 'originalMissingCallbackNegativeRejected': True,
        'polygonBypassMutationRejected': True, 'repairedActualQueryConditionalLinkage': True,
        'scope': 'Actual area query and polygon/coordinate bodies using explicit synthetic tessellation; synchronous callback and borrowed list lifetime. Native only if this receipt platform is win32. No chart ingestion, tessellation generation, private DLL ABI, real-chart rendering or boat qualification.'})
    print('29 actual query cases agree; repaired actual-query callback linkage passed; both negatives rejected')


if __name__ == '__main__':
    main()

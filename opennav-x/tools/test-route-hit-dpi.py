#!/usr/bin/env python3
"""Exercise pinned native Select methods/geometry without a chart or display.

The actual prepared sources supply SelectCtx, Select, selection methods and
Mercator/vector math. Inert list/route/window boundaries replace wx runtime
objects only. --unpatched keeps the old fixed-pixel defect as a failing control.
Native wx DPI/event delivery remains a separate acceptance gate.
"""
import argparse
import ast
import os
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PIN = '37fd0cddb7334fe489e9f18aa163977a9c5c84f7'
PATCH = 'patches/opencpn-5.12.4-route-hit-dpi.patch'
FILES = ('model/include/model/select.h', 'model/src/select.cpp', 'gui/src/chcanv.cpp')


def block(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--unpatched', action='store_true')
    args = parser.parse_args()
    upstream, output = args.upstream.resolve(), args.output.resolve()
    if output.exists():
        raise SystemExit('Use a fresh focused-test output directory')
    output.mkdir(parents=True)
    if subprocess.check_output(['git', '-C', str(upstream), 'rev-parse', 'HEAD'],
                               text=True).strip() != PIN:
        raise SystemExit('Exact pinned OpenCPN source required')

    def pinned(name):
        return subprocess.check_output(['git', '-C', str(upstream), 'show',
                                        PIN + ':' + name]).decode('utf-8')

    fixture = output / 'source'
    for name in FILES:
        path = fixture / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(pinned(name), encoding='utf-8')
    subprocess.run(['git', '-C', str(fixture), 'init', '-q'], check=True)
    tree = ast.parse((ROOT / 'tools/prepare-integration.py').read_text())
    assignment = next(node for node in tree.body if isinstance(node, ast.Assign)
                      and any(isinstance(t, ast.Name) and t.id == 'patches'
                              for t in node.targets))
    patches = [node.right.value for node in assignment.value.elts]
    if patches.count(PATCH) != 1:
        raise SystemExit('Route hit DPI patch missing or duplicated in actual build')
    for patch in patches:
        if args.unpatched and patch == PATCH:
            continue
        subprocess.run(['git', '-C', str(fixture), 'apply',
                        *['--include=' + name for name in FILES], '-'],
                       input=(ROOT / patch).read_bytes().replace(b'\r\n', b'\n'),
                       check=True)
    header, select, canvas = ((fixture / name).read_text() for name in FILES)
    calls = re.findall(r'SelectCtx ctx\([^;]+\);', canvas)
    if len(calls) != 15:
        raise SystemExit('Unexpected native canvas query inventory')
    if not args.unpatched and any(
            'FromDIP(100) * m_displayScale / 100.0' not in call for call in calls):
        raise SystemExit('A native canvas query omitted its physical DIP scale')
    # Compile one actual canvas construction expression, not a test copy of
    # its conversion. Windows displayScale is 1; Retina physical pixels use 2.
    query = ('SelectCtx CanvasQuery() {\n' + calls[0] + '\nreturn ctx;\n}\n')
    georef = pinned('model/src/georef.cpp')
    constants = pinned('model/include/model/georef.h')
    constants = constants[constants.index('#ifndef PI'):
                          constants.index('static const double WGSinvf')]
    vector = pinned('model/src/navutil_base.cpp')
    production = '\n'.join([
        '' if args.unpatched else '#define HAS_ROUTE_HIT_DIP_SCALE 1',
        '\n'.join(line for line in header.splitlines() if line.startswith('#define SELTYPE_')),
        block(header, 'struct SelectCtx {') + ';',
        block(header, 'class Select {') + ';',
        constants,
        block(georef, 'void toSM('),
        block(georef, 'void DistanceBearingMercator('),
        *[block(vector, signature) for signature in (
            'double vGetLengthOfNormal(', 'double vDotProduct(',
            'pVector2D vSubtractVectors(', 'double vVectorSquared(',
            'double vVectorMagnitude(')],
        block(select, 'Select::Select('), block(select, 'Select::~Select('),
        block(select, 'bool Select::IsSegmentSelected('),
        block(select, 'void Select::CalcSelectRadius('),
        block(select, 'SelectItem *Select::FindSelection('),
        block(select, 'bool Select::IsSelectableSegmentSelected('),
        block(select, 'static bool is_selectable_wp('),
        block(select, 'SelectableItemList Select::FindSelectionList('),
        'struct CanvasQueryFixture {\n'
        'bool m_bShowNavobjects=true; double dpi=1, m_displayScale=1;\n'
        'int FromDIP(int value) { return int(std::lround(value*dpi)); }\n'
        'double GetCanvasTrueScale() { return dpi*m_displayScale; }\n'
        'double GetScaleValue() { return 10000; }\n' + query + '};\n',
    ])
    (output / 'production-route-hit.h').write_text(production, encoding='utf-8')
    executable = output / ('route-hit-dpi.exe' if os.name == 'nt' else 'route-hit-dpi')
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-O2',
                    '-Wall', '-Wextra', '-Werror', '-I' + str(output),
                    '-I' + str(upstream / 'libs/geoprim/src'),
                    str(ROOT / 'tests/route_hit_dpi_tests.cpp'), '-o', str(executable)],
                   check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    main()

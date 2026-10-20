#!/usr/bin/env python3
"""Focused source SVG, provenance, native-role and complete-inventory checks.

--render compares controlled CSS to flattened SVG using librsvg and creates
source-only contact sheets. It is not native wx/OpenGL or chart acceptance.
"""
import argparse
import importlib.util
import json
import re
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import chart_symbol_svg as svg


def module(name, filename):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def render(text, output, width, height):
    subprocess.run(['rsvg-convert', '-w', str(width), '-h', str(height),
                    '-o', str(output)], input=text.encode(), check=True)


def pixels(path):
    return subprocess.check_output(['magick', str(path), '-depth', '8', 'rgba:-'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--styled', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--render', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    original, styled = args.source.read_bytes(), args.styled.read_bytes()
    items = svg.entries(original, styled)
    assert len(items) == 129 and len({item['name'] for item in items}) == 43
    native = json.loads((svg.ASSETS / 'native-points.json').read_text())
    # Exercise the real immutable HPGL importer against pinned source, not a
    # duplicated expected path list in the test.
    derive = module('derive_native', 'derive-native-point-art.py')
    assert derive.derive(args.source) == native
    audit = module('audit_symbols', 'audit-chart-symbol-coverage.py')
    coverage = audit.audit(args.source, args.styled)
    recorded = json.loads((svg.ASSETS / 'symbol-coverage.json').read_text())
    assert coverage == recorded, 'Coverage inventory is stale'
    assert len(coverage['symbols']) == 1105
    assert all(item['reason'] for item in coverage['symbols'])
    assert len({item['id'] for item in coverage['symbols']}) == 1105
    assert sum(len(item['shadowedRcids']) for item in coverage['shadowedDefinitions']) == 75
    assert coverage['countByKind']['point'] == {'definitions': 1093, 'effectiveNames': 1018}
    assert {'OUTLW', 'CHYLW'} == set(native['symbols']['BCNCAR01']['colorRoles'])
    assert native['symbols']['OBSTRN01']['colorRoles'] == ['DEPVS', 'CHBLK']
    assert native['symbols']['OBSTRN03']['colorRoles'] == ['DEPIT', 'CHBLK']
    assert native['symbols']['OBSTRN18']['colorRoles'] == ['CSTLN', 'LANDA']
    assert native['symbols']['WRECKS01']['colorRoles'] == ['CHGRD']
    assert 'BCNDEF13' not in svg.NATIVE_NAMES and 'OBSTRN11' not in svg.NATIVE_NAMES
    for name, item in native['symbols'].items():
        tx, ty, scale = svg.native_transform(item)
        pivot, bitmap = item['nativePivot'], item['bitmap']
        assert abs(tx + pivot['x'] * scale - bitmap['pivot']['x']) < 1e-9, name
        assert abs(ty + pivot['y'] * scale - bitmap['pivot']['y']) < 1e-9, name
        x, y, width, height = item['geometry']['viewBox']
        assert tx+x*scale >= -1e-9 and ty+y*scale >= -1e-9
        assert tx+(x+width)*scale <= bitmap['width']+1e-9
        assert ty+(y+height)*scale <= bitmap['height']+1e-9
        left, top, right, bottom = svg.painted_bounds(item, tx, ty, scale)
        assert min(left, top) >= -1e-9, name
        assert right <= bitmap['width']+1e-9 and bottom <= bitmap['height']+1e-9, name
    for item in items:
        tree = ET.fromstring(item['svg'])
        assert tree.get('viewBox') == f"0 0 {item['width']} {item['height']}"
        assert 'class=' not in item['svg'] and '<style' not in item['svg']
        if item['name'] in native['symbols']:
            actual = tree.find('{http://www.w3.org/2000/svg}g').get('transform')
            values = list(map(float, re.fullmatch(
                r'translate\(([^ ]+) ([^)]+)\) scale\(([^)]+)\)', actual).groups()))
            tx, ty, scale = values
            source = native['symbols'][item['name']]
            assert abs(tx+source['nativePivot']['x']*scale-source['bitmap']['pivot']['x']) < 1e-5
            assert abs(ty+source['nativePivot']['y']*scale-source['bitmap']['pivot']['y']) < 1e-5
            left, top, right, bottom = svg.painted_bounds(source, tx, ty, scale)
            assert min(left, top) >= -1e-5
            assert right <= source['bitmap']['width']+1e-5
            assert bottom <= source['bitmap']['height']+1e-5
        if item['name'] in {'OBSTRN01', 'OBSTRN03', 'OBSTRN18'}:
            role = {'OBSTRN01':'DEPVS','OBSTRN03':'DEPIT','OBSTRN18':'LANDA'}[item['name']]
            color = ET.fromstring(original).find(
                "color-tables/color-table[@name='"+item['scheme']+"']/color[@name='"+role+"']")
            expected = '#' + ''.join(f'{int(color.get(c)):02x}' for c in ('r','g','b'))
            assert any(n.get('fill') == expected for n in tree.iter()), 'Hazard categorical fill changed'
    # Fail closed on CSS outside the controlled source subset.
    for bad in ('.a path{fill:#ffffff}', '.a:hover{fill:#ffffff}'):
        try:
            svg.normalize('<svg xmlns="http://www.w3.org/2000/svg"><style>' + bad + '</style></svg>')
        except AssertionError:
            pass
        else:
            raise AssertionError('Unsupported CSS accepted')
    fixtures = {'b66e6c': 'bd6765', 'd3948c': 'da8f85', '508d78': '4a937a',
                '8fbaa2': '8bbea2', '885e57': '8d5a52', '5e7a67': '597b64'}
    for source, expected in fixtures.items():
        night = source in {'885e57', '5e7a67'}
        rgb = tuple(bytes.fromhex(source))
        target = svg.richer_rgb(rgb, night)
        assert bytes(target).hex() == expected
        if night:
            assert sum(v*w for v,w in zip(target, (2126,7152,722))) <= sum(v*w for v,w in zip(rgb, (2126,7152,722)))
    header = '#pragma once\nnamespace opennav::chart_style::generated {\n'
    header += '\n'.join(svg.header_lines(items)) + '\n}\n'
    (args.output / 'XNavChartResources.h').write_text(header)
    (args.output / 'header-check.cpp').write_text(
        '#include "XNavChartResources.h"\n'
        'static_assert(sizeof(opennav::chart_style::generated::symbol_svgs)/'
        'sizeof(opennav::chart_style::generated::symbol_svgs[0]) == 129);\n')
    subprocess.run(['c++', '-std=c++17', '-fsyntax-only', str(args.output / 'header-check.cpp')], check=True)
    receipt = {'pointNames': 43, 'schemeEntries': 129, 'catalogueEffectiveEntries': 1105,
               'shadowedPointDefinitions': 75, 'nativeSourceGeometryReproduced': 15,
               'nativeAnchorsAndUnclippedBounds': 15,
               'headerSyntax': 'pass', 'renderedCssEquivalencePairs': 0}
    if args.render:
        # Compare identical intended colors before the owner-requested richer
        # color delta. This isolates whether flattening changes geometry/alpha.
        with tempfile.TemporaryDirectory() as temporary:
            temp = Path(temporary)
            for family in svg.FAMILIES:
                svg.verified_proof(family)
                for path in sorted((svg.ASSETS / family).glob('*.svg')):
                    raw = path.read_text()
                    flat = svg.normalize(raw)
                    tree = ET.fromstring(raw)
                    for factor in (1, 1.25, 1.5):
                        width = round(float(tree.get('width')) * factor)
                        height = round(float(tree.get('height')) * factor)
                        render(raw, temp/'raw.png', width, height)
                        render(flat, temp/'flat.png', width, height)
                        assert pixels(temp/'raw.png') == pixels(temp/'flat.png'), \
                            'CSS flattening changed raster: ' + str(path) + ' scale ' + str(factor)
                        receipt['renderedCssEquivalencePairs'] += 1
        effective = {node.findtext('name'): node for node in ET.fromstring(styled).findall('symbols/symbol')}
        font = '/usr/share/fonts/liberation/LiberationSans-Regular.ttf'
        for scheme, sheet in [('DAY_BRIGHT', 'day'), ('DUSK', 'dusk'), ('NIGHT', 'dark')]:
            cells = []
            theme_items = [item for item in items if item['scheme'] == scheme]
            for item in theme_items:
                name = item['name']
                bitmap = effective[name].find('bitmap')
                position = bitmap.find('graphics-location')
                width, height = item['width'], item['height']
                for percent in (100, 125, 150):
                    w, h = round(width * percent / 100), round(height * percent / 100)
                    output = args.output / f'{name}-{scheme}-{percent}.png'
                    render(item['svg'], output, w, h)
                    old = args.output / f'{name}-{scheme}-{percent}-before.png'
                    subprocess.run(['magick', str(args.styled.parent/f'rastersymbols-{sheet}.png'),
                                    '-crop', f'{width}x{height}+{position.get("x")}+{position.get("y")}',
                                    '+repage', '-resize', f'{w}x{h}!', str(old)], check=True)
                    cell = args.output / f'{name}-{scheme}-{percent}-cell.png'
                    # Enlarge the final-size raster for inspection only.
                    subprocess.run(['magick', str(old), str(output), '+append',
                                    '-filter', 'point', '-resize', '400%', '-background',
                                    {'DAY_BRIGHT':'#d5e5e5','DUSK':'#344f59','NIGHT':'#121e24'}[scheme],
                                    '-alpha', 'remove', '-alpha', 'off', '-gravity', 'center',
                                    '-extent', '210x180', '-gravity', 'south', '-fill',
                                    '#ffffff' if scheme != 'DAY_BRIGHT' else '#222222',
                                    '-font', font, '-pointsize', '12',
                                    '-annotate', '+0+7', f'{name} {percent}%  atlas | SVG', str(cell)], check=True)
                    cells.append(str(cell))
            subprocess.run(['magick', 'montage', *cells, '-tile', '9x', '-geometry', '+3+3',
                            str(args.output/f'contact-{scheme}.png')], check=True)
        receipt['renderedFinalSizePoints'] = 43 * 3 * 3
        receipt['renderLimit'] = 'librsvg source render only; native wx/NanoSVG and GL acceptance remain separate.'
    (args.output/'receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
    print(json.dumps(receipt))


if __name__ == '__main__':
    main()

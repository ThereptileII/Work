"""Source-locked scalable point artwork for the existing generated header.

Only reviewed point assets are embedded. Unknown symbols keep their native
portrayal. CSS is resolved here because the native SVG loader does not implement
stylesheets. No runtime style inference, classification or size gain occurs.
"""
import colorsys
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'resources/chart-style/v1'
SCHEMES = ('DAY_BRIGHT', 'DUSK', 'NIGHT')
FAMILIES = ('anchorage', 'services', 'hazards', 'cardinals', 'seamarks',
            'special-buoy', 'generic-beacon', 'yellow-buoy')
CHROMATIC_FAMILIES = {'cardinals', 'seamarks', 'yellow-buoy'}
SATURATION_GAIN = 1.20
NATIVE_BEACONS = (
    'BCNCAR01', 'BCNCAR02', 'BCNCAR03', 'BCNCAR04', 'BCNISD21',
    'BCNLAT15', 'BCNLAT16', 'BCNSAW13', 'BCNSPP13')
NATIVE_HAZARDS = ('OBSTRN01', 'OBSTRN02', 'OBSTRN03', 'OBSTRN18',
                  'WRECKS01', 'WRECKS04')
NATIVE_NAMES = NATIVE_BEACONS + NATIVE_HAZARDS
PAINT = {'fill', 'stroke', 'stroke-width', 'stroke-linecap', 'stroke-linejoin',
         'stroke-miterlimit', 'fill-opacity', 'stroke-opacity', 'fill-rule',
         'stroke-dasharray', 'stroke-dashoffset'}
IGNORED = {'pointer-events', 'shape-rendering'}
ET.register_namespace('', 'http://www.w3.org/2000/svg')


def digest(content):
    return hashlib.sha256(content).hexdigest()


def declarations(text):
    result = {}
    for declaration in text.split(';'):
        if not declaration.strip():
            continue
        key, value = map(str.strip, declaration.split(':', 1))
        assert key in PAINT | IGNORED | {'opacity'}, 'Unsupported SVG style: ' + key
        result[key] = value
    return result


def normalize(svg):
    """Resolve the controlled simple-class CSS; retain group compositing."""
    root = ET.fromstring(svg)
    assert root.tag == '{http://www.w3.org/2000/svg}svg'
    rules = []
    for style in list(root):
        if style.tag.rsplit('}', 1)[-1] != 'style':
            continue
        text = style.text or ''
        blocks = list(re.finditer(r'([^{}]+)\{([^{}]*)\}', text))
        assert ''.join(m[0] for m in blocks) == text
        for order, match in enumerate(blocks):
            selector = match[1].strip()
            assert re.fullmatch(r'(?:\.[\w-]+)+', selector)
            classes = set(selector.split('.')[1:])
            rules.append((len(classes), order, classes, declarations(match[2])))
        root.remove(style)

    def visit(node, inherited):
        tag = node.tag.rsplit('}', 1)[-1]
        assert tag in {'svg', 'g', 'path', 'circle', 'rect', 'ellipse',
                       'line', 'polyline', 'polygon'}, 'Unsupported SVG node: ' + tag
        own = {key: value for key, value in node.attrib.items()
               if key in PAINT | {'opacity'}}
        classes = set(node.get('class', '').split())
        for _, _, selector, values in sorted(rules, key=lambda r: r[:2]):
            if selector <= classes:
                own.update(values)
        own.update(declarations(node.get('style', '')))
        effective = {**inherited, **{k: v for k, v in own.items() if k in PAINT}}
        for key in list(node.attrib):
            if key in PAINT | IGNORED | {'class', 'style', 'opacity', 'aria-hidden'}:
                del node.attrib[key]
        if tag not in {'svg', 'g'}:
            node.attrib.update(effective)
        # Opacity is not an inherited property. Keep its original group
        # boundary so intersecting paths retain the same compositing.
        if 'opacity' in own:
            node.set('opacity', own['opacity'])
        for child in node:
            visit(child, effective)
    visit(root, {'fill': '#000000', 'stroke': 'none'})
    result = ET.tostring(root, encoding='unicode')
    assert '<style' not in result and 'class=' not in result
    for node in root.iter():
        for key in ('fill', 'stroke'):
            if key in node.attrib:
                assert node.get(key) == 'none' or re.fullmatch(r'#[0-9a-fA-F]{6}', node.get(key))
        assert not any('href' in key for key in node.attrib)
    return result


def verified_proof(family):
    proof = json.loads((ASSETS / family / 'provenance.json').read_text())
    for path, sha in proof['sources'].items():
        assert digest((ROOT / path).read_bytes().replace(b'\r\n', b'\n')) == sha, \
            'Scalable artwork provenance changed: ' + path
    return proof


def vivid_aids(svg, scheme):
    """Owner-requested derived-color delta; immutable source SVGs stay intact."""
    tokens = json.loads((ROOT / 'docs/design/prototype-tokens.json').read_text())
    theme = {'DAY_BRIGHT': 'day', 'DUSK': 'dusk', 'NIGHT': 'night'}[scheme]
    mapping = {}
    for role in ('--mark-red', '--mark-green', '--mark-yellow'):
        raw = tokens['themes'][theme][role]
        rgb = tuple(int(raw[i:i+2], 16) for i in (1, 3, 5))
        if scheme == 'NIGHT':
            rgb = tuple(round(v * .78) for v in rgb)
        target = richer_rgb(rgb, scheme == 'NIGHT')
        mapping['#' + ''.join(f'{v:02x}' for v in rgb)] = \
            '#' + ''.join(f'{v:02x}' for v in target)
    tree = ET.fromstring(svg)
    for node in tree.iter():
        for paint in ('fill', 'stroke'):
            value = node.get(paint)
            if value in mapping:
                node.set(paint, mapping[value])
    return ET.tostring(tree, encoding='unicode')


def richer_rgb(rgb, night=False):
    hue, light, saturation = colorsys.rgb_to_hls(*(v / 255 for v in rgb))
    target = tuple(math.floor(v * 255 + .5) for v in
                   colorsys.hls_to_rgb(hue, light, min(1, saturation * SATURATION_GAIN)))
    weights = (2126, 7152, 722)
    before = sum(v * weight for v, weight in zip(rgb, weights))
    after = sum(v * weight for v, weight in zip(target, weights))
    if night and after > before:
        # Night's existing weighted brightness is a ceiling. Uniform scaling
        # preserves the richer hue; downward rounding cannot exceed the cap.
        target = tuple(v * before // after for v in target)
    return target


def painted_bounds(item, tx, ty, scale):
    """Actual path/circle bounds including the final logical pen/dot size."""
    bounds = []
    for shape in item['geometry']['shapes']:
        stroke = 0 if shape.get('fill') else max(shape['width'] * 3 * scale, .8) / 2
        if shape['type'] == 'circle':
            radius = shape['r'] * scale
            if shape.get('fill') and shape['r'] == shape['width'] * 5:
                radius = max(radius, .45)
            x, y = tx + shape['cx'] * scale, ty + shape['cy'] * scale
            radius += stroke
            bounds.append((x-radius, y-radius, x+radius, y+radius))
        else:
            # The verified native importer emits only M/L paths.
            number = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
            assert re.fullmatch(r'[ML\s]*', re.sub(number, '', shape['d']))
            values = list(map(float, re.findall(number, shape['d'])))
            assert len(values) % 2 == 0
            xs = [tx+x*scale for x in values[::2]]
            ys = [ty+y*scale for y in values[1::2]]
            bounds.append((min(xs)-stroke, min(ys)-stroke, max(xs)+stroke, max(ys)+stroke))
    return (min(b[0] for b in bounds), min(b[1] for b in bounds),
            max(b[2] for b in bounds), max(b[3] for b in bounds))


def native_transform(item):
    """Map the anchor exactly and reserve the actual final painted extent."""
    width, height = item['bitmap']['width'], item['bitmap']['height']
    x, y, box_width, box_height = item['geometry']['viewBox']
    px, py = (item['nativePivot'][key] for key in ('x', 'y'))
    bx, by = (item['bitmap']['pivot'][key] for key in ('x', 'y'))
    constraints = [(bx, px-x), (by, py-y),
                   (width-bx, x+box_width-px), (height-by, y+box_height-py)]
    assert all(available > 0 and extent > 0 for available, extent in constraints)
    scale = min(available / extent for available, extent in constraints)
    def fits(value):
        left, top, right, bottom = painted_bounds(item, bx-px*value, by-py*value, value)
        return left >= 0 and top >= 0 and right <= width and bottom <= height
    if not fits(scale):
        low, high = 0, scale
        assert fits(low)
        for _ in range(48):
            middle = (low+high)/2
            if fits(middle):
                low = middle
            else:
                high = middle
        scale = low
    return bx-px*scale, by-py*scale, scale


def native_svg(item, palette, scheme, beacon=False):
    """Redraw reviewed native geometry, including its categorical color roles."""
    width, height = item['bitmap']['width'], item['bitmap']['height']
    tx, ty, scale = native_transform(item)
    root = ET.Element('svg', xmlns='http://www.w3.org/2000/svg',
                      width=str(width), height=str(height),
                      viewBox=f'0 0 {width} {height}')
    group = ET.SubElement(root, 'g', transform=f'translate({tx:.9g} {ty:.9g}) scale({scale:.9g})')
    for shape in item['geometry']['shapes']:
        # Native SW values are pen widths, not the hundredth-millimetre
        # coordinate unit. Keep a readable 0.8px minimum at logical size.
        rgb = palette[shape['color']]
        if beacon and shape['color'] in {'CHRED', 'CHGRN', 'CHYLW'}:
            rgb = richer_rgb(rgb, scheme == 'NIGHT')
        ink = '#' + ''.join(f'{v:02x}' for v in rgb)
        attributes = {'fill': ink if shape.get('fill') else 'none',
                      'stroke': 'none' if shape.get('fill') else ink,
                      'stroke-width': f'{max(shape["width"] * 3, .8 / scale):.9g}',
                      'stroke-linecap': 'round', 'stroke-linejoin': 'round'}
        if shape['type'] == 'path':
            attributes['d'] = shape['d']
        else:
            attributes.update({key: str(shape[key]) for key in ('cx', 'cy', 'r')})
            # Preserve all obstruction boundary dots: the prototype importer
            # records a native PD-without-coordinates as a five-unit disk.
            if shape.get('fill') and shape['r'] == shape['width'] * 5:
                attributes['r'] = f'{max(shape["r"], .45 / scale):.9g}'
        ET.SubElement(group, shape['type'], attributes)
    return ET.tostring(root, encoding='unicode')


def entries(original_xml, styled_xml):
    original, styled = ET.fromstring(original_xml), ET.fromstring(styled_xml)
    effective = {node.findtext('name'): node for node in styled.findall('symbols/symbol')}
    result = []
    for family in FAMILIES:
        proof = verified_proof(family)
        for path in sorted((ASSETS / family).glob('*.svg')):
            stem = path.stem
            themed = next((scheme for scheme in SCHEMES if stem.endswith('-' + scheme)), None)
            name = stem[:-(len(themed) + 1)] if themed else stem
            assert len(name) == 8 and name in effective
            source = path.read_text()
            # The provenance must explicitly cover the exact SVG being used.
            # Provenance keys are repository-relative POSIX paths.  Keep the
            # generated metadata byte-stable on Windows as well as Linux;
            # ``str(Path)`` uses backslashes on Windows and otherwise makes a
            # verified SVG look missing from its proof.
            source_path = path.relative_to(ROOT).as_posix()
            assert source_path in proof['sources']
            for scheme in (themed,) if themed else SCHEMES:
                svg = source
                if not themed:
                    role = 'hazardRgb' if family == 'hazards' else 'serviceRgb'
                    ink = '#' + ''.join(f'{v:02x}' for v in proof[role][scheme])
                    svg = svg.replace('#ffffff', ink)
                normalized = normalize(svg)
                if family in CHROMATIC_FAMILIES:
                    normalized = vivid_aids(normalized, scheme)
                bitmap = effective[name].find('bitmap')
                width, height = int(bitmap.get('width')), int(bitmap.get('height'))
                tree = ET.fromstring(normalized)
                assert tree.get('viewBox') == f'0 0 {width} {height}'
                assert float(tree.get('width')) == width and float(tree.get('height')) == height
                result.append(dict(name=name, scheme=scheme, svg=normalized,
                                   width=width, height=height, source=source_path,
                                   sourceSha256=digest(path.read_bytes().replace(b'\r\n', b'\n')),
                                   kind='prototype-art',
                                   chromaticSaturationGain=SATURATION_GAIN if family in CHROMATIC_FAMILIES else 1))
    native = json.loads((ASSETS / 'native-points.json').read_text())
    assert set(native['symbols']) == set(NATIVE_NAMES)
    assert digest((ROOT / native['parser']).read_bytes().replace(b'\r\n', b'\n')) == native['parserSha256']
    for name, item in native['symbols'].items():
        stock = original.findall("symbols/symbol[name='" + name + "']")[-1]
        assert digest(ET.tostring(stock)) == item['sourceNodeSha256'], 'Native point source changed: ' + name
        assert ET.tostring(stock) == ET.tostring(effective[name]), 'Native point semantics changed: ' + name
        for scheme in SCHEMES:
            table = styled.find("color-tables/color-table[@name='" + scheme + "']")
            palette = {color.get('name'): tuple(int(color.get(c)) for c in ('r', 'g', 'b'))
                       for color in table.findall('color')}
            if name in NATIVE_HAZARDS:
                # These fills encode water-level/depth categories. The atlas
                # retained them while the general chart-area palette changed;
                # applying that area palette here would wash out hazards.
                source_table = original.find("color-tables/color-table[@name='" + scheme + "']")
                for role in ('DEPVS', 'DEPIT', 'LANDA', 'CSTLN'):
                    color = source_table.find("color[@name='" + role + "']")
                    palette[role] = tuple(int(color.get(c)) for c in ('r', 'g', 'b'))
            svg = normalize(native_svg(item, palette, scheme, name in NATIVE_BEACONS))
            result.append(dict(name=name, scheme=scheme, svg=svg,
                               width=item['bitmap']['width'], height=item['bitmap']['height'],
                               source='resources/chart-style/v1/native-points.json',
                               sourceSha256=digest((ASSETS / 'native-points.json').read_bytes()),
                               kind='native-geometry-redraw'))
    assert len({(v['name'], v['scheme']) for v in result}) == len(result)
    return sorted(result, key=lambda item: (item['name'], item['scheme']))


def header_lines(items):
    lines = ['struct SymbolSvg { const char* name; const char* scheme; const char* svg; int width; int height; };',
             'inline constexpr SymbolSvg symbol_svgs[] = {']
    for item in items:
        assert ')XNSVG"' not in item['svg']
        lines.append('  {"%s", "%s", R"XNSVG(%s)XNSVG", %d, %d},' %
                     (item['name'], item['scheme'], item['svg'], item['width'], item['height']))
    return lines + ['};']

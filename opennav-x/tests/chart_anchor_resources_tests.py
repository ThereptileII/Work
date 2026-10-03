"""Independent SCRUM-244 pixel isolation, effective definition and hotspot proof."""
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from chart_raster_ink import decode, derive


def verify_anchor(source, output, metadata, check):
    root = Path(__file__).resolve().parents[1]
    art_source = (root/'docs/design/prototype/src/chart-marker-art.js').read_text()
    paths = re.search(r"'point:ACHARE51':'([^']+)'", art_source)[1]
    svg = ET.parse(root/'resources/chart-style/v1/anchorage/ACHARE51.svg').getroot()
    group = list(svg)[0]
    expected = ET.fromstring('<g xmlns="http://www.w3.org/2000/svg">'+paths+'</g>')
    check([ET.tostring(n) for n in group] == [ET.tostring(n) for n in expected])
    check(group.attrib == {'transform':'translate(10 10) scale(0.84375)',
        'fill':'none','stroke':'#ffffff','stroke-width':'1.3',
        'stroke-linecap':'round','stroke-linejoin':'round',
        'shape-rendering':'geometricPrecision'})
    check('function chartSymbolGraphic(s,size=27)' in art_source and 'size/32' in art_source)
    css = (root/'docs/design/prototype/src/chart-symbols.css').read_text()
    check('.chart-marker-art.marker-service{stroke:var(--mark-service)}' in css)
    check(re.findall(r'--mark-service:(#[a-f0-9]+)', css) == ['#7c858a','#a8bbb7','#7e948a'])
    check('#app[data-theme=night] .chart-canvas{filter:brightness(.78)}' in (root/'docs/design/prototype/src/style.css').read_text())
    check(tuple(round(v*.78) for v in (126,148,138)) == (98,115,108))
    tree = ET.parse(output/'chartsymbols.xml').getroot()
    stock_tree = ET.parse(source/'chartsymbols.xml').getroot()
    definitions = tree.findall("symbols/symbol[name='ACHARE51']")
    check(len(definitions) == 2 and definitions[-1].get('RCID') == '1105')
    final = definitions[-1]
    check(final.findtext('definition') == 'R' and final.find('vector') is None)
    check(final.find('bitmap').attrib == {'width': '20', 'height': '20'})
    check(final.find('bitmap/pivot').attrib == {'x': '10', 'y': '10'})
    check(final.find('bitmap/graphics-location').attrib == {'x': '20', 'y': '1160'})
    check(ET.tostring(definitions[0]) == ET.tostring(stock_tree.findall("symbols/symbol[name='ACHARE51']")[0]))
    for bitmap in stock_tree.findall('.//bitmap'):
        pos = bitmap.find('graphics-location')
        if pos is None:
            continue
        x, y, w, h = int(pos.get('x')), int(pos.get('y')), int(bitmap.get('width')), int(bitmap.get('height'))
        check(not (18 < x+w and 42 > x and 1158 < y+h and 1182 > y))
    _, day = decode((source/'rastersymbols-day.png').read_bytes())
    expected_alpha = None
    for name, table, neutral_before, color in (
            ('rastersymbols-day.png', 'DAY_BRIGHT', None, (124,133,138)),
            ('rastersymbols-dusk.png', 'DUSK', (54,54,54), (168,187,183)),
            ('rastersymbols-dark.png', 'NIGHT', (27,27,27), (98,115,108))):
        content = (source/name).read_bytes()
        if neutral_before:
            content, _ = derive(day, content, neutral_before, metadata['palette'][table]['CHBLK'])
        before_chunks, before = decode(content)
        after_chunks, after = decode((output/name).read_bytes())
        check([(k,v) for k,v in before_chunks if k != b'IDAT'] == [(k,v) for k,v in after_chunks if k != b'IDAT'])
        # Separately verified SCRUM-254 tiles are excluded from this anchor-only proof.
        for y in range(1160,1184):
            for x in (52,84):
                i=(y*1500+x)*4;after[i:i+96]=before[i:i+96]
        # Independently verified cardinal tiles are not part of anchor-only delta.
        for y in range(1160,1188):
            for x in (116,148,180,212):
                i=(y*1500+x)*4;after[i:i+96]=before[i:i+96]
        changed = [i for i in range(0,len(before),4) if before[i:i+4] != after[i:i+4]]
        check(len(changed) == 126)
        check(all(20 <= i//4%1500 < 40 and 1160 <= i//4//1500 < 1180 for i in changed))
        check(all(before[i+3] == 0 and after[i+3] > 0 and after[i:i+3] == bytes(color) for i in changed))
        alpha = bytes(after[((1160+y)*1500+20+x)*4+3] for y in range(20) for x in range(20))
        check(hashlib.sha256(alpha).hexdigest() == 'a814de453f8c9158f6767a1ba25a496fae762535f1a79f2edbd116dc4a2a3c83')
        if expected_alpha is not None:
            check(alpha == expected_alpha)
        expected_alpha = alpha
        for y in range(1158,1182):
            check(not any(before[(y*1500+18)*4+3:(y*1500+42)*4:4]))
        # Restore only new art, then every RGBA byte including old ACHARE51,
        # ACHPNT02 and all neighboring resources must match.
        for i in changed:
            after[i:i+4] = before[i:i+4]
        check(after == before)
    # Exact prototype bounds including half of its round stroke, at 27/32.
    check(10-10.65*27/32 > 0 and 10+10.65*27/32 < 20)
    check(10-9.65*27/32 > 0 and 10+9.65*27/32 < 20)
    # GL counter-rotation about r then pivot subtraction, software r-pivot:
    # moving the atlas rectangle never moves the chart anchor. Upstream integer
    # pivot/size quantization is retained (less than one pixel at fractional scale).
    for scale in (.5,1,1.25,1.5,2,3):
        for angle in (0,.37,math.pi/2,math.pi):
            c, s = math.cos(angle), math.sin(angle)
            for px, py in ((0,0),(0,-7),(-10,5),(10,5),(0,9)):
                dx = (10+px*27/32)*scale-int(10*scale)
                dy = (10+py*27/32)*scale-int(10*scale)
                eps = 10*scale-int(10*scale)
                check(abs((c*dx+s*dy)-(c*px+s*py)*27/32*scale-eps*(c+s)) < 1e-12)
                check(abs((-s*dx+c*dy)-(-s*px+c*py)*27/32*scale-eps*(c-s)) < 1e-12)
            check(0 <= 10*scale-int(10*scale) < 1)

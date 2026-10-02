"""SCRUM-244: one exact prototype glyph in proven unused pinned atlas space.

The checked-in coverage mask is rasterized from the unchanged SVG paths, not
redrawn geometry. Generation needs only stdlib; the review fixture separately
re-renders the SVG. Stock resources and the old tile remain byte-for-byte intact.
"""
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from chart_raster_ink import decode, encode

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'resources/chart-style/v1/anchorage'
TILE = (20, 1160, 20, 20)
PIVOT = (10, 10)
OLD_BITMAP = '<bitmap width="25" height="29">'


def effective_anchor(root):
    matches = root.findall("symbols/symbol[name='ACHARE51']")
    assert len(matches) == 2 and matches[-1].get('RCID') == '1105'
    return matches[-1]


def restore_bitmap_for_validation(before, after):
    stock = effective_anchor(before).find('bitmap')
    styled = effective_anchor(after).find('bitmap')
    assert stock.attrib == {'width': '25', 'height': '29'}
    assert stock.find('pivot').attrib == {'x': '18', 'y': '1'}
    assert stock.find('graphics-location').attrib == {'x': '373', 'y': '415'}
    assert styled.attrib == {'width': '20', 'height': '20'}
    assert styled.find('pivot').attrib == {'x': '10', 'y': '10'}
    assert styled.find('graphics-location').attrib == {'x': '20', 'y': '1160'}
    # Restore only the six authorized numeric attributes. Every other property,
    # child and symbol is still checked by the caller's complete-tree equality.
    styled.attrib = stock.attrib.copy()
    for name in ('pivot', 'graphics-location'):
        styled.find(name).attrib = stock.find(name).attrib.copy()


def relocate(xml):
    tree = ET.fromstring(xml)
    effective_anchor(tree)
    # All bitmap declarations count, including overwritten symbols and patterns.
    # A two-pixel transparent moat protects sampling around the new tile.
    x, y, w, h = TILE
    for bitmap in tree.findall('.//bitmap'):
        location = bitmap.find('graphics-location')
        if location is None:
            continue
        bx, by = int(location.get('x')), int(location.get('y'))
        bw, bh = int(bitmap.get('width')), int(bitmap.get('height'))
        assert not (x-2 < bx+bw and x+w+2 > bx and y-2 < by+bh and y+h+2 > by), 'Anchor tile overlaps a declared resource'
    pattern = r'(<symbol RCID="1105">\s*<name>ACHARE51</name>)(.*?)(</symbol>)'
    matches = list(re.finditer(pattern, xml, re.S))
    assert len(matches) == 1
    body = matches[0][2]
    for old, new in ((OLD_BITMAP, '<bitmap width="20" height="20">'),
                     ('<pivot x="18" y="1" />', '<pivot x="10" y="10" />'),
                     ('<graphics-location x="373" y="415" />', '<graphics-location x="20" y="1160" />')):
        assert body.count(old) == 1
        body = body.replace(old, new)
    return re.sub(pattern, lambda m: m[1]+body+m[3], xml, flags=re.S)


def coverage():
    provenance = json.loads((ASSETS / 'provenance.json').read_text())
    for path, digest in provenance['sources'].items():
        # Git autocrlf is the only permitted source checkout normalization.
        content = (ROOT / path).read_bytes().replace(b'\r\n', b'\n')
        assert hashlib.sha256(content).hexdigest() == digest, 'Anchor artwork provenance changed: '+path
    data = json.loads((ASSETS / 'ACHARE51-alpha.json').read_text())
    assert (data['width'], data['height']) == TILE[2:]
    assert len(data['rows']) == 20 and all(len(row) == 40 for row in data['rows'])
    alpha = bytes.fromhex(''.join(data['rows']))
    assert hashlib.sha256(alpha).hexdigest() == provenance['alphaSha256']
    assert all(alpha[n] == alpha[380+n] == alpha[n*20] == alpha[n*20+19] == 0 for n in range(20))
    return alpha, provenance


def paint(content, table):
    alpha, provenance = coverage()
    rgb = bytes(provenance['serviceRgb'][table])
    assert len(rgb) == 3
    chunks, before = decode(content)
    after = bytearray(before)
    x, y, w, h = TILE
    for row in range(y-2, y+h+2):
        assert not any(before[(row*1500+x-2)*4+3:(row*1500+x+w+2)*4:4]), 'Anchor tile or moat is not transparent'
    changed = 0
    for dy in range(h):
        for dx in range(w):
            a = alpha[dy*w+dx]
            if not a:
                continue  # Preserve even invisible source RGB outside the art.
            i = ((y+dy)*1500+x+dx)*4
            after[i:i+4] = rgb + bytes([a])
            changed += 1
    assert changed == 126
    return encode(chunks, after), {'tile': TILE, 'pivot': PIVOT,
        'serviceRgb': list(rgb), 'changedPixels': changed,
        'alphaSha256': provenance['alphaSha256'],
        'atlasDimensionsUnchanged': [1500, 1200]}

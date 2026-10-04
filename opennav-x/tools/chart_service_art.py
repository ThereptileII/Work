"""SCRUM-254/279: exact pilot/radar/marina service art in isolated atlas tiles.

The source-locked coverage is derived from the immutable prototype SVG paths.
Only effective raster metadata and new artwork pixels change; generation uses
stdlib only. No S-52 selector, semantic color role or renderer is replaced.
"""
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from chart_raster_ink import decode, encode

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT/'resources/chart-style/v1/services'
# name: RCID, definition count, original rectangle/pivot, new rectangle
SYMBOLS = {
    'PILBOP02': (1, 2, (736, 778, 17, 17), (8, 8), (52, 1160, 24, 24)),
    'RTPBCN02': (2259, 1, (816, 239, 20, 19), (10, 9), (84, 1160, 24, 24)),
    'SMCFAC02': (2108, 1, (940, 239, 21, 21), (10, 10), (820, 1160, 24, 24)),
}
PIVOT = (12, 12)


def effective(root, name):
    nodes = root.findall("symbols/symbol[name='"+name+"']")
    rcid, count, *_ = SYMBOLS[name]
    assert len(nodes) == count and nodes[-1].get('RCID') == str(rcid)
    node = nodes[-1]
    assert node.find('bitmap') is not None and node.findtext('prefer-bitmap') not in ('no', 'false')
    return node


def bitmap_attributes(bitmap, tile, pivot):
    x, y, w, h = tile
    assert bitmap.attrib == {'width':str(w), 'height':str(h)}
    assert bitmap.find('pivot').attrib == dict(zip(('x','y'),map(str,pivot)))
    assert bitmap.find('graphics-location').attrib == {'x':str(x),'y':str(y)}


def restore_bitmap_for_validation(before, after):
    for name, (_, _, original, pivot, tile) in SYMBOLS.items():
        stock, styled = effective(before,name).find('bitmap'), effective(after,name).find('bitmap')
        bitmap_attributes(stock,original,pivot)
        bitmap_attributes(styled,tile,PIVOT)
        # Restore exactly the six approved numeric attributes; the caller then
        # requires full XML tree equality, including all other bitmap children.
        styled.attrib = stock.attrib.copy()
        for tag in ('pivot','graphics-location'):
            styled.find(tag).attrib = stock.find(tag).attrib.copy()


def relocate(xml):
    tree = ET.fromstring(xml)
    occupied = []
    # Includes overwritten definitions, patterns and the already-relocated
    # ACHARE51 tile. The new tiles also reserve their moats against each other.
    for bitmap in tree.findall('.//bitmap'):
        pos = bitmap.find('graphics-location')
        if pos is not None:
            occupied.append(tuple(map(int,(pos.get('x'),pos.get('y'),bitmap.get('width'),bitmap.get('height')))))
    for name, (rcid, _, original, pivot, tile) in SYMBOLS.items():
        bitmap_attributes(effective(tree,name).find('bitmap'),original,pivot)
        x,y,w,h = tile
        assert 2 <= x and 2 <= y and x+w+2 <= 1500 and y+h+2 <= 1200
        for bx,by,bw,bh in occupied:
            assert not (x-2 < bx+bw and x+w+2 > bx and y-2 < by+bh and y+h+2 > by), 'Service tile overlaps a declared resource'
        occupied.append((x-2,y-2,w+4,h+4))
        pattern = r'(<symbol RCID="'+str(rcid)+r'">\s*<name>'+name+r'</name>)(.*?)(</symbol>)'
        matches = list(re.finditer(pattern,xml,re.S)); assert len(matches) == 1
        body = matches[0][2]
        ox,oy,ow,oh = original
        for old,new in ((f'<bitmap width="{ow}" height="{oh}">',f'<bitmap width="{w}" height="{h}">'),
                        (f'<pivot x="{pivot[0]}" y="{pivot[1]}" />','<pivot x="12" y="12" />'),
                        (f'<graphics-location x="{ox}" y="{oy}" />',f'<graphics-location x="{x}" y="{y}" />')):
            assert body.count(old) == 1
            body = body.replace(old,new)
        xml = re.sub(pattern,lambda m:m[1]+body+m[3],xml,flags=re.S)
    return xml


def coverage():
    provenance = json.loads((ASSETS/'provenance.json').read_text())
    assert set(provenance['symbols']) == set(SYMBOLS)
    for path,digest in provenance['sources'].items():
        content = (ROOT/path).read_bytes().replace(b'\r\n',b'\n')
        assert hashlib.sha256(content).hexdigest() == digest, 'Service artwork provenance changed: '+path
    masks = {}
    for name,(rcid,_,_,_,tile) in SYMBOLS.items():
        item = provenance['symbols'][name]
        assert item['tile'] == list(tile) and item['pivot'] == list(PIVOT) and item['effectiveRcid'] == rcid
        data = json.loads((ASSETS/(name+'-alpha.json')).read_text())
        assert (data['width'],data['height']) == (24,24)
        assert len(data['rows']) == 24 and all(len(row) == 48 for row in data['rows'])
        alpha = bytes.fromhex(''.join(data['rows']))
        assert hashlib.sha256(alpha).hexdigest() == item['alphaSha256']
        assert sum(bool(a) for a in alpha) == item['changedPixels']
        assert all(alpha[n] == alpha[552+n] == alpha[n*24] == alpha[n*24+23] == 0 for n in range(24))
        masks[name] = alpha
    return masks, provenance


def paint(content, table):
    masks, provenance = coverage()
    rgb = bytes(provenance['serviceRgb'][table]); assert len(rgb) == 3
    chunks, before = decode(content)
    after = bytearray(before)
    evidence = {}
    for name,(_,_,_,_,tile) in SYMBOLS.items():
        x,y,w,h = tile
        for row in range(y-2,y+h+2):
            assert not any(before[(row*1500+x-2)*4+3:(row*1500+x+w+2)*4:4]), 'Service tile or moat is not transparent'
        for dy in range(h):
            for dx in range(w):
                a = masks[name][dy*w+dx]
                if not a: continue  # Preserve even invisible original RGB.
                i = ((y+dy)*1500+x+dx)*4
                after[i:i+4] = rgb+bytes([a])
        evidence[name] = {**provenance['symbols'][name], 'serviceRgb':list(rgb),
                          'atlasDimensionsUnchanged':[1500,1200]}
    return encode(chunks,after), evidence

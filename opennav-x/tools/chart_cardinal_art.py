"""SCRUM-256: exact classified Simplified cardinal art in isolated atlas tiles.

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
ASSETS = ROOT/'resources/chart-style/v1/cardinals'
# name: RCID, definition count, original rectangle/pivot, new rectangle
SYMBOLS = {
    'BOYCAR01': (1270, 1, (984, 10, 13, 19), (8, 9), (116, 1160, 24, 28)),
    'BOYCAR02': (1271, 1, (1007, 10, 12, 20), (5, 10), (148, 1160, 24, 28)),
    'BOYCAR03': (1272, 1, (1029, 10, 13, 19), (3, 9), (180, 1160, 24, 28)),
    'BOYCAR04': (1273, 1, (1052, 10, 19, 19), (9, 9), (212, 1160, 24, 28)),
}
PIVOT = (12, 14)


def require_selectors(tree):
    for category,name in enumerate(SYMBOLS,1):
        lookups = [n for n in tree.findall('lookups/lookup')
                   if 'SY('+name+')' in (n.findtext('instruction') or '')]
        assert len(lookups) == 1
        node = lookups[0]
        assert node.attrib == {'id':str(1021+category),'RCID':str(31073+category),'name':'BOYCAR'}
        assert node.findtext('type') == 'Point' and node.findtext('table-name') == 'Simplified'
        assert [a.text for a in node.findall('attrib-code')] == ['CATCAM'+str(category)]
        assert node.findtext('instruction') == "SY("+name+");TE('%s','OBJNAM',2,1,2,'15110',-1,-1,CHBLK,21)"
    defaults = tree.findall("lookups/lookup[@id='1262']")
    assert len(defaults) == 1 and defaults[0].get('RCID') == '31314'
    assert defaults[0].get('name') == 'TOPMAR' and defaults[0].findtext('table-name') == 'Simplified'
    assert not defaults[0].findall('attrib-code') and not defaults[0].findtext('instruction')



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
    require_selectors(tree)
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
            assert not (x-2 < bx+bw and x+w+2 > bx and y-2 < by+bh and y+h+2 > by), 'Cardinal tile overlaps a declared resource'
        occupied.append((x-2,y-2,w+4,h+4))
        pattern = r'(<symbol RCID="'+str(rcid)+r'">\s*<name>'+name+r'</name>)(.*?)(</symbol>)'
        matches = list(re.finditer(pattern,xml,re.S)); assert len(matches) == 1
        body = matches[0][2]
        ox,oy,ow,oh = original
        for old,new in ((f'<bitmap width="{ow}" height="{oh}">',f'<bitmap width="{w}" height="{h}">'),
                        (f'<pivot x="{pivot[0]}" y="{pivot[1]}" />','<pivot x="12" y="14" />'),
                        (f'<graphics-location x="{ox}" y="{oy}" />',f'<graphics-location x="{x}" y="{y}" />')):
            assert body.count(old) == 1
            body = body.replace(old,new)
        xml = re.sub(pattern,lambda m:m[1]+body+m[3],xml,flags=re.S)
    return xml


def coverage(table):
    provenance = json.loads((ASSETS/'provenance.json').read_text())
    assert set(provenance['symbols']) == set(SYMBOLS)
    assert provenance['prototypeScale'] == 27/32 and provenance['strokeWidth'] == 1.3
    assert provenance['nightBrightness'] == .78
    for path,digest in provenance['sources'].items():
        content = (ROOT/path).read_bytes().replace(b'\r\n',b'\n')
        assert hashlib.sha256(content).hexdigest() == digest, 'Cardinal artwork provenance changed: '+path
    tiles = {}
    for category,(name,(rcid,_,_,_,tile)) in enumerate(SYMBOLS.items(),1):
        item = provenance['symbols'][name]
        assert item['category'] == category and item['tile'] == list(tile)
        assert item['pivot'] == list(PIVOT) and item['effectiveRcid'] == rcid
        data = json.loads((ASSETS/(name+'-'+table+'-rgba.json')).read_text())
        assert (data['width'],data['height']) == (24,28)
        assert len(data['rows']) == 28 and all(len(row) == 192 for row in data['rows'])
        rgba = bytes.fromhex(''.join(data['rows']))
        assert hashlib.sha256(rgba).hexdigest() == item['themes'][table]['rgbaSha256']
        alpha = rgba[3::4]
        assert sum(bool(a) for a in alpha) == item['themes'][table]['changedPixels']
        assert not any(alpha[:24]+alpha[-24:]+alpha[::24]+alpha[23::24])
        tiles[name] = rgba
    return tiles, provenance


def paint(content, table):
    tiles, provenance = coverage(table)
    chunks, before = decode(content)
    after = bytearray(before)
    evidence = {}
    for name,(_,_,_,_,tile) in SYMBOLS.items():
        x,y,w,h = tile
        for row in range(y-2,y+h+2):
            assert not any(before[(row*1500+x-2)*4+3:(row*1500+x+w+2)*4:4]), 'Cardinal tile or moat is not transparent'
        for dy in range(h):
            for dx in range(w):
                pixel = tiles[name][(dy*w+dx)*4:(dy*w+dx+1)*4]
                if not pixel[3]: continue  # Preserve even invisible original RGB.
                i = ((y+dy)*1500+x+dx)*4
                after[i:i+4] = pixel
        evidence[name] = {**provenance['symbols'][name], 'colors':provenance['colors'][table],
                          'atlasDimensionsUnchanged':[1500,1200]}
    return encode(chunks,after), evidence

"""SCRUM-264: source-locked Simplified marine aliases and isolated light art.

All new names are eight-byte S-52 keys. Original lookup order and source nodes
remain intact; only the sixteen proven marine lateral instructions are redirected.
Unclassified/special-purpose, inland and Paper Chart selectors remain stock.
"""
import copy
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET
from chart_raster_ink import decode, encode
from chart_cardinal_art import bitmap_attributes

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT/'resources/chart-style/v1/seamarks'
MAPPING_SHA256 = 'ce6d9e8cc9b18532e3235f981bc894d6dd367b67e23530adeab9305b423f226b'
_mapping_bytes = (ASSETS/'mapping.json').read_bytes().replace(b'\r\n', b'\n')
assert hashlib.sha256(_mapping_bytes).hexdigest() == MAPPING_SHA256
_mapping = json.loads(_mapping_bytes)
SYMBOLS, SELECTORS = _mapping['symbols'], _mapping['selectors']
PIVOT = (12, 14)


def source_node(tree, spec):
    nodes = tree.findall("symbols/symbol[name='"+spec['source']+"']")
    assert len(nodes) == 1 and nodes[0].get('RCID') == str(spec['sourceRcid'])
    node = nodes[0]
    assert hashlib.sha256(ET.tostring(node)).hexdigest() == spec['sourceNodeSha256'], 'Seamark source node changed'
    if spec['source'] == 'LIGHTS13':
        assert node.findtext('prefer-bitmap') == 'no'
    else:
        assert node.findtext('prefer-bitmap') not in ('no', 'false')
    bitmap_attributes(node.find('bitmap'), spec['original'], spec['originalPivot'])
    return node


def instruction(lookup):
    text = lookup.findtext('instruction') or ''
    spec = SELECTORS.get(lookup.get('id'))
    if spec is None:
        return text
    assert lookup.attrib == {'id':lookup.get('id'), 'RCID':spec['rcid'], 'name':'BOYLAT'}
    assert lookup.findtext('type') == 'Point' and lookup.findtext('table-name') == 'Simplified'
    assert [n.text for n in lookup.findall('attrib-code')] == spec['attributes']
    assert text == spec['instruction']
    # Preferred selectors originally reuse the ordinary can/cone glyphs.
    assert text.startswith('SY(BOYLAT')
    return re.sub(r'^SY\(BOYLAT\d{2}\)', 'SY('+spec['alias']+')', text, count=1)


def styled_node(stock, name, spec):
    node = copy.deepcopy(stock)
    node.set('RCID',str(spec['rcid']))
    node.find('name').text = name
    if name == 'LIGHTS13': node.find('prefer-bitmap').text = 'yes'
    x,y,w,h = spec['tile']
    bitmap = node.find('bitmap')
    bitmap.attrib = {'width':str(w),'height':str(h)}
    bitmap.find('pivot').attrib = {'x':'12','y':'14'}
    bitmap.find('graphics-location').attrib = {'x':str(x),'y':str(y)}
    return node


def canonical(node):
    node = copy.deepcopy(node)
    for child in node.iter():
        if not (child.text or '').strip(): child.text = None
        if not (child.tail or '').strip(): child.tail = None
    return ET.tostring(node)


def restore_for_validation(before, after):
    for name,spec in SYMBOLS.items():
        stock = source_node(before,spec)
        nodes = after.findall("symbols/symbol[name='"+name+"']")
        assert len(nodes) == 1
        assert canonical(nodes[0]) == canonical(styled_node(stock,name,spec)), 'Seamark node changed beyond approved alias/bitmap'
        if name != spec['source']:
            after.find('symbols').remove(nodes[0])
        else:
            if name == 'LIGHTS13': nodes[0].find('prefer-bitmap').text = 'no'
            bitmap = nodes[0].find('bitmap'); original = stock.find('bitmap')
            bitmap.attrib = original.attrib.copy()
            for tag in ('pivot','graphics-location'):
                bitmap.find(tag).attrib = original.find(tag).attrib.copy()


def relocate(xml):
    tree = ET.fromstring(xml)
    for lookup in tree.findall('lookups/lookup'):
        replacement = instruction(lookup)
        if lookup.get('id') in SELECTORS:
            old = lookup.findtext('instruction')
            pattern = r'(<lookup id="'+lookup.get('id')+r'"[^>]*>)(.*?)(</lookup>)'
            matches = list(re.finditer(pattern,xml,re.S)); assert len(matches) == 1
            assert matches[0][2].count(old) == 1
            xml = re.sub(pattern,lambda m:m[1]+m[2].replace(old,replacement)+m[3],xml,flags=re.S)
    assert set(SELECTORS) <= {n.get('id') for n in tree.findall('lookups/lookup')}
    rcids = {n.get('RCID') for n in tree.iter() if n.get('RCID')}
    names = {n.text for n in tree.findall('.//name')}
    occupied = []
    for bitmap in tree.findall('.//bitmap'):
        pos = bitmap.find('graphics-location')
        if pos is not None:
            occupied.append(tuple(map(int,(pos.get('x'),pos.get('y'),bitmap.get('width'),bitmap.get('height')))))
    aliases = []
    for name,spec in SYMBOLS.items():
        stock = source_node(tree,spec)
        assert len(name) == 8 and name.isascii()
        if name != spec['source']:
            assert name not in names and str(spec['rcid']) not in rcids, 'Owned seamark name/RCID collision'
            names.add(name); rcids.add(str(spec['rcid']))
        x,y,w,h = spec['tile']
        assert 2 <= x and 2 <= y and x+w+2 <= 1500 and y+h+2 <= 1200
        for bx,by,bw,bh in occupied:
            assert not (x-2 < bx+bw and x+w+2 > bx and y-2 < by+bh and y+h+2 > by), 'Seamark tile overlaps a declared resource'
        occupied.append((x-2,y-2,w+4,h+4))
        replacement = ET.tostring(styled_node(stock,name,spec),encoding='unicode')
        if name != spec['source']:
            aliases.append(replacement)
        else:
            pattern = r'<symbol RCID="'+str(spec['sourceRcid'])+r'">\s*<name>'+name+r'</name>.*?</symbol>'
            assert len(re.findall(pattern,xml,re.S)) == 1
            xml = re.sub(pattern,lambda _:replacement.rstrip(),xml,flags=re.S)
    assert xml.count('</symbols>') == 1
    return xml.replace('</symbols>',''.join(aliases)+'</symbols>')


def coverage(table):
    provenance = json.loads((ASSETS/'provenance.json').read_text())
    assert set(provenance['symbols']) == set(SYMBOLS)
    assert provenance['nightBrightness'] == .78
    for path,digest in provenance['sources'].items():
        content = (ROOT/path).read_bytes().replace(b'\r\n',b'\n')
        assert hashlib.sha256(content).hexdigest() == digest, 'Seamark artwork provenance changed: '+path
    tiles = {}
    for name,spec in SYMBOLS.items():
        item = provenance['symbols'][name]
        assert item['tile'] == spec['tile'] and item['pivot'] == list(PIVOT)
        assert item['prototypeScale'] == (25/32 if name == 'LIGHTS13' else 27/32)
        data = json.loads((ASSETS/(name+'-'+table+'-rgba.json')).read_text())
        assert (data['width'],data['height']) == (24,28)
        assert len(data['rows']) == 28 and all(len(row) == 192 for row in data['rows'])
        rgba = bytes.fromhex(''.join(data['rows']))
        assert hashlib.sha256(rgba).hexdigest() == item['themes'][table]['rgbaSha256']
        alpha = rgba[3::4]
        assert sum(bool(a) for a in alpha) == item['themes'][table]['changedPixels']
        assert not any(alpha[:24]+alpha[-24:]+alpha[::24]+alpha[23::24])
        tiles[name] = rgba
    return tiles,provenance


def paint(content, table):
    tiles, provenance = coverage(table)
    chunks,before = decode(content)
    after = bytearray(before)
    for name,spec in SYMBOLS.items():
        x,y,w,h = spec['tile']
        for row in range(y-2,y+h+2):
            assert not any(before[(row*1500+x-2)*4+3:(row*1500+x+w+2)*4:4]), 'Seamark tile or moat is not transparent'
        for dy in range(h):
            for dx in range(w):
                pixel = tiles[name][(dy*w+dx)*4:(dy*w+dx+1)*4]
                if not pixel[3]: continue
                i = ((y+dy)*1500+x+dx)*4
                after[i:i+4] = pixel
    return encode(chunks,after), provenance['symbols']

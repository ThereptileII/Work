"""Prototype generic beacon for the two existing Simplified generic selections.

Classified marine and Paper bodies, including their composed topmarks, stay
unchanged. The original two BCNGEN01 definitions are never replaced.
"""
import hashlib
import json
import re
from pathlib import Path
import xml.etree.ElementTree as ET

from chart_raster_ink import decode, encode
from chart_seamark_art import canonical, styled_node

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'resources/chart-style/v1/generic-beacon'
NAME = 'XNBCNG01'
SPEC = {'source': 'BCNGEN01', 'sourceRcid': 11, 'rcid': 60014,
        'tile': [724, 1160, 24, 28]}
SOURCE_NODES = (
    ('1238', 'bd9eb94d0fc9ef2f54187aa4127aa73522e98c2fea8237afbcf0612df745fe0a'),
    ('11', 'a880d8aba3d7d46bf6103020dacb7348485f13749522ef97ebf3ff6d6d8036e0'))
SELECTORS = {
    '1696': 'd019ee48edd8b45d031a4e80a28d78dd9822d23f6a7c2cbbdb7ff7bdda241fd6',
    '1708': '9969d069bb48ed0f7bc7c570ec72ed63f739b993f7206ebcd6627a446758d2e3'}


def source_node(tree):
    nodes = tree.findall("symbols/symbol[name='BCNGEN01']")
    assert len(nodes) == len(SOURCE_NODES), 'Generic beacon definition count changed'
    for node, (rcid, digest) in zip(nodes, SOURCE_NODES):
        assert node.get('RCID') == rcid
        assert hashlib.sha256(ET.tostring(node)).hexdigest() == digest, 'Generic beacon source node changed'
    return nodes[-1]


def instruction(lookup):
    original = lookup.findtext('instruction') or ''
    digest = SELECTORS.get(lookup.get('id'))
    if digest is None:
        return original
    assert hashlib.sha256(ET.tostring(lookup)).hexdigest() == digest, 'Generic beacon lookup changed'
    assert lookup.findtext('table-name') == 'Simplified'
    assert original.startswith('SY(BCNGEN01);')
    return original.replace('SY(BCNGEN01)', 'SY(' + NAME + ')', 1)


def relocate(xml):
    tree = ET.fromstring(xml)
    stock = source_node(tree)
    assert not tree.findall("symbols/symbol[name='" + NAME + "']")
    assert not any(node.get('RCID') == str(SPEC['rcid']) for node in tree.iter())
    x, y, w, h = SPEC['tile']
    for bitmap in tree.findall('.//bitmap'):
        pos = bitmap.find('graphics-location')
        if pos is None:
            continue
        bx, by = int(pos.get('x')), int(pos.get('y'))
        bw, bh = int(bitmap.get('width')), int(bitmap.get('height'))
        assert not (x-2 < bx+bw and x+w+2 > bx and y-2 < by+bh and y+h+2 > by), 'Generic beacon tile overlaps declared artwork'
    for key in SELECTORS:
        nodes = tree.findall("lookups/lookup[@id='" + key + "']")
        assert len(nodes) == 1
        replacement = instruction(nodes[0])
        original = nodes[0].findtext('instruction')
        pattern = r'(<lookup id="' + key + r'"[^>]*>)(.*?)(</lookup>)'
        matches = list(re.finditer(pattern, xml, re.S))
        assert len(matches) == 1 and matches[0][2].count(original) == 1
        xml = re.sub(pattern, lambda m: m[1] + m[2].replace(original, replacement) + m[3], xml, flags=re.S)
    assert xml.count('</symbols>') == 1
    return xml.replace('</symbols>', ET.tostring(styled_node(stock, NAME, SPEC), encoding='unicode') + '</symbols>')


def restore_for_validation(before, after):
    stock = source_node(before)
    nodes = after.findall("symbols/symbol[name='" + NAME + "']")
    assert len(nodes) == 1
    assert canonical(nodes[0]) == canonical(styled_node(stock, NAME, SPEC)), 'Generic beacon alias changed beyond approved bitmap'
    after.find('symbols').remove(nodes[0])


def coverage(table):
    proof = json.loads((ASSETS / 'provenance.json').read_text())
    assert proof['tile'] == SPEC['tile'] and proof['pivot'] == [12, 14]
    assert proof['prototypeScale'] == 27/32 and proof['nightBrightness'] == .78
    for name, digest in proof['sources'].items():
        assert hashlib.sha256((ROOT/name).read_bytes().replace(b'\r\n', b'\n')).hexdigest() == digest, 'Generic beacon artwork provenance changed'
    data = json.loads((ASSETS / (NAME + '-' + table + '-rgba.json')).read_text())
    assert data['width'] == 24 and data['height'] == 28
    assert len(data['rows']) == 28 and all(len(row) == 192 for row in data['rows'])
    rgba = bytes.fromhex(''.join(data['rows']))
    assert hashlib.sha256(rgba).hexdigest() == proof['themes'][table]['rgbaSha256']
    alpha = rgba[3::4]
    assert sum(bool(a) for a in alpha) == proof['themes'][table]['changedPixels']
    assert not any(alpha[:24] + alpha[-24:] + alpha[::24] + alpha[23::24])
    return rgba, proof


def paint(content, table):
    rgba, proof = coverage(table)
    chunks, before = decode(content)
    after = bytearray(before)
    x, y, w, h = SPEC['tile']
    for row in range(y-2, y+h+2):
        assert not any(before[(row*1500+x-2)*4+3:(row*1500+x+w+2)*4:4]), 'Generic beacon tile or moat is not transparent'
    for dy in range(h):
        for dx in range(w):
            pixel = rgba[(dy*w+dx)*4:(dy*w+dx+1)*4]
            if pixel[3]:
                offset = ((y+dy)*1500+x+dx)*4
                after[offset:offset+4] = pixel
    return encode(chunks, after), proof

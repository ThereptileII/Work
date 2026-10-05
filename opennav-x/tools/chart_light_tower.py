"""Owned light-support tower ink; exact pinned shape and classifications retained."""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from chart_raster_ink import decode, encode
from chart_seamark_art import canonical
ROOT = Path(__file__).resolve().parents[1]
RECIPE = ROOT/'resources/chart-style/v1/light-tower/recipe.json'
RECIPE_SHA256 = '4c98e642f74df3af80d89f456267694fb59a085169e5d7a7701a9f089ca60a5b'

def recipe():
    raw = RECIPE.read_bytes().replace(b'\r\n', b'\n')
    assert hashlib.sha256(raw).hexdigest() == RECIPE_SHA256, 'Tower recipe changed'
    return json.loads(raw)['symbols']

def source_node(tree, spec):
    nodes = tree.findall("symbols/symbol[name='"+spec['source']+"']")
    assert len(nodes) == spec['sourceDefinitionCount']
    node = nodes[-1]  # The loader deliberately replaces the earlier vector.
    assert hashlib.sha256(canonical(node)).hexdigest() == spec['sourceNodeCanonicalSha256'], 'Effective tower changed'
    lup = tree.find("lookups/lookup[@id='"+spec['lookupId']+"']")
    assert hashlib.sha256(canonical(lup)).hexdigest() == spec['lookupCanonicalSha256'], 'Tower selector changed'
    return node

def alias(stock, name, spec):
    node = copy.deepcopy(stock)
    node.set('RCID', str(spec['rcid']))
    node.find('name').text = name
    node.find('bitmap/graphics-location').attrib = dict(zip(('x','y'),map(str,spec['tile'][:2])))
    node.find('color-ref').text = 'A'+spec['fillRole']+'B'+spec['outlineRole']
    return node

def relocate(xml):
    tree = ET.fromstring(xml)
    for name,spec in recipe().items():
        stock = source_node(tree,spec)
        assert not tree.findall("symbols/symbol[name='"+name+"']")
        assert not any(n.get('RCID') == str(spec['rcid']) for n in tree.iter())
        x,y,w,h = spec['tile']
        assert 2 <= x and 2 <= y and x+w+2 <= 1500 and y+h+2 <= 1200
        for bitmap in tree.findall('.//bitmap'):
            pos = bitmap.find('graphics-location')
            if pos is None: continue
            bx,by = int(pos.get('x')),int(pos.get('y'))
            bw,bh = int(bitmap.get('width')),int(bitmap.get('height'))
            assert not(x-2 < bx+bw and x+w+2 > bx and y-2 < by+bh and y+h+2 > by), 'Tower alias overlaps artwork'
        node = alias(stock,name,spec)
        tree.find('symbols').append(node)
        assert xml.count('</symbols>') == 1
        xml = xml.replace('</symbols>',ET.tostring(node,encoding='unicode')+'</symbols>')
    return xml

def restore_for_validation(before,after):
    for name,spec in recipe().items():
        nodes = after.findall("symbols/symbol[name='"+name+"']")
        assert len(nodes)==1 and canonical(nodes[0])==canonical(alias(source_node(before,spec),name,spec)), 'Tower alias changed'
        after.find('symbols').remove(nodes[0])

def tile(pixels, rect):
    x,y,w,h=rect
    return b''.join(pixels[((y+j)*1500+x)*4:((y+j)*1500+x+w)*4] for j in range(h))

def tile_rgba(source,table,spec,colors):
    assert hashlib.sha256(source).hexdigest()==spec['sourceTileRgbaSha256'][table], 'Tower source pixels changed'
    alpha=bytes.fromhex(''.join(spec['alphaRows']))
    assert source[3::4]==alpha
    fill,outline=colors[spec['fillRole']],colors[spec['outlineRole']]
    denominator=spec['denominator']; rgba=bytearray()
    for n,a in zip(sum(spec['fillNumerators'],[]),alpha):
        rgb=[(o*denominator+n*(f-o)+denominator//2)//denominator for f,o in zip(fill,outline)] if a else [0,0,0]
        assert all(0<=c<=255 for c in rgb)
        rgba.extend((*rgb,a))
    assert rgba[3::4]==source[3::4] and len(rgba)==len(source)
    return bytes(rgba)

def paint(content, original, table, colors):
    chunks,before=decode(content); _,pinned=decode(original)
    after=bytearray(before); proof={}
    for name,spec in recipe().items():
        rgba=tile_rgba(tile(pinned,spec['sourceTile']),table,spec,colors)
        x,y,w,h=spec['tile']
        for row in range(y-2,y+h+2):
            assert not any(before[(row*1500+x-2)*4+3:(row*1500+x+w+2)*4:4]), 'Tower tile moat occupied'
        for row in range(h):
            i=((y+row)*1500+x)*4; after[i:i+w*4]=rgba[row*w*4:(row+1)*w*4]
        proof[name]={'source':spec['source'],'tile':spec['tile'],'pivot':spec['pivot'],'outlineRole':spec['outlineRole'],'fillRole':spec['fillRole'],'rgbaSha256':hashlib.sha256(rgba).hexdigest(),'alphaSha256':hashlib.sha256(rgba[3::4]).hexdigest(),'recipeSha256':RECIPE_SHA256,'originalSymbolsAndLookupsUnchanged':True}
    return encode(chunks,after),proof

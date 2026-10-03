"""Source-locked yellow special-mark body and explicit fitted-X head."""
import hashlib,json
from pathlib import Path
import xml.etree.ElementTree as ET
from chart_raster_ink import decode,encode
from chart_seamark_art import canonical,source_node,styled_node
ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'resources/chart-style/v1/yellow-buoy'
BASE={'source':'BOYSPP11','sourceRcid':1297,'rcid':60013,'original':[37,78,13,13],
      'originalPivot':[6,6],'tile':[692,1160,24,28],
      'sourceNodeSha256':'ddbc4680324551fe4ba7f471ff26869f544ea37bc44c04af2f5a4a4325c439bb'}

SPECS={'XNSPPY01':BASE, 'XNSPPT01':dict(BASE,rcid=60015,tile=[756,1160,24,28])}

def relocate_one(xml, NAME, SPEC):
    tree=ET.fromstring(xml);stock=source_node(tree,SPEC)
    assert not tree.findall("symbols/symbol[name='"+NAME+"']")
    assert not any(n.get('RCID')==str(SPEC['rcid']) for n in tree.iter())
    x,y,w,h=SPEC['tile']
    for bitmap in tree.findall('.//bitmap'):
        pos=bitmap.find('graphics-location')
        if pos is None:continue
        bx,by=int(pos.get('x')),int(pos.get('y'));bw,bh=int(bitmap.get('width')),int(bitmap.get('height'))
        assert not(x-2<bx+bw and x+w+2>bx and y-2<by+bh and y+h+2>by),'Special buoy tile overlaps declared artwork'
    assert xml.count('</symbols>')==1
    return xml.replace('</symbols>',ET.tostring(styled_node(stock,NAME,SPEC),encoding='unicode')+'</symbols>')

def restore_one(before,after, NAME, SPEC):
    stock=source_node(before,SPEC);nodes=after.findall("symbols/symbol[name='"+NAME+"']")
    assert len(nodes)==1 and canonical(nodes[0])==canonical(styled_node(stock,NAME,SPEC))
    after.find('symbols').remove(nodes[0])

def coverage(table, NAME, SPEC):
    proof=json.loads((ASSETS/'provenance.json').read_text())
    item=proof['symbols'][NAME]
    assert item['tile']==SPEC['tile'] and item['rcid']==SPEC['rcid'] and proof['pivot']==[12,14] and proof['prototypeScale']==27/32
    assert proof['nightBrightness']==.78
    for name,digest in proof['sources'].items():
        assert hashlib.sha256((ROOT/name).read_bytes().replace(b'\r\n',b'\n')).hexdigest()==digest,'Special buoy artwork provenance changed'
    data=json.loads((ASSETS/(NAME+'-'+table+'-rgba.json')).read_text())
    assert data['width']==24 and data['height']==28 and len(data['rows'])==28 and all(len(row)==192 for row in data['rows'])
    rgba=bytes.fromhex(''.join(data['rows']));assert hashlib.sha256(rgba).hexdigest()==item['themes'][table]['rgbaSha256']
    alpha=rgba[3::4];assert sum(bool(a) for a in alpha)==item['themes'][table]['changedPixels']
    assert not any(alpha[:24]+alpha[-24:]+alpha[::24]+alpha[23::24])
    return rgba,proof

def paint_one(content,table,NAME,SPEC):
    rgba,proof=coverage(table,NAME,SPEC);chunks,before=decode(content);after=bytearray(before);x,y,w,h=SPEC['tile']
    for row in range(y-2,y+h+2):
        assert not any(before[(row*1500+x-2)*4+3:(row*1500+x+w+2)*4:4]),'Special buoy tile or moat is not transparent'
    for dy in range(h):
        for dx in range(w):
            pixel=rgba[(dy*w+dx)*4:(dy*w+dx+1)*4]
            if pixel[3]:
                i=((y+dy)*1500+x+dx)*4;after[i:i+4]=pixel
    return encode(chunks,after),proof

def relocate(xml):
    for name,spec in SPECS.items():xml=relocate_one(xml,name,spec)
    return xml

def restore_for_validation(before,after):
    for name,spec in SPECS.items():restore_one(before,after,name,spec)

def paint(content,table):
    for name,spec in SPECS.items():content,proof=paint_one(content,table,name,spec)
    return content,proof

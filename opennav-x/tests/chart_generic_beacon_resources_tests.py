"""Independent generic-beacon selection, immutable artwork and atlas guards."""
import copy
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from chart_raster_ink import decode

ASSETS = ROOT/'resources/chart-style/v1/generic-beacon'
TABLES = (('DAY_BRIGHT','rastersymbols-day.png','#7c858a'),
          ('DUSK','rastersymbols-dusk.png','#a8bbb7'),
          ('NIGHT','rastersymbols-dark.png','#62736c'))


def rgba(table):
    data = json.loads((ASSETS/('XNBCNG01-'+table+'-rgba.json')).read_text())
    assert data['width'] == 24 and data['height'] == 28
    return bytes.fromhex(''.join(data['rows']))


def restore_generic_beacon(before, after):
    # Validate all 672 pixels, including transparent RGB, before undoing ink.
    for table, _, _ in TABLES:
        pixels = rgba(table)
        valid = True
        for y in range(28):
            for x in range(24):
                i = ((1160+y)*1500+724+x)*4
                p = pixels[(y*24+x)*4:(y*24+x+1)*4]
                valid &= before[i+3] == 0 and after[i:i+4] == (p if p[3] else before[i:i+4])
        if valid:
            for y in range(28):
                for x in range(24):
                    if pixels[(y*24+x)*4+3]:
                        i = ((1160+y)*1500+724+x)*4
                        after[i:i+4] = before[i:i+4]
            return
    raise AssertionError('Generic beacon tile differs from approved complete coverage')


def canonical(tree):
    tree = copy.deepcopy(tree)
    for n in tree.iter():
        if n.text is not None and not n.text.strip(): n.text = None
        if n.tail is not None and not n.tail.strip(): n.tail = None
    return ET.tostring(tree)


def verify_generic_beacon(source, output, check):
    stock = ET.parse(source/'chartsymbols.xml').getroot()
    final = ET.parse(output/'chartsymbols.xml').getroot()
    original = stock.findall("symbols/symbol[name='BCNGEN01']")
    check([n.get('RCID') for n in original] == ['1238','11'])
    check([ET.tostring(n) for n in original] == [ET.tostring(n) for n in final.findall("symbols/symbol[name='BCNGEN01']")])
    nodes = final.findall("symbols/symbol[name='XNBCNG01']")
    check(len(nodes) == 1)
    alias = nodes[0]
    check(alias.get('RCID') == '60014')
    check(alias.find('bitmap').attrib == {'width':'24','height':'28'})
    check(alias.find('bitmap/pivot').attrib == {'x':'12','y':'14'})
    check(alias.find('bitmap/graphics-location').attrib == {'x':'724','y':'1160'})
    restored = copy.deepcopy(alias)
    restored.set('RCID','11'); restored.find('name').text = 'BCNGEN01'
    restored.remove(restored.find('bitmap'))
    restored.insert(list(original[-1]).index(original[-1].find('bitmap')), copy.deepcopy(original[-1].find('bitmap')))
    check(canonical(restored) == canonical(original[-1]))
    users = []
    for old, new in zip(stock.findall('lookups/lookup'), final.findall('lookups/lookup')):
        text = old.findtext('instruction') or ''
        if 'BCNGEN01' not in text: continue
        users.append(old.get('id'))
        if old.get('id') in ('1696','1708'):
            check(old.attrib == ({'id':'1696','RCID':'31748','name':'_bcngn'} if old.get('id')=='1696' else {'id':'1708','RCID':'31760','name':'_slgto'}))
            check(old.findtext('table-name') == 'Simplified' and not old.findall('attrib-code'))
            check(new.findtext('instruction') == text.replace('SY(BCNGEN01)','SY(XNBCNG01)',1))
            normalized = copy.deepcopy(new); normalized.find('instruction').text = text
            check(ET.tostring(normalized) == ET.tostring(old))
        else:
            check(old.findtext('table-name') == 'Paper')
            check(ET.tostring(old) == ET.tostring(new))
    check(len(users) == 21)
    for identity in range(1700,1708):
        # Earlier coloured/shaped _slgto selections must still win unchanged.
        check(ET.tostring(stock.find("lookups/lookup[@id='"+str(identity)+"']")) == ET.tostring(final.find("lookups/lookup[@id='"+str(identity)+"']")))
    for table, filename, color in TABLES:
        svg = ET.parse(ASSETS/('XNBCNG01-'+table+'.svg')).getroot()
        for n in svg.iter(): n.tag = n.tag.rsplit('}',1)[-1]
        group = svg.find('g')
        check(group.attrib == {'transform':'translate(12 14)'})
        mark = group.find('g')
        check(mark.get('transform') == 'scale(0.84375)' and mark.get('class') == 'chart-marker-art marker-service')
        check([n.get('d') for n in mark] == ['M-5 8H5M-3 7-2-5H2L3 7M-4-5H4L0-11Z','M-2 2H2'])
        check('.chart-marker-art.marker-service{stroke:'+color+'}' in svg.findtext('style'))
        check('stroke-width:1.3;stroke-linecap:round;stroke-linejoin:round' in svg.findtext('style'))
        pixels = rgba(table)
        check(len(pixels) == 24*28*4 and sum(bool(a) for a in pixels[3::4]) == 102)
        _, atlas = decode((output/filename).read_bytes())
        for y in range(28):
            for x in range(24):
                p = pixels[(y*24+x)*4:(y*24+x+1)*4]
                i = ((1160+y)*1500+724+x)*4
                if p[3]: check(atlas[i:i+4] == p)
                else: check(atlas[i+3] == 0)


if __name__ == '__main__':
    import argparse
    import chart_generic_beacon_art as implementation
    parser = argparse.ArgumentParser()
    for name in ('source','generated','baseline','report'): parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    count = 0
    def check(value):
        global count
        count += 1
        assert value, count
    verify_generic_beacon(args.source,args.generated,check)
    baseline = ET.parse(args.baseline/'chartsymbols.xml').getroot()
    final = ET.parse(args.generated/'chartsymbols.xml').getroot()
    def inverse(tree):
        tree = copy.deepcopy(tree)
        implementation.restore_for_validation(baseline,tree)
        for identity in ('1696','1708'):
            node = tree.find("lookups/lookup[@id='"+identity+"']/instruction")
            old = baseline.findtext("lookups/lookup[@id='"+identity+"']/instruction")
            assert node.text == old.replace('SY(BCNGEN01)','SY(XNBCNG01)',1)
            node.text = old
        assert canonical(tree) == canonical(baseline)
    inverse(final); check(True)
    for change in ('pivot','rcid','hpgl','lookup-type','lookup-instruction','duplicate','paper'):
        tree = copy.deepcopy(final); alias = tree.find("symbols/symbol[name='XNBCNG01']")
        if change=='pivot': alias.find('bitmap/pivot').set('x','13')
        elif change=='rcid': alias.set('RCID','60015')
        elif change=='hpgl': tree.find("symbols/symbol[name='BCNGEN01']/vector/HPGL").text += 'PU0,0;'
        elif change=='lookup-type': tree.find("lookups/lookup[@id='1696']/type").text = 'Area'
        elif change=='lookup-instruction': tree.find("lookups/lookup[@id='1708']/instruction").text = 'SY(XNBCNG01);'
        elif change=='duplicate': tree.find('symbols').append(copy.deepcopy(alias))
        else: tree.find("lookups/lookup[@id='1727']/instruction").text = 'SY(XNBCNG01);'
        try: inverse(tree)
        except AssertionError: check(True)
        else: raise AssertionError('Mutation escaped: '+change)
    for table, filename, _ in TABLES:
        _, before = decode((args.baseline/filename).read_bytes())
        _, after = decode((args.generated/filename).read_bytes())
        check(sum(before[i:i+4] != after[i:i+4] for i in range(0,len(before),4)) == 102)
        restored = bytearray(after); restore_generic_beacon(before,restored); check(restored == before)
        for x,y in ((724,1160),(736,1174)):
            damaged = bytearray(after); damaged[(y*1500+x)*4] ^= 1
            try: restore_generic_beacon(before,damaged)
            except AssertionError: check(True)
            else: raise AssertionError('Changed owned pixel accepted')
    check((args.baseline/'S52RAZDS.RLE').read_bytes() == (args.generated/'S52RAZDS.RLE').read_bytes())
    report = {'checks':count,'wholeXmlInverse':True,'wholeRgbaInverse':True,
        'negativeXmlCases':7,'negativePixelCases':6,'changedPixelsPerTheme':102,
        'baselineManifestSha256':hashlib.sha256((args.baseline/'manifest.json').read_bytes()).hexdigest(),
        'manifestSha256':hashlib.sha256((args.generated/'manifest.json').read_bytes()).hexdigest(),
        'nativeWindowsAcceptance':False,'boatAcceptance':False}
    args.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))

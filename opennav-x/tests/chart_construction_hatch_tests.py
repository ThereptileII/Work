"""Independent CROSSX01 ownership, alpha, whole-atlas inverse and rule guards."""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from chart_raster_ink import decode,encode
import chart_construction_hatch as hatch


def restore_hatch(before,after):
    # Validate every tile byte before restoring exactly its proven Day RGB.
    # This is used by independent atlas proofs, never as a broad exclusion.
    for y in range(1040,1056):
        for x in range(400,416):
            i=(y*1500+x)*4
            if before[i+3]:
                assert before[i:i+4]==bytes((177,145,57,255))
                assert after[i:i+4]==bytes((83,100,95,255))
                after[i:i+3]=before[i:i+3]
            else:assert after[i:i+4]==before[i:i+4]


def verify_contrast_and_copy(generated,check):
    metadata=json.loads((generated/'manifest.json').read_text())
    def luminance(rgb):
        c=[v/255 for v in rgb]
        return sum((v/12.92 if v<=.04045 else ((v+.055)/1.055)**2.4)*w for v,w in zip(c,(.2126,.7152,.0722)))
    def contrast(a,b):
        x,y=sorted((luminance(a),luminance(b)))
        return (y+.05)/(x+.05)
    colors=metadata['palette']['DAY_BRIGHT'];ratios={}
    for role in ('LANDA','DEPDW','DEPMD','DEPMS','DEPVS','DEPIT'):
        old=contrast((177,145,57),colors[role]);new=contrast((83,100,95),colors[role])
        check(new>old);ratios[role]={'stock':old,'neutral':new}
    for role,minimum in [('LANDA',3),('DEPDW',4),('DEPVS',2)]:check(ratios[role]['neutral']>=minimum)
    # Actual private preparer uses this same verifier before copying all five
    # common generated files; its manifest/header bind the later host package.
    from curl_package import verify_file
    for name,identity in metadata['files'].items():verify_file(generated/name,identity);check(True)
    with tempfile.TemporaryDirectory(prefix='construction-resource-negative-') as folder:
        bad=Path(folder)/'rastersymbols-day.png';raw=bytearray((generated/bad.name).read_bytes());raw[-1]^=1;bad.write_bytes(raw)
        try:verify_file(bad,metadata['files'][bad.name])
        except ValueError:check(True)
        else:raise AssertionError('Private resource copy accepted changed atlas')
    return ratios


def verify(source,generated,check):
    spec=importlib.util.spec_from_file_location('hatch_generator',ROOT/'tools/generate-xnav-chart-style.py')
    generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
    raw=(source/'chartsymbols.xml').read_bytes();styled=(generated/'chartsymbols.xml').read_bytes()
    before,after=ET.fromstring(raw),ET.fromstring(styled)
    metadata=json.loads((generated/'manifest.json').read_text());colors=metadata['palette']
    generator.validate_resource_changes(raw,styled,colors);check(True)
    verify_contrast_and_copy(generated,check)
    old=before.find("patterns/pattern[name='CROSSX01']");new=after.find("patterns/pattern[name='CROSSX01']")
    check(old.get('RCID')==new.get('RCID')=='3' and old.find('vector') is None and new.find('vector') is None)
    check(new.findtext('color-ref')=='AXNHAT')
    restored=copy.deepcopy(new);restored.find('color-ref').text='ACHBRN'
    check(hatch.canonical(restored)==hatch.canonical(old))
    # Independent complete consumer list: conditional stays on area/line/point
    # records; only the area branch actually requests the raster pattern.
    users=[n.get('id') for n in before.findall('lookups/lookup') if 'CS(SLCONS03)' in (n.findtext('instruction') or '')]
    check(users==['181','280','522','627','830','906','1253','1564','1565','2371','2797','2798'])
    for identity in users:
        q="lookups/lookup[@id='"+identity+"']";check(ET.tostring(before.find(q))==ET.tostring(after.find(q)))
    for table,name,target in [('DAY_BRIGHT','rastersymbols-day.png',(83,100,95)),('DUSK','rastersymbols-dusk.png',(225,229,216)),('NIGHT','rastersymbols-dark.png',(117,133,121))]:
        check(tuple(colors[table]['XNHAT'])==target)
        q="color-tables/color-table[@name='"+table+"']/color[@name='CHBRN']"
        check(ET.tostring(before.find(q))==ET.tostring(after.find(q)))
        rawpng=(source/name).read_bytes();chunks,pixels=decode(rawpng)
        painted,receipt=hatch.paint(rawpng,table,target);_,result=decode(painted)
        changed=[i for i in range(0,len(pixels),4) if pixels[i:i+4]!=result[i:i+4]]
        check(len(changed)==(192 if table=='DAY_BRIGHT' else 0))
        check(all(400<=i//4%1500<416 and 1040<=i//4//1500<1056 and pixels[i:i+4]==bytes((177,145,57,255)) and result[i:i+4]==bytes((83,100,95,255)) for i in changed))
        check(pixels[3::4]==result[3::4])
        restored=bytearray(result);restore_hatch(pixels,restored);check(restored==pixels)
        if table!='DAY_BRIGHT':check(painted==rawpng)  # no re-encoding or invented night ink
        _,actual=decode((generated/name).read_bytes())
        for y in range(1040,1056):
            i=(y*1500+400)*4;check(actual[i:i+64]==result[i:i+64])
        check(metadata['constructionHatch'][name]==receipt)
        # Changing even transparent source RGB, alpha or a neighboring owner is refused.
        mutated=bytearray(pixels);mutated[(1040*1500+400)*4]^=1
        try:hatch.paint(encode(chunks,mutated),table,target)
        except AssertionError:check(True)
        else:raise AssertionError('Accepted changed construction source pixels')
    def reject(change):
        tree=ET.fromstring(styled);change(tree)
        try:generator.validate_resource_changes(raw,ET.tostring(tree),colors)
        except AssertionError:check(True)
        else:raise AssertionError('Accepted hatch exception escape')
    base="patterns/pattern[name='CROSSX01']"
    for path,attr,value in [('/bitmap','width','17'),('/bitmap/pivot','x','9'),('/bitmap/graphics-location','y','1041')]:
        reject(lambda tree,p=path,a=attr,v=value:tree.find(base+p).set(a,v))
    for tag,value in [('spacing','S'),('filltype','S'),('color-ref','ACHBRN')]:
        reject(lambda tree,t=tag,v=value:setattr(tree.find(base+'/'+t),'text',v))
    reject(lambda tree:tree.find(base).append(ET.Element('vector')))
    reject(lambda tree:tree.find('patterns').append(copy.deepcopy(tree.find(base))))
    reject(lambda tree:setattr(tree.find("patterns/pattern[name='CROSSX02']/color-ref"),'text','AXNHAT'))
    reject(lambda tree:tree.find("color-tables/color-table/color[@name='CHBRN']").set('r','83'))
    reject(lambda tree:setattr(tree.find("lookups/lookup[@id='181']/instruction"),'text','AC(LANDA)'))
    for mutation in ('overlap','shape'):
        tree=ET.fromstring(raw)
        if mutation=='overlap':
            other=copy.deepcopy(tree.find(base));other.find('name').text='OTHER';tree.find('patterns').append(other)
        else:tree.find(base+'/bitmap').set('height','15')
        try:hatch.recolor(ET.tostring(tree,encoding='unicode'))
        except AssertionError:check(True)
        else:raise AssertionError('Accepted changed/ambiguous pattern ownership')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--generated',type=Path,required=True);a=p.parse_args()
    count=0
    def check(value):
        global count
        count+=1;assert value, 'Construction hatch check '+str(count)
    verify(a.source,a.generated,check)
    print(count,'construction-hatch checks passed; native/boat visuals remain pending')

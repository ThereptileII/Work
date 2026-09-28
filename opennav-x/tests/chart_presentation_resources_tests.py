"""Deterministic palette/provenance tests; not ENC visual/safety acceptance."""
import importlib.util
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('generate',ROOT/'tools/generate-xnav-chart-style.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
source=ROOT/'upstream/OpenCPN/data/s57data'
checks=0
def check(value):
    global checks
    checks+=1
    assert value, f'Chart presentation resource check {checks}'
def luminance(rgb):
    channels=[v/255 for v in rgb]
    linear=[v/12.92 if v<=.04045 else ((v+.055)/1.055)**2.4 for v in channels]
    return sum(v*w for v,w in zip(linear,[.2126,.7152,.0722]))
def contrast(a,b):
    bright,dark=sorted([luminance(a),luminance(b)],reverse=True)
    return (bright+.05)/(dark+.05)
with tempfile.TemporaryDirectory(prefix='xnav-chart-test-') as d:
    folder=Path(d)
    output=folder/'generated'
    original={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir() if p.is_file()}
    data=g.generate(source,output)
    check(data['upstreamCommit']=='37fd0cddb7334fe489e9f18aa163977a9c5c84f7')
    check(len(data['palette'])==3)
    first={p.name:p.read_bytes() for p in output.iterdir()}
    check(g.generate(source,output)==data)
    check(first=={p.name:p.read_bytes() for p in output.iterdir()})
    for name,identity in data['files'].items():
        check(hashlib.sha256((output/name).read_bytes()).hexdigest()==identity['sha256'])
        if name!='chartsymbols.xml':check((output/name).read_bytes()==g.pinned_bytes(source/name,identity))
    a,b=ET.parse(source/'chartsymbols.xml').getroot(),ET.parse(output/'chartsymbols.xml').getroot()
    for section in ['lookups','line-styles','patterns','symbols']:
        check(ET.tostring(a.find(section))==ET.tostring(b.find(section)))
    for stock,styled in zip(a.find('color-tables'),b.find('color-tables')):
        check(stock.attrib==styled.attrib)
        for before,after in zip(stock,styled):
            if before.tag=='color' and stock.attrib['name'] in data['palette'] and before.attrib['name'] in g.ALLOWED:
                check(before.attrib['name']==after.attrib['name'])
            else:check(ET.tostring(before)==ET.tostring(after))
    for table,colors in data['palette'].items():
        check(len({tuple(colors[n]) for n in ['DEPDW','DEPMD','DEPMS','DEPVS','DEPIT']})==5)
        check(colors['DEPSC']!=colors['DEPCN'])
        check(colors['SNDG1']!=colors['SNDG2'])
        check(colors['LANDA']!=colors['DEPDW'])
    for color in g.ALLOWED:
        check(luminance(data['palette']['NIGHT'][color])<luminance(data['palette']['DUSK'][color]))
    # Guard the observed invisible dark ink on the new Night water. These
    # numerical checks do not replace actual symbol/hazard review.
    check(data['palette']['DAY_BRIGHT']['CHBLK']==(7,7,7))
    for table in ('DUSK','NIGHT'):
        colors=data['palette'][table]
        check(contrast(colors['CHBLK'],colors['DEPDW'])>=4)
        check(contrast(colors['CHBLK'],colors['DEPVS'])>=2)
        check(contrast(colors['CHBLK'],colors['LANDA'])>=3)
    check(original=={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir() if p.is_file()})
    damaged=folder/'damaged';damaged.mkdir()
    for name in data['files']:shutil.copyfile(source/name,damaged/name)
    for name in ['chartsymbols.xml','S52RAZDS.RLE']:
        (damaged/name).write_bytes((source/name).read_bytes().replace(b'\r\n',b'\n').replace(b'\n',b'\r\n'))
    check(g.generate(damaged,folder/'windows-checkout')==data)
    check(first=={p.name:p.read_bytes() for p in (folder/'windows-checkout').iterdir()})
    (damaged/'chartsymbols.xml').write_bytes((damaged/'chartsymbols.xml').read_bytes()+b'changed')
    try:g.generate(damaged,folder/'refused')
    except AssertionError:check(not (folder/'refused').exists())
    else:raise AssertionError('Unknown presentation source was accepted')
print(f'{checks} chart-resource checks passed; real ENC, native palette review and GL/software gates remain required')

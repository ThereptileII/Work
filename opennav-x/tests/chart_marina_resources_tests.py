"""SCRUM-279 delta proof against the previous generated resources, no app run."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from chart_raster_ink import decode
from chart_service_resources_tests import verify_services

p = argparse.ArgumentParser()
p.add_argument('--source', type=Path, required=True)
p.add_argument('--prior', type=Path, required=True)
p.add_argument('--current', type=Path, required=True)
a = p.parse_args()
checks = 0


def check(value):
    global checks
    checks += 1
    assert value, f'Marina check {checks}'


spec = importlib.util.spec_from_file_location('generator', ROOT/'tools/generate-xnav-chart-style.py')
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)
metadata = json.loads((a.current/'manifest.json').read_text())
prior_xml = (a.prior/'chartsymbols.xml').read_bytes()
new_xml = (a.current/'chartsymbols.xml').read_bytes()
stock_xml = (a.source/'chartsymbols.xml').read_bytes()
before, after = ET.fromstring(prior_xml), ET.fromstring(new_xml)
old = before.find("symbols/symbol[name='SMCFAC02']")
new = after.find("symbols/symbol[name='SMCFAC02']")
check(len(after.findall("symbols/symbol[name='SMCFAC02']")) == 1)
check(old.get('RCID') == new.get('RCID') == '2108')
check(old.find('bitmap').attrib == {'width':'21', 'height':'21'})
check(new.find('bitmap').attrib == {'width':'24', 'height':'24'})
for tag, attrs in [('pivot', {'x':'12','y':'12'}),
                   ('graphics-location', {'x':'820','y':'1160'})]:
    check(new.find('bitmap/'+tag).attrib == attrs)
    new.find('bitmap/'+tag).attrib = old.find('bitmap/'+tag).attrib.copy()
new.find('bitmap').attrib = old.find('bitmap').attrib.copy()
# Byte equality after restoring just the six permitted numeric attributes.
check(ET.tostring(before) == ET.tostring(after))
consumers = [n for n in before.findall('lookups/lookup')
             if 'SMCFAC02' in n.findtext('instruction', '')]
check({n.get('RCID') for n in consumers} ==
      {'32113','32225','32295','32304','32452','32566','32642','32651','31179','30374'})
check(len(consumers) == 10)
for n in consumers:
    same = ET.fromstring(new_xml).find("lookups/lookup[@id='"+n.get('id')+"']")
    check(ET.tostring(n) == ET.tostring(same))
check((a.prior/'S52RAZDS.RLE').read_bytes() == (a.current/'S52RAZDS.RLE').read_bytes())

for name in ('rastersymbols-day.png','rastersymbols-dusk.png','rastersymbols-dark.png'):
    cb, previous = decode((a.prior/name).read_bytes())
    ca, current = decode((a.current/name).read_bytes())
    check([(k,v) for k,v in cb if k != b'IDAT'] == [(k,v) for k,v in ca if k != b'IDAT'])
    changed = [i for i in range(0,len(previous),4) if previous[i:i+4] != current[i:i+4]]
    check(len(changed) == 152)
    check(all(820 <= i//4%1500 < 844 and 1160 <= i//4//1500 < 1184 for i in changed))
    for i in changed:
        check(previous[i+3] == 0 and current[i+3] > 0)
        current[i:i+4] = previous[i:i+4]
    check(current == previous)  # Every other pixel, hidden RGB and previous glyph.

# Existing independent service proof verifies immutable SVG geometry, all three
# service colors/alpha masks, moats, effective rules and scaled/rotated pivots.
verify_services(a.source, a.current, metadata, check)


def reject(mutator):
    tree = ET.fromstring(new_xml)
    mutator(tree)
    try:
        g.validate_resource_changes(stock_xml, ET.tostring(tree), metadata['palette'])
    except AssertionError:
        check(True)
    else:
        raise AssertionError('Altered marina meaning or placement accepted')


for tag, attr, value in [('bitmap','width','25'), ('bitmap/pivot','y','13'),
                          ('bitmap/graphics-location','x','788'), ('bitmap/origin','x','1')]:
    reject(lambda t: t.find("symbols/symbol[name='SMCFAC02']/"+tag).set(attr,value))
reject(lambda t: t.find("symbols/symbol[name='SMCFAC02']").append(ET.fromstring('<prefer-bitmap>no</prefer-bitmap>')))
for rcid in ('32113','32225','32295','32304','32452','32566','32642','32651','31179','30374'):
    reject(lambda t: setattr(t.find("lookups/lookup[@RCID='"+rcid+"']/instruction"), 'text', 'SY(PILBOP02)'))
reject(lambda t: setattr(t.find("lookups/lookup[@RCID='31179']/attrib-code"), 'text', 'CATHAF6'))
reject(lambda t: setattr(t.find("lookups/lookup[@RCID='31179']/display-cat"), 'text', 'Standard'))
print(f'{checks} focused marina resource, original-equality, selector and placement checks passed')

"""Focused SCRUM-265 full-tree reverse proof and structural-area negative controls."""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import chart_structure_paint as structure


def verify(source, generated, check):
    spec = importlib.util.spec_from_file_location('generator', ROOT / 'tools/generate-xnav-chart-style.py')
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    original = (source / 'chartsymbols.xml').read_bytes()
    styled = (generated / 'chartsymbols.xml').read_bytes()
    colors = json.loads((generated / 'manifest.json').read_text())['palette']
    before, after = ET.fromstring(original), ET.fromstring(styled)
    generator.validate_resource_changes(original, styled, colors)
    check(True)
    # Independent source inventory: exact lookup IDs/RCIDs and semantic content.
    identities = {'17':'32053','18':'32054','19':'32055','20':'32056',
                  '61':'32097','98':'32134','135':'32171',
                  '357':'32392','358':'32393','359':'32394','360':'32395',
                  '401':'32436','438':'32473','477':'32512'}
    check(set(structure.RULES) == set(identities))
    for identity, rcid in identities.items():
        query = "lookups/lookup[@id='" + identity + "']"
        old, new = before.find(query), after.find(query)
        check(old.get('RCID') == new.get('RCID') == rcid)
        expected = old.findtext('instruction').replace('AC(CHBRN)', 'AC(XNSTR)', 1)
        if identity in {'18','20','358','360'}:
            expected = expected.replace('LS(SOLD,1,LANDF)', 'LS(SOLD,1,XNSHR)', 1)
        check(new.findtext('instruction') == expected)
        restored = copy.deepcopy(new)
        restored.find('instruction').text = old.findtext('instruction')
        check(ET.tostring(restored) == ET.tostring(old))
    for table, expected in [('DAY_BRIGHT',(238,238,226)),('DUSK',(78,97,93)),('NIGHT',(29,41,37))]:
        check(tuple(colors[table]['XNSTR']) == expected == tuple(colors[table]['LANDA']))
        query = "color-tables/color-table[@name='" + table + "']/color[@name='CHBRN']"
        check(ET.tostring(before.find(query)) == ET.tostring(after.find(query)))
        check(all(tuple(colors[table]['XNSTR']) != tuple(colors[table][role])
                  for role in ('DEPDW','DEPMD','DEPMS','DEPVS','DEPIT','CSTLN')))
    for identity, rcid in [('16','32052'),('356','32391')]:
        query = "lookups/lookup[@id='" + identity + "']"
        old, new = before.find(query), after.find(query)
        check(old.get('RCID') == new.get('RCID') == rcid)
        expected = old.findtext('instruction').replace('AC(CHBRN)','AC(XNBUA)').replace(',CHBLK,26)',',XNGEO,26)').replace('LS(SOLD,1,LANDF)','LS(SOLD,1,XNSHR)')
        check(new.findtext('instruction') == expected)
        restored = copy.deepcopy(new)
        restored.find('instruction').text = old.findtext('instruction')
        check(ET.tostring(restored) == ET.tostring(old))
    check(set(structure.OUTLINE_RULES) == {'16','18','20','356','358','360'})
    for table, expected in [('DAY_BRIGHT',(175,191,174)),('DUSK',(116,135,121)),('NIGHT',(55,68,58))]:
        check(tuple(colors[table]['XNSHR']) == expected == tuple(colors[table]['CSTLN']))
        query = "color-tables/color-table[@name='" + table + "']/color[@name='LANDF']"
        check(ET.tostring(before.find(query)) == ET.tostring(after.find(query)))
    def reject(change):
        tree = ET.fromstring(styled)
        change(tree)
        try:
            generator.validate_resource_changes(original, ET.tostring(tree), colors)
        except AssertionError:
            check(True)
        else:
            raise AssertionError('Accepted a structural paint exception escape')
    for identity in identities:
        query = "lookups/lookup[@id='" + identity + "']"
        reject(lambda tree, q=query: setattr(tree.find(q + '/instruction'), 'text', 'AC(XNSTR)'))
        reject(lambda tree, q=query: setattr(tree.find(q + '/display-cat'), 'text', 'Displaybase-mutated'))
    for identity in ('17','98','438','477'):
        query = "lookups/lookup[@id='" + identity + "']"
        reject(lambda tree, q=query: tree.find(q).set('RCID', '1'))
        reject(lambda tree, q=query: tree.find(q).append(ET.Element('attrib-code', {'invalid':'1'})))
        reject(lambda tree, q=query: tree.find('lookups').append(copy.deepcopy(tree.find(q))))
    # All line/point variants, including CATMOR6 chain and CATMOR7 buoys, stay exact.
    for old, new in zip(before.findall('lookups/lookup'), after.findall('lookups/lookup')):
        if old.get('name') in ('BUISGL','FLODOC','MORFAC','PONTON') and old.findtext('type') != 'Area':
            if old.get('id')=='1091':
                restored=copy.deepcopy(new);check(restored.findtext('instruction')=='SY(XNBLDG01)');restored.find('instruction').text='SY(BUISGL01)'
                check(ET.tostring(old)==ET.tostring(restored));continue
            check(ET.tostring(old) == ET.tostring(new))
    reject(lambda tree: setattr(tree.find("lookups/lookup[@id='768']/instruction"), 'text', 'LS(DASH,1,XNSTR)'))
    reject(lambda tree: setattr(tree.find("lookups/lookup[@id='1185']/instruction"), 'text', 'AC(XNSTR);SY(BOYMOR11)'))
    color = "color-tables/color-table/color[@name='XNSTR']"
    reject(lambda tree: tree.find(color).set('a', '.78'))
    reject(lambda tree: tree.find(color).set('r', '0'))
    reject(lambda tree: tree.find('color-tables/color-table').remove(tree.find(color)))
    reject(lambda tree: tree.find('color-tables/color-table').append(copy.deepcopy(tree.find(color))))
    reject(lambda tree: tree.find("color-tables/color-table/color[@name='CHBRN']").set('r', '238'))
    for identity in ('16','18','20','356','358','360'):
        query = "lookups/lookup[@id='" + identity + "']/instruction"
        for replacement in ('LS(SOLD,2,XNSHR)','LS(DASH,1,XNSHR)','LS(SOLD,1,LANDF)'):
            reject(lambda tree, q=query, r=replacement: setattr(tree.find(q),'text',tree.find(q).text.replace('LS(SOLD,1,XNSHR)',r)))
    for identity in ('17','19','357','359','61','98','135'):
        query = "lookups/lookup[@id='" + identity + "']/instruction"
        reject(lambda tree, q=query: setattr(tree.find(q),'text',tree.find(q).text.replace('CHBLK','XNSHR').replace('CSTLN','XNSHR')))
    reject(lambda tree: tree.find("color-tables/color-table/color[@name='LANDF']").set('r','175'))
    color = "color-tables/color-table/color[@name='XNSHR']"
    reject(lambda tree: tree.find(color).set('a','.78'))
    reject(lambda tree: tree.find(color).set('r','0'))
    reject(lambda tree: tree.find('color-tables/color-table').remove(tree.find(color)))
    # Source drift also fails closed before painting.
    for identity in ('17','61','98','135'):
        tree = ET.fromstring(original)
        query = "lookups/lookup[@id='" + identity + "']"
        tree.find(query + '/instruction').text = 'AC(CHBRN);LS(SOLD,9,CHBLK)'
        try:
            structure.recolor(ET.tostring(tree, encoding='unicode'))
        except AssertionError:
            check(True)
        else:
            raise AssertionError('Accepted changed source rule')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--generated', type=Path, required=True)
    args = parser.parse_args()
    checks = []
    def check(value):
        assert value
        checks.append(True)
    verify(args.source, args.generated, check)
    print(f'{len(checks)} structural-area checks passed; actual renderer/native readability remains open')

"""SCRUM-265: isolated land fill for fourteen pinned structural area rules."""
import re
import xml.etree.ElementTree as ET

COLOR = 'XNSTR'
# identity: RCID, class, area table, ordered selectors, exact stock instruction.
BUILDING_LABEL = "TX(OBJNAM,1,2,2,'15110',0,0,CHBLK,26);"
RULES = {}
for table, ids, rcids in (
        ('Plain', (17, 18, 19, 20), (32053, 32054, 32055, 32056)),
        ('Symbolized', (357, 358, 359, 360), (32392, 32393, 32394, 32395))):
    for identity, rcid, selectors, label, border in zip(
            ids, rcids, [('FUNCTN33', 'CONVIS1'), ('FUNCTN33',), ('CONVIS1',), ()],
            [BUILDING_LABEL, BUILDING_LABEL, '', ''], ['CHBLK', 'LANDF', 'CHBLK', 'LANDF']):
        RULES[str(identity)] = (str(rcid), 'BUISGL', table, selectors,
                               f'AC(CHBRN);{label}LS(SOLD,1,{border})')
for identity, rcid, name, table in (
        (61, 32097, 'FLODOC', 'Plain'), (401, 32436, 'FLODOC', 'Symbolized'),
        (98, 32134, 'MORFAC', 'Plain'), (438, 32473, 'MORFAC', 'Symbolized'),
        (135, 32171, 'PONTON', 'Plain'), (477, 32512, 'PONTON', 'Symbolized')):
    border = 'LS(SOLD,1,CHBLK)' if name == 'MORFAC' else 'LS(SOLD,2,CSTLN)'
    RULES[str(identity)] = (str(rcid), name, table, (), 'AC(CHBRN);' + border)


def instruction(lookup):
    """Never infer eligibility from a brown pixel, class alone or a point symbol."""
    original = lookup.findtext('instruction')
    identity = lookup.get('id')
    if identity not in RULES:
        return original
    rcid, name, table, selectors, expected = RULES[identity]
    assert lookup.attrib == {'id': identity, 'RCID': rcid, 'name': name}
    assert lookup.findtext('type') == 'Area'
    assert lookup.findtext('table-name') == table
    assert tuple(n.text for n in lookup.findall('attrib-code')) == selectors
    assert original == expected, 'Pinned structural area rule changed'
    return original.replace('AC(CHBRN)', 'AC(XNSTR)', 1)


def recolor(xml):
    tree = ET.fromstring(xml)
    for identity, (rcid, name, _, _, _) in RULES.items():
        nodes = tree.findall("lookups/lookup[@id='" + identity + "']")
        assert len(nodes) == 1, 'Missing or duplicated structural area lookup'
        after = instruction(nodes[0])
        before = nodes[0].findtext('instruction')
        pattern = (r'(<lookup id="' + identity + r'" RCID="' + rcid +
                   r'" name="' + name + r'">)(.*?)(</lookup>)')
        matches = list(re.finditer(pattern, xml, re.S))
        token = '<instruction>' + before + '</instruction>'
        assert len(matches) == 1 and matches[0][2].count(token) == 1
        xml = re.sub(pattern, lambda m: m[1] + m[2].replace(
            token, '<instruction>' + after + '</instruction>') + m[3], xml, flags=re.S)
    return xml


OUTLINE_COLOR = 'XNSHR'
OUTLINE_RULES = {identity: RULES[identity] for identity in ('18', '20', '358', '360')}
for identity, rcid, table in (('16', '32052', 'Plain'), ('356', '32391', 'Symbolized')):
    OUTLINE_RULES[identity] = (
        rcid, 'BUAARE', table, (),
        "AC(CHBRN);TX(OBJNAM,1,2,3,'16120',0,0,CHBLK,26);LS(SOLD,1,LANDF)")


def outline_instruction(lookup, current):
    identity = lookup.get('id')
    if identity not in OUTLINE_RULES:
        return current
    rcid, name, table, selectors, expected = OUTLINE_RULES[identity]
    assert lookup.attrib == {'id': identity, 'RCID': rcid, 'name': name}
    assert lookup.findtext('type') == 'Area' and lookup.findtext('table-name') == table
    assert tuple(n.text for n in lookup.findall('attrib-code')) == selectors
    assert lookup.findtext('instruction') == expected, 'Pinned building outline rule changed'
    assert current.count('LS(SOLD,1,LANDF)') == 1
    return current.replace('LS(SOLD,1,LANDF)', 'LS(SOLD,1,XNSHR)', 1)


def recolor_outlines(xml):
    tree = ET.fromstring(xml)
    for identity, (rcid, name, table, selectors, original) in OUTLINE_RULES.items():
        nodes = tree.findall("lookups/lookup[@id='" + identity + "']")
        assert len(nodes) == 1, 'Missing or duplicated building outline lookup'
        lookup = nodes[0]
        # These exact earlier owned transforms have already run in the generator.
        before = original.replace('AC(CHBRN)', 'AC(XNBUA)' if name == 'BUAARE' else 'AC(XNSTR)')
        if name == 'BUAARE':
            before = before.replace(',CHBLK,26)', ',XNGEO,26)')
        assert lookup.findtext('instruction') == before
        lookup.find('instruction').text = original
        after = outline_instruction(lookup, before)
        pattern = (r'(<lookup id="' + identity + r'" RCID="' + rcid +
                   r'" name="' + name + r'">)(.*?)(</lookup>)')
        matches = list(re.finditer(pattern, xml, re.S))
        token = '<instruction>' + before + '</instruction>'
        assert len(matches) == 1 and matches[0][2].count(token) == 1
        xml = re.sub(pattern, lambda m: m[1] + m[2].replace(
            token, '<instruction>' + after + '</instruction>') + m[3], xml, flags=re.S)
    return xml

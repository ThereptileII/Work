#!/usr/bin/env python3
"""Account for every effective pinned point, line and pattern definition."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from chart_symbol_svg import ROOT, ASSETS, NATIVE_NAMES, entries

SECTIONS = (('point', 'symbols', 'symbol'),
            ('line', 'line-styles', 'line-style'),
            ('pattern', 'patterns', 'pattern'))


def retained_reason(kind, name, description):
    if kind != 'point':
        return ('Native repeating ' + kind + ' geometry preserves area/line conventions; '
                'point artwork is not a replacement for this portrayal.')
    if name in {'BCNLAT23', 'BCNLAT24'}:
        return 'Inland lateral beacon variant; keep its distinct native color composition and waterway semantics.'
    if name == 'BCNDEF13':
        return 'Default/unknown beacon; a classified glyph would assert unavailable mark attributes.'
    if name in {'BCNLAT21', 'BCNLAT22', 'BCNSAW21', 'BCNSPP21'}:
        return 'Minor beacon bitmap has an off-center anchor unlike its vector geometry; fitting a redraw around the exact existing anchor would severely shrink the shape. Retain native bitmap and pivot.'
    if name == 'OBSTRN11':
        return 'Bitmap-only above-water obstruction; no verified vector geometry is available for an equivalent redraw.'
    if name.startswith(('TOP', 'DAY', 'TMAR')):
        return 'Native physical topmark/daymark geometry is composed separately by conditional rules; do not invent or merge it.'
    if name.startswith(('BOY', 'BCN', 'LIT', 'LIGHT', 'FOG')):
        return 'Native specialized mark/light variant retained where no exact qualified prototype mapping exists; color, shape and conditional selection remain authoritative.'
    if name.startswith(('SOUND', 'DEP', 'DQUAL', 'QUA', 'LOWACC', 'NODATA', 'OVERSC')):
        return 'Depth, sounding or survey-quality notation retains its native categorical/digit convention.'
    if name.startswith(('WRECK', 'OBSTRN', 'UWTROC', 'DANGER', 'ISODGR', 'FOULAR')):
        return 'Distinct native danger or water-level notation retained; no semantically equivalent custom artwork is approved.'
    if name.startswith(('NOT', 'ADDMRK', 'BNK', 'WTW', 'INL', 'SIST', 'NMK')) or 'inland' in description.lower():
        return 'Specialized inland/waterway sign retains its standardized native notation; no exact prototype equivalent.'
    if name.startswith(('AIS', 'ARP', 'OWNSHP', 'VESS', 'PASTRK', 'PLNPOS', 'NORTHAR', 'SCALEB', 'VEC')):
        return 'Navigation/control or vessel notation retains native meaning and state distinctions; outside the reviewed chart-mark artwork.'
    return 'Native portrayal retained: the prototype has no reviewed exact artwork mapping for this object; generic reference-pin previews are not navigation glyphs.'


def audit(source, styled):
    source_bytes = source.read_bytes().replace(b'\r\n', b'\n')
    lock = json.loads((ASSETS / 'source-lock.json').read_text())
    assert hashlib.sha256(source_bytes).hexdigest() == lock['files']['chartsymbols.xml']['sha256']
    before, after = ET.fromstring(source_bytes), ET.parse(styled).getroot()
    svg_items = entries(source_bytes, styled.read_bytes())
    svg_names = {item['name'] for item in svg_items}
    proto_path = ROOT / 'docs/design/prototype'
    art = (proto_path / 'src/chart-marker-art.js').read_text()
    designed = set(re.findall(r"'((?:point|line|pattern):[A-Z0-9]+)'", art))
    guide = json.loads((proto_path / 'src/seamarks.json').read_text())
    for mark in guide:
        for region in ('A', 'B'):
            code = mark.get('code' + region) or mark.get('code')
            if code:
                designed.add('point:' + code)
    prototype = json.loads((proto_path / 'src/symbol-catalogue.json').read_text())
    catalogue = {item['id']: item for item in prototype['items']}
    aliases = defaultdict(list)
    mapping = json.loads((ASSETS / 'seamarks/mapping.json').read_text())['symbols']
    for name, item in mapping.items():
        if name != item['source']:
            aliases[item['source']].append(name)
    aliases['BCNGEN01'].append('XNBCNG01')
    aliases['BOYSPP11'].extend(('XNSPPW01', 'XNSPPY01', 'XNSPPT01'))
    aliases['BUISGL01'].append('XNBLDG01')
    aliases['TOWERS01'].append('XNLIT013')
    aliases['TOWERS03'].append('XNLIT013')
    conditional = {
        'BCNGEN01': 'Only exact Simplified generic lookups 1696/1708 use XNBCNG01; classified and Paper bodies remain native.',
        'BOYSPP11': 'Runtime guards require exact white/yellow special-buoy attributes; X head requires explicit matching topmark. Other shapes/statuses remain native.',
        'BUISGL01': 'Only exact generic-building lookup 1091 receives XNBLDG01; other buildings remain native.',
        'TOWERS01': 'Runtime light-support tower guard requires Simplified LNDMRK CATLMK=17/FUNCTN=33. It uses XNLIT013; ordinary towers remain native.',
        'TOWERS03': 'Runtime light-support tower guard requires Simplified LNDMRK CATLMK=17/FUNCTN=33. It uses XNLIT013; conspicuous ordinary towers remain native.',
    }
    refs = defaultdict(list)
    for lookup in before.findall('lookups/lookup'):
        instruction = lookup.findtext('instruction') or ''
        for operation, name in re.findall(r'\b(SY|LC|AP)\(([A-Z0-9]{8})', instruction):
            kind = {'SY': 'point', 'LC': 'line', 'AP': 'pattern'}[operation]
            refs[(kind, name)].append({'id': lookup.get('id'),
                                      'object': lookup.get('name'),
                                      'table': lookup.findtext('table-name')})
    records, counts, shadowed = [], {}, []
    for kind, section, element in SECTIONS:
        groups = defaultdict(list)
        for node in before.findall(f'{section}/{element}'):
            groups[node.findtext('name')].append(node)
        effective_after = {n.findtext('name'): n for n in after.findall(f'{section}/{element}')}
        counts[kind] = {'definitions': sum(map(len, groups.values())), 'effectiveNames': len(groups)}
        for name, nodes in sorted(groups.items()):
            node = nodes[-1]
            desc = next((n.findtext('description') for n in reversed(nodes) if n.findtext('description')), '')
            identifier = kind + ':' + name
            status, reason = 'native-retained', retained_reason(kind, name, desc)
            if kind == 'point' and name in NATIVE_NAMES:
                status = 'native-geometry-redraw'
                reason = 'Verified native vector silhouette and categorical color roles rendered at requested size; original bitmap size/pivot and selection retained. Stock atlas remains the SVG-unavailable fallback.'
            elif kind == 'point' and name in svg_names:
                status = 'prototype-art'
                reason = 'Reviewed prototype point artwork replaces only this effective bitmap; original lookup/conditional selection remains unchanged.'
            elif kind == 'point' and name in aliases:
                status = 'qualified-prototype-alias'
                reason = conditional.get(name) or (
                    'Only existing qualified Simplified marine/light selections use the listed owned alias; native source glyph remains for unqualified/Paper portrayals.')
            elif identifier == 'line:CBLSUB06':
                status = 'prototype-line-art'
                reason = 'Reviewed prototype cable wave; native cable classification and paint-role boundaries remain.'
            elif identifier == 'pattern:FSHFAC03':
                status = 'prototype-pattern-art'
                reason = 'Reviewed prototype fishing diamond; fishing-facility selector/repetition semantics remain.'
            # Other owned resource transformations are paint treatment, not a
            # new shape. The complete generator separately proves those edits.
            transformed = ET.tostring(node) != ET.tostring(effective_after[name])
            entry = {'id': identifier, 'description': desc,
                     'effectiveRcid': node.get('RCID'), 'definitionCount': len(nodes),
                     'status': status, 'reason': reason,
                     'prototypeDesignedArtwork': identifier in designed,
                     'prototypeCategory': catalogue.get(identifier, {}).get('category', 'not-in-prototype-catalogue'),
                     'derivedResourceNodeChanged': transformed,
                     'scalableSvg': kind == 'point' and name in svg_names,
                     'qualifiedAliases': sorted(aliases.get(name, [])) if kind == 'point' else [],
                     'directLookupReferences': refs[(kind, name)],
                     'referenceLimit': 'Direct XML references only; conditional procedures also select symbols at runtime.'}
            records.append(entry)
            if len(nodes) > 1:
                shadowed.append({'id': identifier, 'effectiveRcid': node.get('RCID'),
                                 'shadowedRcids': [n.get('RCID') for n in nodes[:-1]],
                                 'reason': 'Upstream last definition per name/type wins; earlier definitions are not additional effective glyphs.'})
    generated_aliases = []
    original_names = {node.findtext('name') for node in before.findall('symbols/symbol')}
    for node in after.findall('symbols/symbol'):
        name = node.findtext('name')
        if name not in original_names:
            generated_aliases.append({'name': name, 'rcid': node.get('RCID'),
                                      'scalableSvg': name in svg_names,
                                      'note': ('Retained resource alias with native tower geometry; current qualified runtime light-tower path uses XNLIT013.'
                                               if name.startswith('XNLTWR') else
                                               'Owned alias only; does not replace all uses of its native source symbol.')})
    return {
        'schema': 1, 'issue': 'SCRUM-378',
        'upstreamCommit': lock['upstreamCommit'],
        'sourceXmlSha256': hashlib.sha256(source_bytes).hexdigest(),
        'prototypeCatalogueSource': prototype['source'],
        'prototypeSourceHashes': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in (proto_path / 'index.html', proto_path / 'src/chart-marker-art.js',
                                           proto_path / 'src/seamarks.json', proto_path / 'src/symbol-catalogue.json')},
        'countByKind': counts,
        'statusCounts': dict(sorted(Counter(item['status'] for item in records).items())),
        'scalablePointNames': sorted(svg_names),
        'scalablePointEntries': len(svg_items),
        'prototypeDesignedIds': sorted(designed),
        'prototypeCatalogueOnlyIds': sorted(set(catalogue) - {r['id'] for r in records}),
        'pinnedOnlyIds': sorted({r['id'] for r in records} - set(catalogue)),
        'limits': 'This inventories resource and source selection coverage, not chart availability, native Windows rendering, chart correctness or boat acceptance. Standard/Legacy source resources remain untouched.',
        'shadowedDefinitions': shadowed, 'generatedAliases': generated_aliases,
        'symbols': records}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--styled', required=True, type=Path)
    parser.add_argument('--output', default=ASSETS / 'symbol-coverage.json', type=Path)
    args = parser.parse_args()
    result = audit(args.source, args.styled)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: result[key] for key in ('countByKind', 'statusCounts', 'scalablePointEntries')}))

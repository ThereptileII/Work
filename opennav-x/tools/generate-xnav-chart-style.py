#!/usr/bin/env python3
"""Derive bounded XNav palette resources from verified pinned OpenCPN bytes.

Only the enumerated palette roles, geographic-name ink, two built-up-area fill tokens and a proven
neutral sprite-ink mask, isolated ACHARE51 artwork tile and CBLSUB06 paint role may change. Original inputs are never modified.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET
from chart_raster_ink import decode, derive
import chart_anchor_art
import chart_cable_paint

ROOT=Path(__file__).resolve().parents[1]
ALLOWED={'LANDA','CSTLN','DEPDW','DEPMD','DEPMS','DEPVS','DEPIT','DEPCN','DEPSC','SNDG1','SNDG2','CHBLK','CHGRD'}
# CHBRN is also used by above-water hazards and obscured light sectors. Keep it
# intact: only these pinned BUAARE area lookups receive the separate paint role.
BUILT_AREA_LOOKUPS={'16':('32052','Plain'),'356':('32391','Symbolized')}
BUILT_AREA_INSTRUCTION="AC(CHBRN);TX(OBJNAM,1,2,3,'16120',0,0,CHBLK,26);LS(SOLD,1,LANDF)"
BUILT_AREA_COLOR='XNBUA'
GEOGRAPHIC_COLOR='XNGEO'
ADDED_COLORS={BUILT_AREA_COLOR,GEOGRAPHIC_COLOR,chart_cable_paint.COLOR}
GEOGRAPHIC_CLASSES={'BUAARE','LNDARE','LNDRGN','SEAARE'}

def geographic_ink(name, instruction):
    if name not in GEOGRAPHIC_CLASSES:return instruction
    # Paint only an OBJNAM TX; no offsets, size specification, classification,
    # symbols, conditional procedures or other attributes may change.
    return re.sub(r"(TX\(OBJNAM,[^;()]+,)(?:CHBLK|CHGRD)(,26\))",
                  r"\g<1>XNGEO\2",instruction)

def styled_instruction(lookup):
    instruction=lookup.findtext('instruction')
    if lookup.get('id') in BUILT_AREA_LOOKUPS:
        assert instruction==BUILT_AREA_INSTRUCTION
        instruction=instruction.replace('AC(CHBRN)','AC(XNBUA)')
    return geographic_ink(lookup.get('name'),instruction)


def pinned_bytes(path, identity):
    content=path.read_bytes()
    # Git's Windows checkout may translate the two text resources to CRLF.
    # Normalize ONLY that known checkout transformation, then require the exact
    # canonical pinned byte length/hash. Do not accept arbitrary whitespace or
    # semantic-equivalence changes. Sprite bytes are always checked verbatim.
    if path.name in {'chartsymbols.xml','S52RAZDS.RLE'}:
        canonical=content.replace(b'\r\n',b'\n')
        if len(canonical)==identity['bytes'] and hashlib.sha256(canonical).hexdigest()==identity['sha256']:
            content=canonical
    assert len(content)==identity['bytes'] and hashlib.sha256(content).hexdigest()==identity['sha256'], 'Pinned resource changed: '+path.name
    return content

def validate_resource_changes(original, styled, colors):
    # Only named paint changes may differ; all remaining rule trees must match.
    before,after=ET.fromstring(original),ET.fromstring(styled)
    for table in after.find('color-tables'):
        source_table=next(t for t in before.find('color-tables') if t.attrib==table.attrib)
        if table.attrib['name'] in colors:
            for name in ADDED_COLORS:
                assert len(table.findall("color[@name='"+name+"']"))==1, 'Missing or duplicated dedicated color'
        for color in table.findall('color'):
            name=color.attrib['name']
            if table.attrib['name'] in colors and name in ALLOWED|ADDED_COLORS:
                rgb=colors[table.attrib['name']][name]
                expected={'name':name,**dict(zip(('r','g','b'),map(str,rgb)))}
                assert color.attrib==expected, 'Unexpected palette color attributes: '+name
                assert len(color)==0 and not (color.text or '').strip(), 'Unexpected palette color content: '+name
                if name in ADDED_COLORS:
                    table.remove(color)
                else:
                    color.attrib=next(c for c in source_table.findall('color') if c.attrib['name']==name).attrib.copy()
    assert len(before.find('lookups'))==len(after.find('lookups'))
    for stock,styled in zip(before.find('lookups'),after.find('lookups')):
        assert styled.findtext('instruction')==styled_instruction(stock), 'Lookup changed beyond approved paint roles'
        styled.find('instruction').text=stock.findtext('instruction')
    chart_anchor_art.restore_bitmap_for_validation(before, after)
    chart_cable_paint.restore_for_validation(before, after)
    # Added nodes must not make whitespace significant in the identity check.
    for tree in (before,after):
        for node in tree.iter():
            if node.text is not None and not node.text.strip():node.text=None
            if node.tail is not None and not node.tail.strip():node.tail=None
    assert ET.tostring(before)==ET.tostring(after), 'Presentation semantics changed'

def generate(source, output):
    definition=json.loads((ROOT/'resources/chart-style/v1/definition.json').read_text())
    lock=json.loads((ROOT/'resources/chart-style/v1/source-lock.json').read_text())
    tokens=json.loads((ROOT/'docs/design/prototype-tokens.json').read_text())
    prototype=(ROOT/'docs/design/prototype/index.html').read_bytes()
    assert hashlib.sha256(prototype).hexdigest()==tokens['htmlSha256']==definition['prototypeSha256']
    original={}
    for name,identity in lock['files'].items():
        original[name]=pinned_bytes(source/name,identity)
    xml=original['chartsymbols.xml'].decode('utf-8')
    colors={}
    for table,item in definition['themes'].items():
        assert set(item['colors'])==ALLOWED|ADDED_COLORS
        colors[table]={}
        for name,value in item['colors'].items():
            value=tokens['themes'][item['theme']][value] if value.startswith('--') else value
            assert re.fullmatch('#[0-9a-fA-F]{6}',value)
            colors[table][name]=tuple(int(value[i:i+2],16) for i in (1,3,5))
            if name==chart_cable_paint.COLOR:
                colors[table][name]=chart_cable_paint.theme_ink(table,colors[table][name])
        pattern=r'(<color-table name="'+re.escape(table)+r'">)(.*?)(</color-table>)'
        assert len(re.findall(pattern,xml,re.S))==1
        def table_replace(match):
            body=match[2]
            for name,rgb in colors[table].items():
                if name in ADDED_COLORS:
                    assert 'name="'+name+'"' not in body
                    body+='<color name="%s" r="%s" g="%s" b="%s"/>\n        '%((name,)+rgb)
                    continue
                color_pattern=r'(<color name="'+name+r'" r=")\d+(" g=")\d+(" b=")\d+("\s*/>)'
                assert len(re.findall(color_pattern,body))==1
                body=re.sub(color_pattern,lambda m:m[1]+str(rgb[0])+m[2]+str(rgb[1])+m[3]+str(rgb[2])+m[4],body)
            return match[1]+body+match[3]
        xml=re.sub(pattern,table_replace,xml,flags=re.S)
    for identity,(rcid,table) in BUILT_AREA_LOOKUPS.items():
        pattern=r'(<lookup id="'+identity+r'" RCID="'+rcid+r'" name="BUAARE">)(.*?)(</lookup>)'
        matches=list(re.finditer(pattern,xml,re.S))
        assert len(matches)==1, 'Pinned built-up-area lookup missing'
        body=matches[0][2]
        assert '<type>Area</type>' in body and '<table-name>'+table+'</table-name>' in body
        old='<instruction>'+BUILT_AREA_INSTRUCTION+'</instruction>'
        assert body.count(old)==1, 'Pinned built-up-area paint changed'
        xml=re.sub(pattern,lambda m:m[1]+m[2].replace(old,old.replace('AC(CHBRN)','AC(XNBUA)'))+m[3],xml,flags=re.S)
    geography_count=0
    def name_replace(match):
        nonlocal geography_count
        before=match[3]
        after=geographic_ink(match[2],before)
        if before!=after:geography_count+=1
        return match[1]+after+match[4]
    xml=re.sub(r'(<lookup\b[^>]*name="([^"]+)"[^>]*>)(.*?)(</lookup>)',name_replace,xml,flags=re.S)
    assert geography_count==18, 'Pinned geographic name lookup count changed'
    xml=chart_anchor_art.relocate(xml)
    xml=chart_cable_paint.recolor(xml)
    validate_resource_changes(original['chartsymbols.xml'],xml,colors)
    result=dict(original);result['chartsymbols.xml']=xml.encode('utf-8')
    # Pinned Day ink identifies neutral CHBLK/CHGRD pixels. Theme sheets use
    # different baked neutral RGBs than the XML table. Change only matching
    # same-coordinate/alpha pixels, never a chromatic pixel or symbol shape.
    _, day_pixels = decode(original['rastersymbols-day.png'])
    raster_ink = {}
    for table, name, source_rgb in [('DUSK','rastersymbols-dusk.png',(54,54,54)),
                                   ('NIGHT','rastersymbols-dark.png',(27,27,27))]:
        assert colors[table]['CHBLK'] == colors[table]['CHGRD']
        result[name], count = derive(day_pixels, original[name], source_rgb, colors[table]['CHBLK'])
        raster_ink[name] = {'sourceRgb':source_rgb, 'targetRgb':colors[table]['CHBLK'],
                            'changedPixels':count, 'alphaAndGeometryPreserved':True}
    anchor_art = {}
    for table, name in [('DAY_BRIGHT','rastersymbols-day.png'),
                        ('DUSK','rastersymbols-dusk.png'),
                        ('NIGHT','rastersymbols-dark.png')]:
        result[name], anchor_art[name] = chart_anchor_art.paint(result[name], table)
    output.mkdir(parents=True,exist_ok=True)
    def write(path,content):
        if not path.exists() or path.read_bytes()!=content:path.write_bytes(content)
    for name,content in result.items():write(output/name,content)
    metadata={'version':definition['version'],'upstreamCommit':lock['upstreamCommit'],
              'prototypeSha256':definition['prototypeSha256'],'palette':colors,
              'neutralRasterInk':raster_ink,
              'anchorageArtwork':anchor_art,
              'geographicNameLookups':geography_count,
              'submarineCablePaint':{'name':'CBLSUB06','RCID':'2012','color':'XNCBL','unchangedHPGL':True},
              'files':{n:{'sha256':hashlib.sha256(c).hexdigest(),'bytes':len(c)} for n,c in result.items()}}
    write(output/'manifest.json',(json.dumps(metadata,indent=2)+'\n').encode())
    header=['#pragma once','#include <cstdint>','namespace opennav::chart_style::generated {',
            'struct Resource { const char *name; const char *sha256; std::uint64_t bytes; };',
            'inline constexpr Resource resources[] = {']
    for name,identity in metadata['files'].items():header.append('  {"%s", "%s", %s},'%(name,identity['sha256'],identity['bytes']))
    header+=['};','struct Background { std::uint32_t land, water; };','inline constexpr Background backgrounds[] = {']
    for key in ['DAY_BRIGHT','DUSK','NIGHT']:
        def color(name):return '0x'+''.join(f'{v:02x}' for v in colors[key][name])
        header.append('  {%s, %s},'%(color('LANDA'),color('DEPDW')))
    header+=['};','} // namespace opennav::chart_style::generated','']
    write(output/'XNavChartResources.h','\n'.join(header).encode())
    return metadata

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();m=generate(a.source,a.output)
    print('Verified pinned resources; generated three chart palettes, two BUAARE fills, 18 geographic-name ink rules, isolated ACHARE51 artwork, CBLSUB06 paint and resource hashes; other navigation rules unchanged')

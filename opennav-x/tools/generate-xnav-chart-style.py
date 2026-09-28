#!/usr/bin/env python3
"""Derive bounded XNav palette resources from verified pinned OpenCPN bytes.

Only eleven named colors in three color tables may change. Original resources
are never modified. The original prototype is never written.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
ALLOWED={'LANDA','CSTLN','DEPDW','DEPMD','DEPMS','DEPVS','DEPIT','DEPCN','DEPSC','SNDG1','SNDG2'}

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
        assert set(item['colors'])==ALLOWED
        colors[table]={}
        for name,value in item['colors'].items():
            value=tokens['themes'][item['theme']][value] if value.startswith('--') else value
            assert re.fullmatch('#[0-9a-fA-F]{6}',value)
            colors[table][name]=tuple(int(value[i:i+2],16) for i in (1,3,5))
        pattern=r'(<color-table name="'+re.escape(table)+r'">)(.*?)(</color-table>)'
        assert len(re.findall(pattern,xml,re.S))==1
        def table_replace(match):
            body=match[2]
            for name,rgb in colors[table].items():
                color_pattern=r'(<color name="'+name+r'" r=")\d+(" g=")\d+(" b=")\d+("\s*/>)'
                assert len(re.findall(color_pattern,body))==1
                body=re.sub(color_pattern,lambda m:m[1]+str(rgb[0])+m[2]+str(rgb[1])+m[3]+str(rgb[2])+m[4],body)
            return match[1]+body+match[3]
        xml=re.sub(pattern,table_replace,xml,flags=re.S)
    # Semantic identity after normalizing the explicit palette overrides.
    before,after=ET.fromstring(original['chartsymbols.xml']),ET.fromstring(xml)
    for table in after.find('color-tables'):
        source_table=next(t for t in before.find('color-tables') if t.attrib==table.attrib)
        for color in table.findall('color'):
            if table.attrib['name'] in colors and color.attrib['name'] in ALLOWED:
                color.attrib=next(c for c in source_table.findall('color') if c.attrib['name']==color.attrib['name']).attrib.copy()
    assert ET.tostring(before)==ET.tostring(after), 'Presentation semantics changed'
    result=dict(original);result['chartsymbols.xml']=xml.encode('utf-8')
    output.mkdir(parents=True,exist_ok=True)
    def write(path,content):
        if not path.exists() or path.read_bytes()!=content:path.write_bytes(content)
    for name,content in result.items():write(output/name,content)
    metadata={'version':definition['version'],'upstreamCommit':lock['upstreamCommit'],
              'prototypeSha256':definition['prototypeSha256'],'palette':colors,
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
    print('Verified pinned resources; generated three XNav palettes, unchanged symbols/lookups and resource hashes')

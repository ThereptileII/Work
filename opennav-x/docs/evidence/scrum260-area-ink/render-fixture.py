"""Render actual pinned ferry HPGL pen paths and a labelled boundary stroke sample.
The sample geometry/stroke scale is a comparison aid, never an ENC screenshot.
"""
import argparse,hashlib,json,re,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
p=argparse.ArgumentParser();p.add_argument('--stock',type=Path,required=True);p.add_argument('--styled',type=Path,required=True);a=p.parse_args()
stock=ET.parse(a.stock).getroot();styled=ET.parse(a.styled).getroot();out=Path(__file__).resolve().parent
node=stock.find("line-styles/line-style[name='FERYRT01']");hpgl=node.findtext('HPGL');paths=[];current=[]
for token in hpgl.split(';'):
 if not token:continue
 op,args=token[:2],token[2:]
 if op in ('SP','SW'):continue
 assert op in ('PU','PD'),op
 nums=[int(x) for x in args.split(',')];assert len(nums)%2==0
 points=list(zip(nums[::2],nums[1::2]))
 if op=='PU':
  if current:paths.append(current)
  current=points
 else:current.extend(points)
if current:paths.append(current)
parts=['<svg xmlns="http://www.w3.org/2000/svg" width="900" height="690" viewBox="0 0 900 690">','<rect width="900" height="690" fill="#fff"/>','<g font-family="Liberation Sans" fill="#233e3e">','<text x="24" y="29" font-size="20">SCRUM-260 — exact ink, unchanged ferry pen paths</text>','<text x="24" y="51" font-size="13">Comparison stroke/scale; not an OpenCPN render or geographic feature capture.</text>']
receipts={}
for row,theme in enumerate(('DAY_BRIGHT','DUSK','NIGHT')):
 y=75+row*200;table=styled.find("color-tables/color-table[@name='"+theme+"']");oldtab=stock.find("color-tables/color-table[@name='"+theme+"']")
 def rgb(tab,name):
  c=tab.find("color[@name='"+name+"']");return '#'+''.join(f'{int(c.get(k)):02x}' for k in ('r','g','b'))
 bg=rgb(table,'DEPDW');before=rgb(oldtab,'CHMGD');after=rgb(table,'XNARE');receipts[theme]={'before':before,'after':after,'background':bg}
 parts.append(f'<text x="24" y="{y+16}" font-size="14">{theme}</text>')
 for col,ink,label in [(0,before,'Before CHMGD'),(1,after,'After XNARE')]:
  x=24+col*440
  parts.extend([f'<rect x="{x}" y="{y+25}" width="416" height="153" rx="5" fill="{bg}"/>',f'<text x="{x+12}" y="{y+44}" font-size="13" fill="{rgb(table,"CHBLK")}">{label} · {ink}</text>'])
  for repeat in range(3):
   for points in paths:
    d=' '.join(('M' if i==0 else 'L')+f'{x+20+repeat*125+(px-387)*.055:.3f},{y+85+(py-853)*.055:.3f}' for i,(px,py) in enumerate(points))
    parts.append(f'<path d="{d}" fill="none" stroke="{ink}" stroke-width="1.3"/>')
  parts.extend([f'<path d="M{x+16} {y+131} H{x+400}" stroke="{ink}" stroke-width="2" stroke-dasharray="6 4"/>',f'<text x="{x+12}" y="{y+165}" font-size="12" fill="{rgb(table,"CHBLK")}">FERYRT01 paths above · CBLARE line sample below</text>'])
parts.extend(['</g></svg>']);(out/'before-after.svg').write_text('\n'.join(parts)+'\n')
subprocess.run(['rsvg-convert',str(out/'before-after.svg'),'-o',str(out/'before-after.png')],check=True)
(out/'fixture-inputs.json').write_text(json.dumps({'stockXML':hashlib.sha256(a.stock.read_bytes()).hexdigest(),'styledXML':hashlib.sha256(a.styled.read_bytes()).hexdigest(),'ferryNodeRCID':'2019','hpgl':hpgl,'inks':receipts,'geometry':'actual ferry HPGL; illustrative straight cable-area boundary comparison only','nativeAcceptance':False},indent=2)+'\n')

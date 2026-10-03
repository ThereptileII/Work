"""Exact historical chart comparison; one predeclared owned toolbar exception."""
import argparse,collections,hashlib,json
from pathlib import Path
from PIL import Image,ImageChops,ImageDraw
p=argparse.ArgumentParser();p.add_argument('current',type=Path);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--dusk-baseline',type=Path,required=True);a=p.parse_args()
report=json.loads((a.current/'report.json').read_text());old_report=json.loads((a.baseline/'report.json').read_text())
assert report['status']=='passed' and report['clean_exit'] and len(report['captures'])==8
assert old_report['commit']=='5c05eb55c15b67d4014df55896d95452e9cc9d64'
assert all(x['persisted_nSymbolStyle']==76 for x in old_report['symbol_table_receipts'])
assert report['chart_source_files']==old_report['chart_source_files'],'Historical public ENC bytes differ'
dusk_report=json.loads((a.dusk_baseline/'report.json').read_text())
assert dusk_report['commit']=='9632421f701c5ec74d7a1360bc713c0034faf9af' and dusk_report['status']=='passed' and dusk_report['clean_exit']
assert dusk_report['chart_source_files']==report['chart_source_files']
chart=(80,68,1094,634);toolbar=(883,545,1072,597);wordmark=(8,8,174,58)
mask=Image.new('L',(1280,800));ImageDraw.Draw(mask).rectangle((883,545,1071,596),fill=255);mask.save(a.current/'toolbar-only-mask.png')
receipt={'historical_commit':old_report['commit'],'source_commit':report['commit'],'chart_roi':chart,'wordmark_roi':wordmark,'only_exclusion':{'Dusk historical963 only':toolbar,'Day/Night/Day-return historical5c05':None},'outside_roi':'Dynamic clock/input ages/caption are retained in whole images but were never part of the historical chart/wordmark equality oracle.','captures':[],'status':'running'}
try:
 for capture in report['captures']:
  name=capture['name'];is_dusk=name.endswith('-Dusk');base=a.dusk_baseline if is_dusk else a.baseline;reference=dusk_report if is_dusk else old_report
  old=Image.open(base/(name+'.png')).convert('RGB');new=Image.open(a.current/(name+'.png')).convert('RGB')
  assert old.size==new.size==(1280,800)
  assert hashlib.sha256((base/(name+'.png')).read_bytes()).hexdigest()==next(c['image_sha256'] for c in reference['captures'] if c['name']==name)
  bounds=next(x['bounds'] for x in report['toolbar_windows'] if x['name']==name);assert tuple(bounds)==toolbar
  diff=ImageChops.difference(old,new);exposed=diff.copy()
  if is_dusk:exposed.paste((0,0,0),toolbar)
  changed=sum(pixel!=(0,0,0) for pixel in exposed.crop(chart).getdata())
  entry={'name':name,'historical_commit':reference['commit'],'comparison_exclusion':toolbar if is_dusk else None,'historical_image_sha256':hashlib.sha256((base/(name+'.png')).read_bytes()).hexdigest(),'changed_chart_pixels_after_declared_exclusion':changed,'wordmark_identical':not diff.crop(wordmark).getbbox()}
  receipt['captures'].append(entry)
  assert capture['wordmark']['exact_native_component_match'] is True,'124-DIP component proof missing: '+name
  if name.startswith('Standard-'):assert changed==0,'Standard chart changed against exact reference: '+name+' pixels='+str(changed)
  if name.endswith('-return'):
   first=Image.open(a.current/(name.replace('-return','')+'.png')).convert('RGB')
   assert not ImageChops.difference(first,new).crop(chart).getbbox(),'Hot Day return differs: '+name
   assert not ImageChops.difference(first,new).crop((0,0,179,68)).getbbox(),'Hot Day return wordmark differs: '+name
 receipt['status']='passed'
except Exception as e:
 receipt['status']='failed';receipt['failure']=str(e);raise
finally:(a.current/'historical-comparison.json').write_text(json.dumps(receipt,indent=2)+'\n')
print('Exact historical Standard and hot-return comparison passed:',a.current)

"""Exact historical chart comparison; one predeclared owned toolbar exception."""
import argparse,collections,hashlib,json
from pathlib import Path
from PIL import Image,ImageChops,ImageDraw
p=argparse.ArgumentParser();p.add_argument('current',type=Path);p.add_argument('--baseline',type=Path,required=True);a=p.parse_args()
report=json.loads((a.current/'report.json').read_text());old_report=json.loads((a.baseline/'report.json').read_text())
assert report['status']=='passed' and report['clean_exit'] and len(report['captures'])==8
assert old_report['commit']=='78eccb8b7f21b260ded57d3ba763f884d60c8180'
assert report['chart_source_files']==old_report['chart_source_files'],'Historical public ENC bytes differ'
chart=(80,68,1094,634);toolbar=(883,545,1072,597);wordmark=(8,8,174,58)
mask=Image.new('L',(1280,800));ImageDraw.Draw(mask).rectangle((883,545,1071,596),fill=255);mask.save(a.current/'toolbar-only-mask.png')
receipt={'historical_commit':old_report['commit'],'source_commit':report['commit'],'chart_roi':chart,'wordmark_roi':wordmark,'only_exclusion':toolbar,'outside_roi':'Dynamic clock/input ages/caption are retained in whole images but were never part of the historical chart/wordmark equality oracle.','captures':[],'status':'running'}
try:
 for capture in report['captures']:
  name=capture['name'];old=Image.open(a.baseline/(name+'.png')).convert('RGB');new=Image.open(a.current/(name+'.png')).convert('RGB')
  assert old.size==new.size==(1280,800)
  assert hashlib.sha256((a.baseline/(name+'.png')).read_bytes()).hexdigest()==next(c['image_sha256'] for c in old_report['captures'] if c['name']==name)
  bounds=next(x['bounds'] for x in report['toolbar_windows'] if x['name']==name);assert tuple(bounds)==toolbar
  diff=ImageChops.difference(old,new);exposed=diff.copy();exposed.paste((0,0,0),toolbar)
  changed=sum(pixel!=(0,0,0) for pixel in exposed.crop(chart).getdata())
  entry={'name':name,'historical_image_sha256':hashlib.sha256((a.baseline/(name+'.png')).read_bytes()).hexdigest(),'changed_chart_pixels_outside_toolbar':changed,'wordmark_identical':not diff.crop(wordmark).getbbox()}
  receipt['captures'].append(entry)
  assert capture['wordmark']['exact_native_component_match'] is True,'124-DIP component proof missing: '+name
  if name.startswith('Standard-'):assert changed==0,'Standard chart changed outside exact toolbar: '+name+' pixels='+str(changed)
  if name.endswith('-return'):
   first=Image.open(a.current/(name.replace('-return','')+'.png')).convert('RGB')
   assert not ImageChops.difference(first,new).crop(chart).getbbox(),'Hot Day return differs: '+name
   assert not ImageChops.difference(first,new).crop((0,0,179,68)).getbbox(),'Hot Day return wordmark differs: '+name
  if name=='SKAGER-Day':
   # Fixed regions selected from the historical chart before this new run.
   # These show actual ferry/cable and monochrome chart marks; no pixel search
   # chooses or moves a region after inspecting a new paint result.
   regions={'ferry_repeat':(104,168,130,185),'plain_cable_boundary':(90,236,230,245),'wreck_mark':(742,415,775,439),'east_tower_mark':(1049,279,1068,308)}
   entry['source_based_ink_samples']={}
   for label,box in regions.items():
    left=old.crop(box);right=new.crop(box);pair=Image.new('RGB',(left.width*2,left.height));pair.paste(left,(0,0));pair.paste(right,(left.width,0));pair.resize((pair.width*6,pair.height*6),Image.Resampling.NEAREST).save(a.current/(label+'-before-after.png'))
    pairs=collections.Counter(zip(left.getdata(),right.getdata()))
    entry['source_based_ink_samples'][label]={'bounds':box,'pixel_transitions':[{'before':before,'after':after,'count':count} for (before,after),count in pairs.most_common() if before!=after],'unchanged_pixels':sum(count for (before,after),count in pairs.items() if before==after)}
 receipt['status']='passed'
except Exception as e:
 receipt['status']='failed';receipt['failure']=str(e);raise
finally:(a.current/'historical-comparison.json').write_text(json.dumps(receipt,indent=2)+'\n')
print('Exact historical Standard and hot-return comparison passed:',a.current)

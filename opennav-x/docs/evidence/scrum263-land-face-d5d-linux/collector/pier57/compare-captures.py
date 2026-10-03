"""Exact full-chart comparison of final font correction against sealed9dee."""
import argparse,hashlib,json
from pathlib import Path
from PIL import Image,ImageChops
p=argparse.ArgumentParser();p.add_argument('current',type=Path);p.add_argument('--baseline',type=Path,required=True);a=p.parse_args()
r=json.loads((a.current/'report.json').read_text());old=json.loads((a.baseline/'report.json').read_text())
assert r['status']=='passed' and r['clean_exit'] and len(r['captures'])==1
assert r['commit']=='d5d71356d806ea8c3518644d10728a24f1334d1d'
assert old['commit']=='9dee9b148f4d6ebdd20bb4c49229fe19340df209' and old['status']=='passed' and old['clean_exit']
assert r['chart_source_files']==old['chart_source_files']
assert all(x['persisted_nSymbolStyle']==76 for x in old['symbol_table_receipts'])
chart=(80,68,1094,634);receipt={'status':'running','source_commit':r['commit'],'historical_commit':old['commit'],'chart_roi':chart,'exclusions':[],'captures':[]}
try:
 for capture in r['captures']:
  name=capture['name'];assert name == 'SKAGER-Day'
  oldpath=a.baseline/(name+'.png');newpath=a.current/(name+'.png')
  for path,report in ((oldpath,old),(newpath,r)):
   assert hashlib.sha256(path.read_bytes()).hexdigest()==next(x['image_sha256'] for x in report['captures'] if x['name']==name)
  before=Image.open(oldpath).convert('RGB');after=Image.open(newpath).convert('RGB');assert before.size==after.size==(1280,800)
  diff=ImageChops.difference(before,after);bounds=diff.crop(chart).getbbox()
  receipt['captures'].append({'name':name,'wholeChartDifferenceBounds':bounds,'historicalImageSha256':hashlib.sha256(oldpath.read_bytes()).hexdigest()})
  assert bounds is None,'Unexpected full-chart font correction difference: '+name
  assert not diff.crop((0,0,179,68)).getbbox(),'Header identity changed'
  assert capture['wordmark']['exact_native_component_match']
 receipt['status']='passed'
except Exception as error:
 receipt['status']='failed';receipt['failure']=str(error);raise
finally:(a.current/'historical-comparison.json').write_text(json.dumps(receipt,indent=2)+'\n')
print('Entire SKAGER Day chart are identical to sealed9dee; no exclusions')

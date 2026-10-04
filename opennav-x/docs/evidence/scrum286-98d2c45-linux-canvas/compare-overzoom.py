from pathlib import Path
import json,hashlib
from PIL import Image,ImageChops
here=Path(__file__).resolve().parent
old=here.parent/'circle-fc6348a/iho/output';new=here/'iho/output';rows=[]
for renderer in ('software','opengl'):
 for mode,suffix in [('SKAGER','r2'),('Standard','standard-control')]:
  before=old/f'capture-fc6348a-s64-light-fog-{renderer}-{suffix}'
  after=new/f'capture-98d2c45-s64-light-fog-{renderer}-{suffix}'
  br=json.loads((before/'report.json').read_text());ar=json.loads((after/'report.json').read_text())
  assert br['status']==ar['status']=='passed' and br['clean_exit'] and ar['clean_exit']
  assert len(br['captures'])==len(ar['captures'])==4
  for b,a in zip(br['captures'],ar['captures']):
   assert b['name']==a['name'] and b['chart']==a['chart']
   bp=before/(b['name']+'.png');ap=after/(a['name']+'.png')
   bi=Image.open(bp).convert('RGB');ai=Image.open(ap).convert('RGB')
   region=(80,68,1094,634);diff=ImageChops.difference(bi.crop(region),ai.crop(region))
   bbox=diff.getbbox();count=sum(p!=(0,0,0) for p in diff.getdata())
   row={'renderer':renderer,'mode':mode,'image':a['name'],'changedPixels':count,'chartRelativeBbox':bbox,'beforeSha256':hashlib.sha256(bp.read_bytes()).hexdigest(),'afterSha256':hashlib.sha256(ap.read_bytes()).hexdigest(),'wholeChartComparison':True,'mask':None}
   if mode=='Standard':assert bbox is None,'Standard chart changed'
   else:
    assert bbox is not None,'Expected overzoom correction absent'
    # Upstream warning starts at the top-left toolbar offset. Inspect all changed
    # pixels and fail if any extend beyond the old embossed warning envelope.
    assert bbox[0]>=0 and bbox[1]>=0 and bbox[2]<=350 and bbox[3]<=80,'Unexpected changes outside overzoom area'
    bounds=(80,68,430,148);row['comparisonCrop']=bounds
    panel=Image.new('RGB',(700,80));panel.paste(bi.crop(bounds),(0,0));panel.paste(ai.crop(bounds),(350,0))
    dest=here/'comparison'/f'{renderer}-{a["name"]}.png';dest.parent.mkdir(exist_ok=True);panel.save(dest)
   rows.append(row)
(here/'comparison.json').write_text(json.dumps({'sourceBefore':'fc6348a6df8374878802b3e051a3670d83ed9a56','sourceAfter':'98d2c459ae0120070a2fa59b87dfa1c004e19394','comparisons':rows},indent=2)+'\n')
print('16 whole-chart comparisons: Standard unchanged; SKAGER changes bounded to top-left overzoom warning')

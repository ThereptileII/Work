from pathlib import Path
import json,hashlib,math
from PIL import Image,ImageChops
here=Path(__file__).resolve().parent
old=here.parent/'combined-symbols-45d73e8/iho/output';new=here/'iho/output';rows=[]
for renderer in ('software','opengl'):
 for mode,suffix in [('SKAGER','r2'),('Standard','standard-control')]:
  before=old/f'capture-45d73e8-s64-light-fog-{renderer}-{suffix}'
  after=new/f'capture-fc6348a-s64-light-fog-{renderer}-{suffix}'
  br=json.loads((before/'report.json').read_text());ar=json.loads((after/'report.json').read_text())
  assert br['status']==ar['status']=='passed' and br['clean_exit'] and ar['clean_exit']
  assert len(br['captures'])==len(ar['captures'])==4
  for b,a in zip(br['captures'],ar['captures']):
   assert b['name']==a['name'] and b['chart']==a['chart']
   bp=before/(b['name']+'.png');ap=after/(a['name']+'.png');bi=Image.open(bp).convert('RGB');ai=Image.open(ap).convert('RGB')
   region=(80,68,1094,634);diff=ImageChops.difference(bi.crop(region),ai.crop(region));bbox=diff.getbbox();count=sum(p!=(0,0,0) for p in diff.getdata())
   row={'renderer':renderer,'mode':mode,'image':a['name'],'changedPixels':count,'chartRelativeBbox':bbox,'beforeSha256':hashlib.sha256(bp.read_bytes()).hexdigest(),'afterSha256':hashlib.sha256(ap.read_bytes()).hexdigest(),'wholeChartComparison':True,'mask':None}
   if mode=='Standard':assert bbox is None,'Standard chart changed'
   else:
    assert bbox is not None,'Expected corrected ring paint is absent'
    lights=[x for x in a['trace']['raster_draws'] if x['class']=='LIGHTS'];assert lights
    cx,cy=lights[-1]['pixel_x'],lights[-1]['pixel_y'];dist=[]
    for y in range(diff.height):
     for x in range(diff.width):
      if diff.getpixel((x,y))!=(0,0,0):dist.append(math.hypot(x-cx,y-cy))
    assert max(dist)<75,'Unexpected changed pixels outside the original light ring envelope'
    row.update(center=[cx,cy],minimumChangedRadius=min(dist),maximumChangedRadius=max(dist))
    bounds=(80+cx-76,68+cy-76,80+cx+77,68+cy+77);row['comparisonCrop']=bounds
    panel=Image.new('RGB',(306,153));panel.paste(bi.crop(bounds),(0,0));panel.paste(ai.crop(bounds),(153,0));out=here/'comparison'/f'{renderer}-{a["name"]}.png';out.parent.mkdir(exist_ok=True);panel.save(out)
   rows.append(row)
(here/'comparison.json').write_text(json.dumps({'sourceBefore':'45d73e831409d602e8205f32fcba08947ea370bc','sourceAfter':'fc6348a6df8374878802b3e051a3670d83ed9a56','comparisons':rows},indent=2)+'\n')
print('16 whole-chart comparisons: Standard unchanged; SKAGER changes bounded to original light-ring envelope')

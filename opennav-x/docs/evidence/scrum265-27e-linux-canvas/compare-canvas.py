"""Offline actual-canvas delta check; original Day-return assertion stays unmasked."""
from pathlib import Path
import argparse,hashlib,json
from PIL import Image,ImageChops,ImageDraw
p=argparse.ArgumentParser();p.add_argument('renderer',choices=['software','opengl']);a=p.parse_args()
root=Path(__file__).resolve().parents[3];here=Path(__file__).resolve().parent
current=here/'iho/output'/('capture-27e93e4-s64-light-fog-'+a.renderer)
prior=(root/'docs/evidence/scrum268-5bb-linux-canvas/iho/output/capture-5bb7e05-s64-light-fog-opengl' if a.renderer=='opengl' else root/'docs/evidence/scrum275276-326-linux-canvas/output/capture-326daf7-s64-light-fog-software')
report=json.loads((current/'report.json').read_text());old=json.loads((prior/'report.json').read_text())
assert report['status']=='passed' and report['clean_exit'] and len(report['captures'])==4
assert old['status']=='passed'
records=[]
for c in report['captures']:
 name=c['name'];before=Image.open(prior/(name+'.png')).convert('RGB');after=Image.open(current/(name+'.png')).convert('RGB')
 assert before.size==after.size==(1280,800)
 diagnostics=json.loads((current/(name+'.json')).read_text());reg=diagnostics['runtime']['display']['chart_region'];x,y,w,h=[reg[k] for k in ['x','y','width','height']];chart=(x,y,x+w,y+h)
 assert chart==(80,68,1094,634)
 buildings=[f for f in c['source_feature_crops'] if f['class']=='BUISGL'];assert {f['source_rcid'] for f in buildings}=={35,36}
 diff=ImageChops.difference(before,after);nonchart=diff.copy();ImageDraw.Draw(nonchart).rectangle((x,y,x+w-1,y+h-1),fill=(0,0,0));chart_diff=diff.crop(chart);full_count=sum(v!=(0,0,0) for v in chart_diff.getdata());outside=chart_diff.copy();points=[]
 for f in buildings:
  draw=f['draw'];assert draw['symbol']=='XNBLDG01' and draw['lookup_rcid']==31143
  px,py=x+draw['pixel_x'],y+draw['pixel_y'];rect=(px-4,py-4,px+5,py+5)
  changed=sum(v!=(0,0,0) for v in diff.crop(rect).getdata());assert changed>0
  local=(rect[0]-x,rect[1]-y,rect[2]-x,rect[3]-y);ImageDraw.Draw(outside).rectangle((local[0],local[1],local[2]-1,local[3]-1),fill=(0,0,0))
  bounds=(px-24,py-24,px+24,py+24)
  panels=Image.new('RGB',(384,215),'white');panels.paste(before.crop(bounds).resize((192,192),Image.Resampling.NEAREST),(0,23));panels.paste(after.crop(bounds).resize((192,192),Image.Resampling.NEAREST),(192,23));label=ImageDraw.Draw(panels);label.text((4,4),'Before: '+a.renderer,fill='black');label.text((196,4),'After: BUISGL'+str(f['source_rcid']),fill='black')
  crop_name=name+'-BUISGL'+str(f['source_rcid'])+'-before-after.png';panels.save(current/crop_name)
  points.append({'sourceRcid':f['source_rcid'],'actualRasterDraw':draw,'exactTileBounds':rect,'changedPixels':changed,'contextCropBounds':bounds,'comparisonImage':crop_name})
 outside_count=sum(v!=(0,0,0) for v in outside.getdata())
 records.append({'name':name,'prior':str(prior/(name+'.png')),'priorSha256':hashlib.sha256((prior/(name+'.png')).read_bytes()).hexdigest(),'currentSha256':hashlib.sha256((current/(name+'.png')).read_bytes()).hexdigest(),'chartBounds':chart,'outsideChartChangedPixels':sum(v!=(0,0,0) for v in nonchart.getdata()),'outsideChartDifferenceBounds':nonchart.getbbox(),'changedChartPixels':full_count,'outsideBuildingTileChangedPixels':outside_count,'points':points})
proof={'renderer':a.renderer,'source':report['commit'],'priorSource':old['commit'],'records':records,'passed':all(r['outsideBuildingTileChangedPixels']==0 for r in records),'limits':'Before/after change confinement only. Original whole-chart Day-return gate remains unmasked. No CONVIS1 point in source viewport; not private/native/boat acceptance.'}
(here/('comparison-'+a.renderer+'.json')).write_text(json.dumps(proof,indent=2)+'\n')
assert proof['passed'],'Pixels outside the two actual9x9 building tiles changed'
print(a.renderer, 'all4 charts differ only within actual BUISGL35/36 tiles; whole-chart returns remain exact')

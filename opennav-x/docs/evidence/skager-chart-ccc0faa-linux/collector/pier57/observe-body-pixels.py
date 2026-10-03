"""Record final chart differences against exact prior originals; no image edits."""
import argparse,collections,hashlib,json
from pathlib import Path
from PIL import Image,ImageChops
p=argparse.ArgumentParser();p.add_argument('--captures',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
# Real source geometry gives centers ~(583,342),(591,360). The original body
# is13x13,pivot6,6; new alpha relative bounds(-3,-7) to(+3,+10). Envelopes retain
# a conservative2px rounding margin. These are geometry, not color-search masks.
boxes=[(575,333,592,355),(583,351,600,373)];chart=(80,68,1094,634);receipt=[]
for renderer in ('software','opengl'):
 current=a.captures/renderer
 report=json.loads((current/'report.json').read_text());old_report=json.loads((a.baseline/renderer/'report.json').read_text())
 assert report['commit']=='ccc0faad089e89a5b3a0b1f2994f4fa4ee18053d'
 assert old_report['commit']=='5c05eb55c15b67d4014df55896d95452e9cc9d64'
 for theme in ('Day','Night'):
  name='SKAGER-'+theme;oldpath=a.baseline/renderer/(name+'.png');newpath=current/(name+'.png')
  for path,r in ((oldpath,old_report),(newpath,report)):
   assert hashlib.sha256(path.read_bytes()).hexdigest()==next(x['image_sha256'] for x in r['captures'] if x['name']==name)
  old=Image.open(oldpath).convert('RGB');new=Image.open(newpath).convert('RGB');diff=ImageChops.difference(old,new);remaining=diff.copy()
  for box in boxes:remaining.paste((0,0,0),box)
  b=diff.crop(chart).getbbox();bounds=[b[0]+chart[0],b[1]+chart[1],b[2]+chart[0],b[3]+chart[1]] if b else None
  entry={'renderer':renderer,'theme':theme,'chartDifferenceBoundsScreen':bounds,'changedPixels':sum(x!=(0,0,0) for x in diff.crop(chart).getdata()),'outsideSourceBodyEnvelopesUnchanged':not remaining.crop(chart).getbbox(),'sourceBodyEnvelopes':boxes,'scope':'Exposed light/name/hatch pixels outside body envelopes stay exact; occluded co-location pixels cannot independently prove light painter identity. No actual per-object rule trace in this run.','newCoreColors':[{str(k):v for k,v in collections.Counter(new.crop(box).getdata()).most_common(12)} for box in boxes]}
  receipt.append(entry)
  assert entry['outsideSourceBodyEnvelopesUnchanged'],entry
# No broad exclusion applies to Standard or Day-return; separate strict collector
# comparisons require entire chart equality. Dusk has its explicit963 reference.
a.output.write_text(json.dumps(receipt,indent=2)+'\n')
print('All Day/Night changes confined to two source-position body envelopes')

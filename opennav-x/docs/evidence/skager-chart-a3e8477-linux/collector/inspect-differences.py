"""Read-only complete image accounting after fail-fast comparison; never overrides its failure."""
from pathlib import Path
from collections import Counter
from PIL import Image,ImageChops
import json
root=Path(__file__).resolve().parent
baseline=Path('/home/standard/Projects/X-nav-worktrees/scrum259-linux-6dafd29/docs/evidence/skager-product-fidelity-78eccb8-linux')
roi=(80,68,1094,634);toolbar=(883,545,1072,597);result={}
for renderer,phase in [('software','a3e8477-software-r1'),('opengl','a3e8477-opengl')]:
 current=root/'output'/('capture-'+phase);report=json.loads((current/'report.json').read_text());rows=[]
 for c in report['captures']:
  name=c['name'];old=Image.open(baseline/renderer/(name+'.png')).convert('RGB');new=Image.open(current/(name+'.png')).convert('RGB');diff=ImageChops.difference(old,new);diff.paste((0,0,0),toolbar)
  entry={'name':name,'historical_chart_changes_outside_toolbar':sum(v!=(0,0,0) for v in diff.crop(roi).getdata()),'historical_wordmark_changes':sum(v!=(0,0,0) for v in ImageChops.difference(old,new).crop((8,8,174,58)).getdata())}
  if name.endswith('-return'):
   first=Image.open(current/(name.replace('-return','')+'.png')).convert('RGB');hot=ImageChops.difference(first,new).crop(roi);box=hot.getbbox()
   entry['hot_return']={'changed_pixels':sum(v!=(0,0,0) for v in hot.getdata()),'screen_bounds':[box[0]+80,box[1]+68,box[2]+80,box[3]+68] if box else None,'changed_transitions':[{'before':a,'after':b,'count':n} for (a,b),n in Counter(zip(first.crop(roi).getdata(),new.crop(roi).getdata())).most_common() if a!=b]}
  rows.append(entry)
 result[renderer]=rows
(root/'output/complete-image-accounting.json').write_text(json.dumps({'note':'Complete read-only inspection, including images after the preserved GL fail-fast assertion. It does not supersede or mark that assertion passed.','chart_roi':roi,'only_historical_exclusion':toolbar,'results':result},indent=2)+'\n')

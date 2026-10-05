"""Read-only independent evidence/source/pixel audit; does not launch application."""
import hashlib,json,pathlib,subprocess,sys
from PIL import Image,ImageChops
here=pathlib.Path(__file__).resolve().parent
cache=pathlib.Path(sys.argv[1])
commit='78eccb8b7f21b260ded57d3ba763f884d60c8180'
exe='a58256d8c824a43d7d74b90deb6ad6697a2ba1dabbf47b13c4e8057dd4738ffb'
manifest='03533971dfba1bb3cb2cba6ab52b1528a56b4f27c8b4fc118b4b503c25b4bdde'
sha=lambda b:hashlib.sha256(b).hexdigest()
assert sha((cache/'install/bin/opencpn').read_bytes())==exe
res=cache/'install/share/opencpn/opennav/chart-style/v1'
assert sha((res/'manifest.json').read_bytes())==manifest
result={'commit':commit,'binary_sha256':exe,'manifest_sha256':manifest,'captures':[],'day_return_exact_chart_and_wordmark':[],'all_exact_f097_chart_and_wordmark':[]}
for renderer in ('software','opengl'):
 folder=here/renderer;report=json.loads((folder/'report.json').read_text())
 assert report['status']=='passed' and report['clean_exit'] is True
 assert report['commit']==commit and report['binary_sha256']==exe and report['manifest_sha256']==manifest
 assert len(report['captures'])==8
 baseline=cache/('capture-1356fd1-'+('sw' if renderer=='software' else 'gl'))
 assert sha((baseline/'report.json').read_bytes())==report['baseline_report_sha256']
 old=json.loads((baseline/'report.json').read_text())
 assert old['commit']=='1356fd1603aacbea04d7081d16331e9a181180bb'
 final_baseline=here.parent/'skager-product-fidelity-f0976cc-linux'/renderer
 prior=json.loads((final_baseline/'report.json').read_text())
 assert prior['commit']=='f0976cc65ea63d3ed6f60ac53ec38a856d11b66b' and prior['status']=='passed'
 for capture in prior['captures']:
  name=capture['name']
  assert sha((final_baseline/(name+'.png')).read_bytes())==capture['image_sha256']
  a=Image.open(folder/(name+'.png')).convert('RGB')
  b=Image.open(final_baseline/(name+'.png')).convert('RGB')
  for box in ((80,68,1094,634),(8,8,174,58)):
   assert ImageChops.difference(a.crop(box),b.crop(box)).getbbox() is None
  result['all_exact_f097_chart_and_wordmark'].append(renderer+'/'+name)
 for path,digest in {**report['source_evidence'],**report['additional_source_evidence']}.items():
  source=subprocess.check_output(['git','-C',str(cache/'app'),'show',commit+':'+path])
  assert sha(source)==digest,path
 for name,identity in report['resource_files'].items():
  raw=(res/name).read_bytes();assert len(raw)==identity['bytes'] and sha(raw)==identity['sha256']
 for capture in report['captures']:
  name=capture['name'];raw=(folder/(name+'.png')).read_bytes()
  assert sha(raw)==capture['image_sha256']
  assert capture['build_commit']==commit
  assert capture['chart']['opengl_enabled']==(renderer=='opengl')
  assert capture['chart']['quilt_members'][0]['file']=='US5SEAFL.000'
  assert capture['edge_negative_rejected'] is True
  assert capture['repeated_edge_pixels']<capture['edge_probe_pixels']==3232
  assert max(v['age_ms'] for v in capture['owned_values'].values())<2000
  im=Image.open(folder/(name+'.png')).convert('RGB');assert im.size==(1280,800)
  theme=name.split('-')[1];bg={'Day':(21,35,38),'Dusk':(29,40,46),'Night':(12,17,21)}[theme]
  assert all(im.getpixel((x,y))==bg for x in range(20,160) for y in (11,40,41,42,56))
  if name=='SKAGER-Night':
   px=list(im.crop((80,68,1094,634)).getdata())
   assert px.count((14,23,28))>200000 and px.count((29,41,37))>50000
  result['captures'].append({'renderer':renderer,'name':name,'sha256':sha(raw),'water_pixels':capture['water_pixels'],'land_or_built_area_pixels':capture['land_or_built_area_pixels']})
 for style in ('SKAGER','Standard'):
  day=Image.open(folder/(style+'-Day.png')).convert('RGB');back=Image.open(folder/(style+'-Day-return.png')).convert('RGB')
  for box in ((80,68,1094,634),(8,8,174,58)):
   assert ImageChops.difference(day.crop(box),back.crop(box)).getbbox() is None
  result['day_return_exact_chart_and_wordmark'].append(renderer+'/'+style)
result['status']='passed';result['visually_inspected_original_images']=16
print(json.dumps(result,indent=2))

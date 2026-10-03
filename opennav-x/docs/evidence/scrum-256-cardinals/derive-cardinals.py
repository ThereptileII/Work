from pathlib import Path
import subprocess,json,re,hashlib
from PIL import Image
root=Path.cwd();assets=root/'resources/chart-style/v1/cardinals'
art=root/'docs/design/prototype/src/chart-marker-art.js';guide=root/'docs/design/prototype/src/seamarks.json'
js='const seamarkGuide='+guide.read_text()+';\n'+art.read_text()+"\nconsole.log(JSON.stringify(Object.fromEntries(['BOYCAR01','BOYCAR02','BOYCAR03','BOYCAR04'].map(code=>[code,chartMarkerArt({id:'point:'+code})]))));"
paths=json.loads(subprocess.check_output(['node','-e',js],text=True))
tokens=json.loads((root/'docs/design/prototype-tokens.json').read_text())
css=(root/'docs/design/prototype/src/chart-symbols.css').read_text()
prov={'issue':'SCRUM-256','prototypeScale':27/32,'strokeWidth':1.3,'nightBrightness':.78,'rasterizer':subprocess.check_output(['rsvg-convert','--version'],text=True).strip(),'symbols':{},'sources':{},'colors':{}}
for i,(name,path) in enumerate(paths.items()):
 item={'effectiveRcid':1270+i,'category':i+1,'tile':[116+32*i,1160,24,28],'pivot':[12,14],'themes':{}}
 for table,theme in [('DAY_BRIGHT','day'),('DUSK','dusk'),('NIGHT','night')]:
  colors={role:re.findall('--mark-'+role+':(#[a-f0-9]+)',css)[['day','dusk','night'].index(theme)] for role in ['black','yellow']};colors['water']=tokens['themes'][theme]['--water']
  # The whole chart, including alpha-composited SVG, is brightness(.78) at Night.
  # Apply the linear-in-sRGB scalar to all input RGB roles once; alpha is unchanged.
  rgb={k:[int(v[n:n+2],16) for n in (1,3,5)] for k,v in colors.items()}
  if theme=='night':rgb={k:[round(c*.78) for c in v] for k,v in rgb.items()}
  prov['colors'][table]=rgb
  rendered=path
  for role,c in rgb.items():rendered=rendered.replace('var(--'+('water' if role=='water' else 'mark-'+role)+')','#'+''.join(f'{v:02x}' for v in c))
  svg='<svg xmlns="http://www.w3.org/2000/svg" width="24" height="28" viewBox="0 0 24 28"><g transform="translate(12 14) scale(0.84375)" fill="none" stroke-width="1.3" stroke-linecap="round" stroke-linejoin="round" shape-rendering="geometricPrecision">'+rendered+'</g></svg>\n'
  f=assets/f'{name}-{table}.svg';f.write_text(svg)
  png=root/'.local'/f'{name}-{table}.png';subprocess.run(['rsvg-convert',str(f),'-o',str(png)],check=True)
  im=Image.open(png).convert('RGBA');raw=im.tobytes();data={'width':24,'height':28,'rows':[raw[y*96:(y+1)*96].hex() for y in range(28)]}
  (assets/f'{name}-{table}-rgba.json').write_text(json.dumps(data,indent=2)+'\n')
  item['themes'][table]={'rgbaSha256':hashlib.sha256(raw).hexdigest(),'changedPixels':sum(a>0 for a in raw[3::4])}
 prov['symbols'][name]=item
for f in [art,guide,root/'docs/design/prototype/src/chart-symbols.css',root/'docs/design/prototype/src/style.css',root/'docs/design/prototype-tokens.json',*sorted(assets.glob('*.svg')),*sorted(assets.glob('*-rgba.json'))]:
 prov['sources'][str(f.relative_to(root))]=hashlib.sha256(f.read_bytes()).hexdigest()
(assets/'provenance.json').write_text(json.dumps(prov,indent=2)+'\n')
print(json.dumps({k:v['themes'] for k,v in prov['symbols'].items()},indent=2))

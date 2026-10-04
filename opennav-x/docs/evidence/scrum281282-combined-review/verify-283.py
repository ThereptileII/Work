from pathlib import Path
import json,hashlib,subprocess
r=Path.cwd();prior=r.parent/'scrum283-private-hazard-association/.local/source';out={}
for p in ['src/eSENCChart.cpp','libs/s52plib/src/s52cnsy.cpp','libs/s52plib/src/s52s57.h']:
 b=(r/'.local/private'/p).read_bytes();assert b==(prior/p).read_bytes(),p;out[p]=hashlib.sha256(b).hexdigest()
api=r.parent/'scrum259-ocharts-port/.local/plugin-source/opencpn-libs/api-17';out['api17ReadOnlyInputs']={str(p.relative_to(api)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(api.rglob('*')) if p.is_file()}
changes=subprocess.check_output(['git','diff','--name-only','af69c5e','HEAD'],text=True).splitlines();assert changes==['docs/upstream-patches.md'],changes
out['sameProductBytesAs']='af69c5e';out['checkoutDifferenceFromRoot']={'docs/upstream-patches.md':'parent documentation-only append not cherry-picked'}
(r/'.local/283-equality.json').write_text(json.dumps(out,indent=2)+'\n')

from pathlib import Path
import sys
sys.path[:0]=['tools','tests']
from chart_hazard_resources_tests import verify_hazards,restore_hazards
from chart_raster_ink import decode
n=0
def check(v):
 global n
 n+=1
 assert v,n
source=Path('/home/standard/Projects/X-nav-worktrees/skager-product-fidelity/build/integration-source/data/s57data');out=Path('.local/generated')
verify_hazards(source,out,check)
for f in ('rastersymbols-day.png','rastersymbols-dusk.png','rastersymbols-dark.png'):
 c,b=decode((Path('.local/before')/f).read_bytes());d,a=decode((out/f).read_bytes());restore_hazards(b,a);check(a==b);check([(x,y) for x,y in c if x!=b'IDAT']==[(x,y) for x,y in d if x!=b'IDAT'])
print(n,'focused resource checks passed including 8 rejection controls and full prior-atlas inverse')
Path('.local/resource-checks.txt').write_text(str(n)+' focused checks passed; eight rejection controls; full prior atlas inverse in all three themes\n')

"""Offline geometry sampler proof; never edits the retained source screenshots."""
from pathlib import Path
import json,sys
from PIL import Image
root=Path(__file__).resolve().parent
sys.path.insert(0,str(root/'collector'))
from route_sample_geometry import exposed_route_samples
report=json.loads((root/'capture-78eccb8-software-1/route-input-results.json').read_text())
points=next(c for c in report['route_contract']['checks'] if c['check']=='middle point real upstream progress')['route_pixels']
image=Image.open(root/'capture-78eccb8-opengl-1/route-opengl-01-active-route.png').convert('RGB')
ink=(38,124,118)
def check(image,samples):
    for s in samples:
        hits=sum(image.getpixel((x,y))==ink for y in range(s['y']-3,s['y']+4) for x in range(s['x']-3,s['x']+4))
        assert hits>=2,(s,hits) # Same exact ink/count predicate as the collector.
negative=0
for a,b in zip(points,points[1:]):
    samples,geometry=exposed_route_samples(a,b,points)
    check(image,samples)
    for sample in samples:
        erased=image.copy();x,y=sample['x'],sample['y']
        erased.paste((238,238,226),(x-3,y-3,x+4,y+4))
        try:check(erased,samples)
        except AssertionError:negative+=1
        else:raise AssertionError('An individually missing exposed stroke passed')
try:check(image,[{'x':667,'y':204}])
except AssertionError:negative+=1
else:raise AssertionError('Overlay card incorrectly counted as route')
assert negative==11
print('10 missing exposed strokes and1 covered-card sample correctly rejected; geometry-selected source samples pass')

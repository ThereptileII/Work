"""Geometry-only exposed-route sampling for the fixed three-name fixture."""
import math
CANDIDATES=(.2,.35,.5,.65,.8,.1,.9,.25,.4,.55,.7,.85)

def exposed_route_samples(a,b,points,left=0,top=0):
    assert len(points)==3,'Known three-name route fixture required'
    # Actual upstream screen projections; each real SIM n name is five UTF-16
    # units, so the reviewed prototype card is50x25. Inflate by3px probe radius
    # and1px border/AA; this sampler never inspects image colors.
    boxes=[]
    for i,p in enumerate(points):
        x,y=p['x']-left,p['y']-top;offset=-64 if i==2 else -18
        boxes.append((x+offset-4,y+14,x+offset+54,y+47))
    selected=[];excluded=[]
    for fraction in CANDIDATES:
        x=round(a['x']+(b['x']-a['x'])*fraction)-left
        y=round(a['y']+(b['y']-a['y'])*fraction)-top
        overlaps=[list(r) for r in boxes if r[0]<=x<=r[2] and r[1]<=y<=r[3]]
        if overlaps:
            excluded.append({'fraction':fraction,'x':x,'y':y,'card_probe_bounds':overlaps});continue
        if any(math.hypot(x-s['x'],y-s['y'])<12 for s in selected):continue
        selected.append({'fraction':fraction,'x':x,'y':y})
        if len(selected)==5:break
    assert len(selected)==5,'Fewer than five separated exposed route samples'
    assert max(s['fraction'] for s in selected)-min(s['fraction'] for s in selected)>=.5,'Insufficient exposed leg coverage'
    return selected,{'original_fractions':list(CANDIDATES[:5]),'selected':selected,'excluded':excluded,'all_card_probe_bounds':boxes,'minimum_pixel_separation':12}

"""Bounded paint samples around actual upstream projected points/leg."""
import math

def route_paint_probe(image,points,light):
    surface={'Day':(247,248,240),'Dusk':(36,58,64),'Night':(16,26,32)}[light]
    ink={'Day':(38,124,118),'Dusk':(176,223,200),'Night':(113,147,126)}[light]
    a,b=points[:2];ax,ay=a['x'],a['y'];bx,by=b['x'],b['y']
    dx,dy=bx-ax,by-ay;length=math.hypot(dx,dy);assert length>50
    near=lambda actual,expected,tolerance=2:all(abs(x-y)<=tolerance for x,y in zip(actual,expected))
    ring=glyph=0
    for y in range(ay-12,ay+13):
        for x in range(ax-12,ax+13):
            radius=math.hypot(x-ax,y-ay)
            if near(image.getpixel((x,y)),ink,1):
                ring+=9<=radius<=11
                # Exclude the actual leg centerline, which crosses the circle
                # in the upstream software paint order. Remaining inner ink is
                # numeral evidence; complete "01" still receives visual review.
            distance=abs(dx*(y-ay)-dy*(x-ax))/length
            pixel=image.getpixel((x,y))
            # Native text antialiasing produces partial ink coverage, unlike
            # the opaque ring. Count visible text contrast away from the leg.
            glyph+=radius<=6 and distance>2.5 and max(abs(c-f) for c,f in zip(pixel,surface))>=16
    assert ring>=15,('Numbered waypoint ring missing',light,ring)
    assert glyph>=2,('Numbered waypoint inner glyph ink missing',light,glyph)
    cx,cy=ax+.4*dx,ay+.4*dy
    background=image.getpixel((round(cx-10*dy/length),round(cy+10*dx/length)))
    expected=tuple(round(.6*f+.4*c) for f,c in zip(surface,background))
    hits=[]
    for y in range(round(cy)-12,round(cy)+13):
        for x in range(round(cx)-12,round(cx)+13):
            distance=abs(dx*(y-ay)-dy*(x-ax))/length
            if 1.8<=distance<=2.5 and near(image.getpixel((x,y)),expected):
                hits.append({'x':x,'y':y,'rgb':image.getpixel((x,y))})
    assert len(hits)>=4,('Six-pixel .6-opacity understroke missing',light,len(hits),expected)
    return {'ring_ink_pixels':ring,'inner_glyph_ink_pixels_off_leg':glyph,
            'understroke_background':background,'understroke_expected_composite':expected,
            'understroke_samples':hits,'channel_tolerance':2,
            'number_text_review':'Inspect actual numbered crop for complete 01; pixel samples do not provide OCR'}

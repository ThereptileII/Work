"""Check actual pinned OpenCPN coastline pixels across a fixed demo view.

This is a rendering regression fixture, not chart/sensor data or a replacement
renderer. Learn the two dominant interior colors from the first native Day
capture; both must remain present after restart. A water-only canvas fails.
"""
from collections import Counter


def navigation_layout(display, frame, client):
    """Verify chart-first geometry against actual native frame/client bounds.

    The immutable prototype supersedes the old 75%-height rule: its 132px
    horizon is outside the chart. Enforce the exact composition, not a looser
    chart-area percentage. OS decorations are measured separately.
    """
    def valid(rect):
        return all(isinstance(rect.get(k), int) for k in ('x', 'y', 'width', 'height')) and rect['width'] > 0 and rect['height'] > 0

    def contains(outer, inner):
        return (inner['x'] >= outer['x'] and inner['y'] >= outer['y'] and
                inner['x'] + inner['width'] <= outer['x'] + outer['width'] and
                inner['y'] + inner['height'] <= outer['y'] + outer['height'])

    def overlaps(a, b):
        return (a['x'] < b['x'] + b['width'] and b['x'] < a['x'] + a['width'] and
                a['y'] < b['y'] + b['height'] and b['y'] < a['y'] + a['height'])

    assert valid(frame) and valid(client)
    assert (frame['width'], frame['height']) == (1280, 800) or (client['width'], client['height']) == (1280, 800), 'Native frame or primary client must be exactly 1280x800'
    assert valid(client) and contains(frame, client), 'Actual client must fit its native frame'
    chart = display.get('chart_region', {})
    assert valid(chart) and contains(client, chart), 'Chart must fit the actual client'
    expected = dict(x=client['x']+80, y=client['y']+68,
                    width=client['width']-80-186,
                    height=client['height']-68-132-34)
    assert all(abs(chart[k]-expected[k]) <= 1 for k in expected), 'Chart must match the 68/80/186/132/34 prototype composition'
    rail_bounds = dict(x=client['x']+client['width']-186, y=client['y']+68,
                       width=186, height=client['height']-68-34)
    rail = display.get('rail_regions', [])
    assert len(rail) == 4 and len({r['label'] for r in rail}) == 4, 'Four distinct primary rail values required'
    for i, region in enumerate(rail):
        assert valid(region) and region.get('visible') and contains(rail_bounds, region), 'Every primary rail value must fit the 186px rail'
        assert region['height'] >= 48 and region['width'] >= 48, 'Primary rail values must remain readable'
        assert not overlaps(chart, region) and all(not overlaps(region, other) for other in rail[:i]), 'Chart and rail values must not overlap'
    controls = [c for c in display.get('interaction_controls', []) if c.get('visible')]
    # The prototype specifies these seven bounded floating controls, not a
    # general permission to cover the chart with permanent toolbars.
    right, bottom = chart['x']+chart['width'], chart['y']+chart['height']
    tools_x, tools_y = right-22-189, bottom-37-52
    floating = {
        'Measure': (tools_x+4, tools_y+4, 44, 44),
        'Waypoint': (tools_x+48, tools_y+4, 44, 44),
        '+': (tools_x+97, tools_y+4, 44, 44),
        '−': (tools_x+141, tools_y+4, 44, 44),
        'Follow boat': (chart['x']+28, bottom-37-44, 142, 44),
        # Centered below the 68x90 compass: 22px top + 90px + 10px gap.
        'Layers': (right-22-68+(68-44)//2, chart['y']+22+90+10, 44, 44),
    }
    orientation = [c for c in controls if c['label'] in ('North', 'Course', 'Head')]
    assert len(orientation) == 1, 'One chart orientation control required'
    floating[orientation[0]['label']] = (right-22-68, chart['y']+22, 68, 90)
    for label, bounds in floating.items():
        found = [c for c in controls if c['label'] == label]
        assert len(found) == 1, 'Each floating chart control must be unique and visible'
        assert all(abs(found[0][k]-v) <= 1 for k,v in zip(('x','y','width','height'),bounds)), 'Floating control differs from prototype geometry'
    for control in controls:
        assert valid(control) and contains(client, control), 'Visible control must fit the actual client'
        if control['label'] in floating:
            assert contains(chart, control), 'Floating controls must fit the chart'
        else:
            assert not overlaps(chart, control), 'Unspecified permanent controls must not cover the chart'
    footer = display.get('footer_region', {})
    expected_footer = dict(x=client['x'], y=client['y']+client['height']-34,
                           width=client['width'], height=34)
    assert valid(footer) and contains(client, footer), 'Status footer must fit the actual client'
    assert all(abs(footer[k]-v) <= 1 for k,v in expected_footer.items()), 'Status footer must match the full-width 34px prototype region'
    assert display.get('footer_middle_visible') is True, 'Primary 1280px layout must retain middle footer information'
    assert not any(c['label'] == 'System' for c in controls), 'Obsolete footer System action must not replace navigation status'
    health = [c for c in controls if c['label'] == 'Source health']
    assert len(health) == 1 and health[0].get('enabled') and contains(footer, health[0]), 'Source health must remain visible and actionable within the footer'
    for label in ('Chart', 'Settings'):
        required = [c for c in controls if c['label'] == label]
        assert len(required) == 1 and required[0].get('enabled'), 'Navigation and Settings recovery access must be uniquely visible and enabled'
        assert required[0]['x']+required[0]['width'] <= chart['x'], 'Chart and Settings entries must fit the navigation strip'
    return {'frame': frame, 'client': client, 'chart': chart,
            'chart_client_area_fraction': chart['width'] * chart['height'] / (client['width'] * client['height']),
            'primary_rail_values_visible': 4, 'prototype_floating_controls': 7,
            'footer': footer, 'recovery_access': 'Settings', 'source_health_visible': True,
            'unspecified_controls_do_not_cover_chart': True}


def functional_layout(display, frame, client):
    """Visibility, containment and non-overlap without prototype geometry."""
    def valid(rect):
        return all(isinstance(rect.get(k),int) for k in ('x','y','width','height')) and rect['width']>0 and rect['height']>0
    def contains(outer,inner):
        return (outer['x']<=inner['x'] and outer['y']<=inner['y'] and
                inner['x']+inner['width']<=outer['x']+outer['width'] and
                inner['y']+inner['height']<=outer['y']+outer['height'])
    def overlaps(a,b):
        return (a['x']<b['x']+b['width'] and b['x']<a['x']+a['width'] and
                a['y']<b['y']+b['height'] and b['y']<a['y']+a['height'])
    assert valid(frame) and valid(client) and contains(frame,client), 'Actual client must fit its native frame'
    chart=display.get('chart_region',{})
    assert valid(chart) and contains(client,chart), 'Chart must fit the actual client'
    rail=display.get('rail_regions',[])
    assert len(rail)==4 and len({r['label'] for r in rail})==4, 'Four distinct primary rail values required'
    for i,region in enumerate(rail):
        assert valid(region) and region.get('visible') and contains(client,region), 'Primary values must remain visible inside the client'
        assert region['height']>=48 and region['width']>=48, 'Primary values must remain readable'
        assert not overlaps(chart,region) and all(not overlaps(region,other) for other in rail[:i]), 'Chart and primary values must not overlap'
    controls=[c for c in display.get('interaction_controls',[]) if c.get('visible')]
    orientation=[c for c in controls if c['label'] in ('North','Course','Head')]
    assert len(orientation)==1, 'One chart orientation control required'
    floating={'Measure','Waypoint','+','−','Follow boat','Layers',orientation[0]['label']}
    for label in floating:
        assert sum(c['label']==label for c in controls)==1, 'Each chart control must be unique and visible'
    for control in controls:
        assert valid(control) and contains(client,control), 'Visible control must fit the client'
        if control['label'] in floating:
            assert contains(chart,control), 'Floating chart controls must fit the chart'
        else:
            assert not overlaps(chart,control), 'Permanent controls must not cover the chart'
    floating_controls=[c for c in controls if c['label'] in floating]
    for i,control in enumerate(floating_controls):
        assert all(not overlaps(control,other) for other in floating_controls[:i]), 'Chart controls must not obstruct one another'
    footer=display.get('footer_region',{})
    assert valid(footer) and contains(client,footer) and not overlaps(chart,footer), 'Status footer must remain visible outside the chart'
    assert display.get('footer_middle_visible') is True, 'Primary view must retain navigation footer information'
    health=[c for c in controls if c['label']=='Source health']
    assert len(health)==1 and health[0].get('enabled') and contains(footer,health[0]), 'Source health must remain visible and actionable'
    for label in ('Chart','Settings'):
        required=[c for c in controls if c['label']==label]
        assert len(required)==1 and required[0].get('enabled'), 'Navigation and Settings recovery must remain uniquely accessible'
    return {'frame':frame,'client':client,'chart':chart,'footer':footer,
            'primary_rail_values_visible':4,'source_health_visible':True,
            'recovery_access':'Settings','scope':'Functional visibility and non-overlap; no prototype geometry'}


def route_visibility(rgb, points, offset=(0,0)):
    """A continuous contrasting stroke at upstream-projected route samples.

    Compare each center patch with perpendicular flank patches, then require a
    common visible stroke ink at every sample. Never accept a uniform canvas or
    choose an expected color from the design prototype.
    """
    import math
    assert len(rgb)==1280*800*3
    common=None;samples=[]
    def patch(x,y,radius):
        assert radius<=x<1280-radius and radius<=y<800-radius, 'Projected route sample offscreen'
        return Counter(tuple(rgb[(py*1280+px)*3:(py*1280+px)*3+3])
                       for py in range(y-radius,y+radius+1) for px in range(x-radius,x+radius+1))
    for a,b in zip(points,points[1:]):
        dx,dy=b['x']-a['x'],b['y']-a['y'];length=math.hypot(dx,dy)
        assert length>0, 'Projected route segment must have a visible extent'
        nx,ny=-dy/length*10,dx/length*10
        for fraction in (.2,.35,.5,.65,.8):
            x=round(a['x']+dx*fraction)-offset[0];y=round(a['y']+dy*fraction)-offset[1]
            center=patch(x,y,3)
            flanks=[patch(round(x+sign*nx),round(y+sign*ny),1) for sign in (-1,1)]
            backgrounds=[flank.most_common(1)[0][0] for flank in flanks]
            candidates={ink for ink,count in center.items() if count>=2 and
                        all(max(abs(c-bg) for c,bg in zip(ink,background))>=8 for background in backgrounds)}
            common=candidates if common is None else common & candidates
            assert common, 'Visible route stroke missing or indistinguishable at an upstream projected sample'
            samples.append({'x':x,'y':y})
    assert samples, 'Route requires projected segments'
    return {'samples':samples,'stroke_inks':[list(c) for c in sorted(common)],
            'projection':'pinned ChartCanvas::GetCanvasPointPix',
            'scope':'Visible contrasting route stroke; no prototype palette comparison'}


def interior(rgb):
    assert len(rgb) == 1280 * 800 * 3
    return Counter(bytes(rgb[(y * 1280 + x) * 3:(y * 1280 + x) * 3 + 3])
                   for y in range(150, 630, 4) for x in range(120, 1050, 4))


def reference(rgb):
    counts = interior(rgb)
    colors = [color for color, count in counts.most_common(2)]
    assert len(colors) == 2 and all(counts[c] > sum(counts.values()) * .05 for c in colors), \
        'Initial coastline fixture must contain substantial land and water'
    return colors


def check(rgb, colors, phase):
    counts = interior(rgb)
    fractions = {c.hex(): counts[c] / sum(counts.values()) for c in colors}
    assert all(f > .02 for f in fractions.values()), \
        f'{phase}: coastline disappeared or chart canvas is covered: {fractions}'
    return {'phase': phase, 'reference_color_fractions': fractions,
            'coastline_visible': True}


def night(rgb, day_colors, phase):
    """Pinned GSHHS SetColorScheme NIGHT multiplies land and water by 0.25."""
    colors = [bytes(int(channel * .25) for channel in color) for color in day_colors]
    return check(rgb, colors, phase)


def functional(rgb, style, light, phase):
    """Require visible land/water areas without comparing a design palette.

    The disconnected coastline fixture fixes geographic extent. Require two
    substantial, distinguishable chart inks in its interior; a blank/water-only
    canvas fails. Native startup and navigation-data checks remain separate.
    """
    colors=reference(rgb)
    assert max(abs(a-b) for a,b in zip(*colors))>=8, 'Land and water must remain distinguishable'
    result=check(rgb,colors,phase)
    result.update(style=style,light=light,source='Functional chart interior coverage and contrast; no design palette comparison')
    return result


def presentation(rgb, style, light, phase):
    """Exact GSHHS ink; custom XNav and stock fallback are separate contracts.

    XNav expectations come from the independently rendered immutable HTML.
    Standard software colors and dimming are pinned GSHHSChart::SetColorScheme.
    Neither a changed palette nor a blank chart is learned as a new baseline.
    """
    assert style in ('XNav','Standard') and light in ('Day','Dusk','Night')
    if style=='XNav':
        import json
        from pathlib import Path
        tokens=json.loads((Path(__file__).resolve().parents[1]/'docs/design/prototype-tokens.json').read_text())['themes'][light.lower()]
        colors=[bytes.fromhex(tokens[key].lstrip('#')) for key in ('--land','--water')]
        if light=='Night':
            # Final .chart-canvas brightness, independently confirmed in the
            # canonical Windows Night reference: land1d2925 / water0e171c.
            colors=[bytes(round(c*.78) for c in rgb) for rgb in colors]
    else:
        dim={'Day':1,'Dusk':.5,'Night':.25}[light]
        colors=[bytes(int(c*dim) for c in rgb) for rgb in ((170,175,80),(170,195,240))]
    result=check(rgb,colors,phase)
    result.update(style=style,light=light,source='immutable HTML effective chart-canvas colors' if style=='XNav' else 'pinned software GSHHSChart::SetColorScheme')
    return result


def dark_surface(rgb, phase):
    """Primary client area only; native OS captions/file dialogs are separate."""
    assert len(rgb) == 1280 * 800 * 3
    pixels = [rgb[(y * 1280 + x) * 3:(y * 1280 + x) * 3 + 3]
              for y in range(110, 730, 4) for x in range(8, 1272, 4)]
    bright = sum(min(p) > 210 for p in pixels) / len(pixels)
    assert bright < .005, f'{phase}: unexpected bright primary surface {bright:.3%}'
    return {'phase': phase, 'bright_fraction': bright, 'scope': 'Primary client area; OS caption/file picker excluded'}

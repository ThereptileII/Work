"""Check actual pinned OpenCPN coastline pixels across a fixed demo view.

This is a rendering regression fixture, not chart/sensor data or a replacement
renderer. Learn the two dominant interior colors from the first native Day
capture; both must remain present after restart. A water-only canvas fails.
"""
from collections import Counter


def navigation_layout(display, frame, client):
    """Verify chart-first geometry against actual native frame/client bounds.

    Windows decorations reduce the chart's height relative to bare Xvfb. An
    exact outer size plus contained, dominant chart/rail/control regions is the
    cross-platform contract; a Linux-specific chart-height constant is not.
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

    assert valid(frame) and (frame['width'], frame['height']) == (1280, 800), 'Native frame must be exactly 1280x800'
    assert valid(client) and contains(frame, client), 'Actual client must fit its native frame'
    chart = display.get('chart_region', {})
    assert valid(chart) and contains(client, chart), 'Chart must fit the actual client'
    assert chart['width'] >= client['width'] * .75 and chart['height'] >= client['height'] * .75, 'Chart must dominate both client dimensions'
    assert chart['width'] * chart['height'] >= client['width'] * client['height'] * .60, 'Chart must occupy most client area'
    rail = display.get('rail_regions', [])
    assert len(rail) == 4 and len({r['label'] for r in rail}) == 4, 'Four distinct primary rail values required'
    for i, region in enumerate(rail):
        assert valid(region) and region.get('visible') and contains(client, region), 'Every primary rail value must be fully visible'
        assert region['height'] >= 48 and region['width'] >= 48, 'Primary rail values must remain readable'
        assert not overlaps(chart, region) and all(not overlaps(region, other) for other in rail[:i]), 'Chart and rail values must not overlap'
    controls = [c for c in display.get('interaction_controls', []) if c.get('visible')]
    for control in controls:
        assert valid(control) and contains(client, control), 'Visible control must fit the actual client'
        assert not overlaps(chart, control), 'Permanent controls must not cover the chart'
    for label in ('Navigation', 'System'):
        required = [c for c in controls if c['label'] == label]
        assert len(required) == 1 and required[0].get('enabled'), 'Bottom navigation controls must be uniquely visible and enabled'
        assert required[0]['y'] >= chart['y'] + chart['height'], 'Bottom controls must remain below the chart'
    return {'frame': frame, 'client': client, 'chart': chart,
            'chart_client_area_fraction': chart['width'] * chart['height'] / (client['width'] * client['height']),
            'primary_rail_values_visible': 4, 'controls_do_not_cover_chart': True}


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


def dark_surface(rgb, phase):
    """Primary client area only; native OS captions/file dialogs are separate."""
    assert len(rgb) == 1280 * 800 * 3
    pixels = [rgb[(y * 1280 + x) * 3:(y * 1280 + x) * 3 + 3]
              for y in range(110, 730, 4) for x in range(8, 1272, 4)]
    bright = sum(min(p) > 210 for p in pixels) / len(pixels)
    assert bright < .005, f'{phase}: unexpected bright primary surface {bright:.3%}'
    return {'phase': phase, 'bright_fraction': bright, 'scope': 'Primary client area; OS caption/file picker excluded'}

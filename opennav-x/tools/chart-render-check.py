"""Check actual pinned OpenCPN coastline pixels across a fixed demo view.

This is a rendering regression fixture, not chart/sensor data or a replacement
renderer. Learn the two dominant interior colors from the first native Day
capture; both must remain present after restart. A water-only canvas fails.
"""
from collections import Counter


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

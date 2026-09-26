"""Native product-control geometry helpers for isolated UI smoke tests."""
import time


def control(read, label, scroll, *, enabled=True, timeout=18):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        display = read().get('runtime', {}).get('display', {})
        items = display.get('product_controls', [])
        last = next((item for item in items if item['label'] == label), None)
        if last and last['visible']:
            if enabled is None or last['enabled'] == enabled:
                assert last['width'] >= 48 and last['height'] >= 48, last
                return last
        elif last:
            # Product pages have fixed chrome above and below. Wheel over their
            # central content until the real native rectangle is fully visible.
            scroll(-1 if last['y'] < 200 else 1)
        time.sleep(.25)
    raise AssertionError(('Product control did not become usable', label, last))


def grouped_regions(read):
    display = read()['runtime']['display']
    regions = display.get('product_regions', [])
    assert regions, 'No grouped numeric regions reported'
    assert regions[0]['visible'], ('Primary numeric region is clipped', regions[0])
    for region in regions:
        assert region['width'] >= 280, region
        assert region['height'] >= 168, region
    assert display['minimum_value_height_dip'] >= 120, display
    return regions

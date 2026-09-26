"""Pure geometry contracts; these do not claim native rendering acceptance."""
import copy
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('chartcheck', Path(__file__).resolve().parents[1] / 'tools/chart-render-check.py')
chartcheck = importlib.util.module_from_spec(spec)
spec.loader.exec_module(chartcheck)


def rect(x, y, width, height, label=None):
    value = dict(x=x, y=y, width=width, height=height)
    if label:
        value.update(label=label, visible=True, enabled=True)
    return value


frame = rect(0, 0, 1280, 800)
client = rect(8, 31, 1264, 761)
native = {
    'chart_region': rect(65, 88, 1070, 647),
    'rail_regions': [rect(1136, 87 + i * 162, 136, 162, name)
                     for i, name in enumerate(('sog', 'depth', 'aws', 'heading'))],
    'interaction_controls': [rect(12, 740, 112, 48, 'Navigation'),
                             rect(1155, 740, 112, 48, 'System'),
                             rect(1203, 35, 64, 48, 'Menu')],
}
assert chartcheck.navigation_layout(native, frame, client)['primary_rail_values_visible'] == 4
linux = copy.deepcopy(native)
linux['chart_region'] = rect(57, 57, 1086, 686)
linux['rail_regions'] = [rect(1144, 56 + i * 172, 136, 172, name)
                         for i, name in enumerate(('sog', 'depth', 'aws', 'heading'))]
linux['interaction_controls'] = [rect(4, 748, 112, 48, 'Navigation'),
                                 rect(1164, 748, 112, 48, 'System')]
chartcheck.navigation_layout(linux, frame, frame)


def reject(display, outer=frame, inner=client):
    try:
        chartcheck.navigation_layout(display, outer, inner)
    except AssertionError:
        return
    raise AssertionError('Invalid startup geometry was accepted')


# Actual c8ff99ea native failure: desktop 1280x800, application still 896x532.
reject(native, rect(0, 0, 896, 532), rect(8, 31, 880, 493))
small = copy.deepcopy(native)
small['chart_region'] = rect(65, 88, 686, 379)
reject(small)  # An outer resize alone is insufficient; wx layout must catch up.
for change in ('chart-outside', 'rail-hidden', 'rail-clipped', 'rail-overlap',
               'rail-missing', 'control-chart', 'control-clipped', 'system-missing'):
    bad = copy.deepcopy(native)
    if change == 'chart-outside': bad['chart_region']['x'] = 300
    if change == 'rail-hidden': bad['rail_regions'][3]['visible'] = False
    if change == 'rail-clipped': bad['rail_regions'][3]['y'] = 750
    if change == 'rail-overlap': bad['rail_regions'][3]['y'] = bad['rail_regions'][2]['y']
    if change == 'rail-missing': bad['rail_regions'].pop()
    if change == 'control-chart': bad['interaction_controls'][0]['y'] = 300
    if change == 'control-clipped': bad['interaction_controls'][1]['x'] = 1250
    if change == 'system-missing': bad['interaction_controls'].pop(1)
    reject(bad)
print('Native 647px and Linux chart geometry pass; small/unsettled/clipped/overlapping layouts fail')

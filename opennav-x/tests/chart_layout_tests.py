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
def composition(client):
    x,y,w,h = [client[k] for k in ('x','y','width','height')]
    right,bottom = x+w-186,y+h-166
    tx,ty = right-211,bottom-89
    return {
        'chart_region': rect(x+80,y+68,w-266,h-234),
        'rail_regions': [rect(right,y+110+i*120,186,120,name)
                         for i,name in enumerate(('sog','depth','aws','heading'))],
        'interaction_controls': [rect(x+9,y+82,62,61,'Chart'),
             rect(x+w-102,y+h-34,90,32,'System'),
             rect(tx+4,ty+4,44,44,'Measure'),rect(tx+48,ty+4,44,44,'Waypoint'),
             rect(tx+97,ty+4,44,44,'+'),rect(tx+141,ty+4,44,44,'−'),
             rect(right-90,y+90,68,90,'North'),
             rect(x+108,bottom-81,142,44,'Follow boat')],
    }

native = composition(client)
assert chartcheck.navigation_layout(native, frame, client)['primary_rail_values_visible'] == 4
linux = composition(frame)
chartcheck.navigation_layout(linux, frame, frame)
primary_client = rect(8,31,1280,800)
chartcheck.navigation_layout(composition(primary_client),rect(0,0,1296,839),primary_client)


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
               'rail-missing', 'control-chart', 'control-clipped', 'system-missing',
               'float-missing', 'float-duplicate', 'float-moved', 'float-oversize',
               'rail-duplicate', 'rail-outside-strip', 'orientation-missing', 'old-layout'):
    bad = copy.deepcopy(native)
    if change == 'chart-outside': bad['chart_region']['x'] = 300
    if change == 'rail-hidden': bad['rail_regions'][3]['visible'] = False
    if change == 'rail-clipped': bad['rail_regions'][3]['y'] = 750
    if change == 'rail-overlap': bad['rail_regions'][3]['y'] = bad['rail_regions'][2]['y']
    if change == 'rail-missing': bad['rail_regions'].pop()
    if change == 'control-chart': bad['interaction_controls'][0]['x'] = 300
    if change == 'control-clipped': bad['interaction_controls'][1]['x'] = 1250
    if change == 'system-missing': bad['interaction_controls'].pop(1)
    if change == 'float-missing': bad['interaction_controls'].pop(2)
    if change == 'float-duplicate': bad['interaction_controls'].append(copy.deepcopy(bad['interaction_controls'][2]))
    if change == 'float-moved': bad['interaction_controls'][2]['x'] -= 10
    if change == 'float-oversize': bad['interaction_controls'][2]['width'] += 10
    if change == 'rail-duplicate': bad['rail_regions'][3]['label'] = 'sog'
    if change == 'rail-outside-strip': bad['rail_regions'][3]['x'] -= 10
    if change == 'orientation-missing': bad['interaction_controls'].pop(6)
    if change == 'old-layout': bad['chart_region'] = rect(65,88,1070,647)
    reject(bad)
print('Prototype native-client/outer and Linux composition pass; 18 small/unsettled/clipped/overlapping/obsolete layouts rejected')
